// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

/*
 * This version has optimized assembly routines for 32 bit operations:
 * - read word
 * - write word
 * - write array of words
 *
 * One thing to be aware of is that the MIPS32 cpu will execute the
 * instruction after a branch instruction (one delay slot).
 *
 * For example:
 *  LW $2, ($5 +10)
 *  B foo
 *  LW $1, ($2 +100)
 *
 * The LW $1, ($2 +100) instruction is also executed. If this is
 * not wanted a NOP can be inserted:
 *
 *  LW $2, ($5 +10)
 *  B foo
 *  NOP
 *  LW $1, ($2 +100)
 *
 * or the code can be changed to:
 *
 *  B foo
 *  LW $2, ($5 +10)
 *  LW $1, ($2 +100)
 *
 * The original code contained NOPs. I have removed these and moved
 * the branches.
 *
 * These changes result in a 35% speed increase when programming an
 * external flash.
 *
 * More improvement could be gained if the registers do no need
 * to be preserved but in that case the routines should be aware
 * OpenOCD is used as a flash programmer or as a debug tool.
 *
 * Nico Coesel
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/align.h>
#include <helper/time_support.h>
#include <jtag/adapter.h>

#include "mips32.h"
#include "mips32_pracc.h"

/* Fastdata scans queued per adapter round trip when draining the handler. */
#define MT7628_DRAIN_CHUNK 1024


static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info)
{
	int64_t then = timeval_ms();

	/* wait for the PrAcc to become "1" */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	while (1) {
		ejtag_info->pa_ctrl = ejtag_info->ejtag_ctrl;
		int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_ctrl);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("stock wait_for_pracc_rw: read ctrl=0x%8.8" PRIx32
				" (wrote 0x%8.8" PRIx32 ")",
				ejtag_info->pa_ctrl, ejtag_info->ejtag_ctrl);

		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC)
			break;

		int64_t timeout = timeval_ms() - then;
		if (timeout > 1000) {
			LOG_DEBUG("DEBUGMODULE: No memory access in progress!");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return ERROR_OK;
}

/* Shift in control and address for a new processor access, save them in ejtag_info */
static int mips32_pracc_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
	int retval = wait_for_pracc_rw(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);

	ejtag_info->pa_addr = 0;
	return  mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);
}

/* Finish processor access */
static void mips32_pracc_finish(struct mips_ejtag *ejtag_info)
{
	uint32_t ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	LOG_DEBUG("stock pracc_finish: writing ctrl=0x%8.8" PRIx32 " (Rocc=%d)",
			ctrl, (ctrl & EJTAG_CTRL_ROCC) ? 1 : 0);
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ejtag_info, ctrl);
}

static int mips32_pracc_clean_text_jump(struct mips_ejtag *ejtag_info)
{
	uint32_t jt_code = MIPS32_J(ejtag_info->isa, MIPS32_PRACC_TEXT);
	pracc_swap16_array(ejtag_info, &jt_code, 1);
	/* do 3 0/nops to clean pipeline before a jump to pracc text, NOP in delay slot */
	for (int i = 0; i != 5; i++) {
		/* Wait for pracc */
		int retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		/* Data or instruction out */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		uint32_t data = (i == 3) ? jt_code : MIPS32_NOP;
		mips_ejtag_drscan_32_out(ejtag_info, data);

		/* finish pa */
		mips32_pracc_finish(ejtag_info);
	}

	if (ejtag_info->mode != 0)	/* async mode support only for MIPS ... */
		return ERROR_OK;

	for (int i = 0; i != 2; i++) {
		int retval = mips32_pracc_read_ctrl_addr(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) {	/* LEXRA/BMIPS ?, shift out another NOP, max 2 */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);
			mips32_pracc_finish(ejtag_info);
		} else
			break;
	}

	return ERROR_OK;
}

static int mips32_pracc_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx,
					uint32_t *param_out, bool check_last)
{
	int code_count = 0;
	int store_pending = 0;		/* increases with every store instr at dmseg, decreases with every store pa */
	uint32_t max_store_addr = 0;	/* for store pa address testing */
	bool restart = 0;		/* restarting control */
	int restart_count = 0;
	uint32_t instr = 0;
	bool final_check = 0;		/* set to 1 if in final checks after function code shifted out */
	bool pass = 0;			/* to check the pass through pracc text after function code sent */
	int retval;

	while (1) {
		if (restart) {
			if (restart_count < 3) {					/* max 3 restarts allowed */
				retval = mips32_pracc_clean_text_jump(ejtag_info);
				if (retval != ERROR_OK)
					return retval;
			} else
				return ERROR_JTAG_DEVICE_ERROR;
			restart_count++;
			restart = 0;
			code_count = 0;
			LOG_DEBUG("restarting code");
		}

		retval = mips32_pracc_read_ctrl_addr(ejtag_info); /* update current pa info: control and address */
		if (retval != ERROR_OK)
			return retval;

		/* Check for read or write access */
		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRNW) {				/* write/store access */
			/* Check for pending store from a previous store instruction at dmseg */
			if (store_pending == 0) {
				LOG_DEBUG("unexpected write at address %" PRIx32
					" ctrl=0x%8.8" PRIx32,
					ejtag_info->pa_addr, ejtag_info->pa_ctrl);
				if (code_count < 2) {	/* allow for restart */
					restart = 1;
					continue;
				} else
					return ERROR_JTAG_DEVICE_ERROR;
			} else {
				/* check address */
				if (ejtag_info->pa_addr < MIPS32_PRACC_PARAM_OUT ||
						ejtag_info->pa_addr > max_store_addr) {
					LOG_DEBUG("writing at unexpected address %" PRIx32, ejtag_info->pa_addr);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			/* read data */
			uint32_t data = 0;
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			retval = mips_ejtag_drscan_32(ejtag_info, &data);
			if (retval != ERROR_OK)
				return retval;

			/* store data at param out, address based offset */
			param_out[(ejtag_info->pa_addr - MIPS32_PRACC_PARAM_OUT) / 4] = data;
			store_pending--;

		} else {					/* read/fetch access */
			 if (!final_check) {			/* executing function code */
				/* check address */
				if (ejtag_info->pa_addr != (MIPS32_PRACC_TEXT + code_count * 4)) {
					LOG_DEBUG("reading at unexpected address %" PRIx32 ", expected %x",
							ejtag_info->pa_addr, MIPS32_PRACC_TEXT + code_count * 4);

					/* restart code execution only in some cases */
					if (code_count == 1 && ejtag_info->pa_addr == MIPS32_PRACC_TEXT &&
										restart_count == 0) {
						LOG_DEBUG("restarting, without clean jump");
						restart_count++;
						code_count = 0;
						continue;
					} else if (code_count < 2) {
						restart = 1;
						continue;
					}
					return ERROR_JTAG_DEVICE_ERROR;
				}
				/* check for store instruction at dmseg */
				uint32_t store_addr = ctx->pracc_list[code_count].addr;
				if (store_addr != 0) {
					if (store_addr > max_store_addr)
						max_store_addr = store_addr;
					store_pending++;
				}

				instr = ctx->pracc_list[code_count++].instr;
				if (code_count == ctx->code_count)	/* last instruction, start final check */
					final_check = 1;

			 } else {	/* final check after function code shifted out */
					/* check address */
				if (ejtag_info->pa_addr == MIPS32_PRACC_TEXT) {
					if (!pass) {	/* first pass through pracc text */
						if (store_pending == 0)		/* done, normal exit */
							return ERROR_OK;
						pass = 1;		/* pracc text passed */
						code_count = 0;		/* restart code count */
					} else {
						LOG_DEBUG("unexpected second pass through pracc text");
						return ERROR_JTAG_DEVICE_ERROR;
					}
				} else {
					if (ejtag_info->pa_addr != (MIPS32_PRACC_TEXT + code_count * 4)) {
						LOG_DEBUG("unexpected read address in final check: %"
							PRIx32 ", expected: %x", ejtag_info->pa_addr,
							MIPS32_PRACC_TEXT + code_count * 4);
						return ERROR_JTAG_DEVICE_ERROR;
					}
				}
				if (!pass) {
					if ((code_count - ctx->code_count) > 1) { /* allow max 2 instr delay slot */
						LOG_DEBUG("failed to jump back to pracc text");
						return ERROR_JTAG_DEVICE_ERROR;
					}
				} else
					if (code_count > 10) {		/* enough, abandon */
						LOG_DEBUG("execution abandoned, store pending: %d", store_pending);
						return ERROR_JTAG_DEVICE_ERROR;
					}
				instr = MIPS32_NOP;	/* shift out NOPs instructions */
				code_count++;
			 }

			/* Send instruction out */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, instr);
		}
		/* finish processor access, let the processor eat! */
		mips32_pracc_finish(ejtag_info);

		if (final_check && !check_last)			/* last instr, don't check, execute and exit */
			return jtag_execute_queue();

		if (store_pending == 0 && pass) {	/* store access done, but after passing pracc text */
			LOG_DEBUG("warning: store access pass pracc text");
			return ERROR_OK;
		}
	}
}

inline void pracc_queue_init(struct pracc_queue_info *ctx)
{
	ctx->retval = ERROR_OK;
	ctx->code_count = 0;
	ctx->store_count = 0;
	ctx->max_code = 0;
	ctx->pracc_list = NULL;
	ctx->isa = ctx->ejtag_info->isa ? 1 : 0;
}

void pracc_add(struct pracc_queue_info *ctx, uint32_t addr, uint32_t instr)
{
	if (ctx->retval != ERROR_OK)	/* On previous out of memory, return */
		return;
	if (ctx->code_count == ctx->max_code) {
		void *p = realloc(ctx->pracc_list, sizeof(struct pa_list) * (ctx->max_code + PRACC_BLOCK));
		if (p) {
			ctx->max_code += PRACC_BLOCK;
			ctx->pracc_list = p;
		} else {
			ctx->retval = ERROR_FAIL;	/* Out of memory */
			return;
		}
	}
	ctx->pracc_list[ctx->code_count].instr = instr;
	ctx->pracc_list[ctx->code_count++].addr = addr;
	if (addr)
		ctx->store_count++;
}

static void pracc_add_li32(struct pracc_queue_info *ctx, uint32_t reg_num, uint32_t data, bool optimize)
{
	if (LOWER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_LUI(ctx->isa, reg_num, UPPER16(data)));	/* load only upper value */
	else if (UPPER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_ORI(ctx->isa, reg_num, 0, LOWER16(data)));	/* load only lower */
	else {
		pracc_add(ctx, 0, MIPS32_LUI(ctx->isa, reg_num, UPPER16(data)));	/* load upper and lower */
		pracc_add(ctx, 0, MIPS32_ORI(ctx->isa, reg_num, reg_num, LOWER16(data)));
	}
}

inline void pracc_queue_free(struct pracc_queue_info *ctx)
{
	free(ctx->pracc_list);
}

static int mt7628_pracc_all_scan(struct mips_ejtag *ejtag_info,
		uint32_t ctrl_out, uint32_t data_out,
		uint32_t *ctrl_in, uint32_t *data_in, uint32_t *addr_in)
{
	uint8_t scan[12] = { 0 };

	mips_ejtag_add_scan_96(ejtag_info, ctrl_out, data_out, scan);
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (ctrl_in)
		*ctrl_in = buf_get_u32(scan, 0, 32);
	if (data_in)
		*data_in = buf_get_u32(scan + 4, 0, 32);
	if (addr_in)
		*addr_in = buf_get_u32(scan + 8, 0, 32);

	return ERROR_OK;
}

/*
 * Select an EJTAG instruction on MT7628 as two physically executed IR scans.
 *
 * Two separate requirements are covered here.  First, OpenOCD's cached
 * tap->cur_instr cannot be trusted on this target, so the transition is
 * forced through BYPASS rather than skipped when the cache claims the
 * instruction is already selected.  Second, each IR scan must be flushed as
 * its own adapter transaction: batching an IR select into the same
 * jtag_execute_queue() as the following DR scan reproduces the corrupted
 * readback that this whole quirk exists to avoid (see the comment in
 * mips_ejtag_drscan_96()).
 *
 * BYPASS is harmless to a pending processor access because no DR transaction
 * is issued while it is selected.
 */
static int mt7628_pracc_force_ir(struct mips_ejtag *ejtag_info, uint32_t instr)
{
	struct scan_field field;
	uint8_t ir_out[4] = { 0 };
	int retval;

	field.num_bits = ejtag_info->tap->ir_length;
	field.out_value = ir_out;
	field.in_value = NULL;

	buf_set_u32(ir_out, 0, field.num_bits,
			(1U << ejtag_info->tap->ir_length) - 1U);
	jtag_add_ir_scan(ejtag_info->tap, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	buf_set_u32(ir_out, 0, field.num_bits, instr);
	jtag_add_ir_scan(ejtag_info->tap, &field, TAP_IDLE);

	return jtag_execute_queue();
}

static int mt7628_pracc_force_all_ir(struct mips_ejtag *ejtag_info)
{
	return mt7628_pracc_force_ir(ejtag_info, EJTAG_INST_ALL);
}

static bool mt7628_is_deret(uint32_t instr)
{
	return instr == MIPS32_DRET(0);
}

static bool mt7628_is_unconditional_b(uint32_t instr)
{
	/*
	 * MIPS32 "b offset" is encoded as beq $zero,$zero,offset.
	 * MT7628 is forced to ordinary MIPS32 ISA in the target quirk path.
	 */
	return (instr & 0xffff0000U) == 0x10000000U;
}

static uint32_t mt7628_rewrite_b_to_text(uint32_t branch_pc)
{
	int32_t delta = (int32_t)MIPS32_PRACC_TEXT -
			(int32_t)(branch_pc + 4);
	int32_t words = delta / 4;

	return 0x10000000U | ((uint32_t)words & 0xffffU);
}

/*
 * Drain zero-valued ALL scans until the pending processor access is the fetch
 * at PRACC_TEXT.  Same job as the entry prime inside mt7628_pracc_exec(), in a
 * form the FASTDATA path can reuse.
 */
static int mt7628_pracc_prime_at_text(struct mips_ejtag *ejtag_info)
{
	for (unsigned int attempt = 0; attempt < 32; attempt++) {
		uint32_t ctrl = 0, data = 0, addr = 0;

		int retval = mt7628_pracc_all_scan(ejtag_info, 0, 0,
				&ctrl, &data, &addr);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("mt7628 prime[%u]: ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32, attempt, ctrl, addr);

		if ((ctrl & EJTAG_CTRL_PRACC) && !(ctrl & EJTAG_CTRL_PRNW) &&
				addr == MIPS32_PRACC_TEXT)
			return ERROR_OK;

		if (ctrl == ejtag_info->tap->idcode && addr == 0) {
			retval = mt7628_pracc_force_all_ir(ejtag_info);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	LOG_ERROR("mt7628: no PRACC_TEXT fetch to prime on");
	return ERROR_FAIL;
}

static int mt7628_pracc_exec_one(struct mips_ejtag *ejtag_info,
		uint32_t service_ctrl, uint32_t instr, uint32_t pc,
		uint32_t store_addr, uint32_t *buf)
{
	uint32_t ctrl;
	uint32_t data;
	uint32_t addr;

	/*
	 * Service the current instruction fetch with a real instruction.
	 */
	int retval = mt7628_pracc_all_scan(ejtag_info, service_ctrl, instr,
			&ctrl, &data, &addr);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("mt7628 PRACC service: pc=0x%8.8" PRIx32
			" instr=0x%8.8" PRIx32
			" -> ctrl=0x%8.8" PRIx32
			" data=0x%8.8" PRIx32
			" addr=0x%8.8" PRIx32,
			pc, instr, ctrl, data, addr);

	if (!(ctrl & EJTAG_CTRL_PRACC) ||
			(ctrl & EJTAG_CTRL_PRNW) ||
			addr != pc) {
		LOG_ERROR("mt7628 service mismatch: ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32
				" expected fetch=0x%8.8" PRIx32,
				ctrl, addr, pc);
		return ERROR_FAIL;
	}

	/*
	 * Mandatory MT7628 interlock.  DATA must be zero.  This services the
	 * sequential fetch with a NOP and advances the processor pipeline.
	 */
	retval = mt7628_pracc_all_scan(ejtag_info, 0, 0,
			&ctrl, &data, &addr);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("mt7628 PRACC interlock: ctrl=0x%8.8" PRIx32
			" data=0x%8.8" PRIx32
			" addr=0x%8.8" PRIx32,
			ctrl, data, addr);

	if (!(ctrl & EJTAG_CTRL_PRACC) ||
			(ctrl & EJTAG_CTRL_PRNW) ||
			addr != pc + 4) {
		LOG_ERROR("mt7628 interlock mismatch: ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32
				" expected fetch=0x%8.8" PRIx32,
				ctrl, addr, pc + 4);
		return ERROR_FAIL;
	}

	if (!store_addr)
		return ERROR_OK;

	/*
	 * A store generated by the real instruction appears one transaction
	 * after the interlock fetch.
	 */
	retval = mt7628_pracc_all_scan(ejtag_info, 0, 0,
			&ctrl, &data, &addr);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("mt7628 PRACC store: ctrl=0x%8.8" PRIx32
			" data=0x%8.8" PRIx32
			" addr=0x%8.8" PRIx32,
			ctrl, data, addr);

	if (!(ctrl & EJTAG_CTRL_PRACC) ||
			!(ctrl & EJTAG_CTRL_PRNW) ||
			addr != store_addr) {
		LOG_ERROR("mt7628 store mismatch: ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32
				" expected store=0x%8.8" PRIx32,
				ctrl, addr, store_addr);
		return ERROR_FAIL;
	}

	if (buf) {
		if (addr < MIPS32_PRACC_PARAM_OUT) {
			LOG_ERROR("mt7628 store below PARAM_OUT: 0x%8.8" PRIx32,
					addr);
			return ERROR_FAIL;
		}
		int buf_index = (addr - MIPS32_PRACC_PARAM_OUT) / 4;
		buf[buf_index] = data;
	}

	return ERROR_OK;
}

/*
 * Walk the core out of dmseg and into the FASTDATA write handler in DRAM.
 *
 * Replaces the stock jump-code loop, which drives the standalone CONTROL
 * register and so cannot complete an access on this part.
 *
 * Three instructions, not four: the interlock services each following
 * sequential fetch with a NOP, and for JR that NOP is the branch delay slot,
 * so the stock explicit delay-slot NOP would be fetched from the handler.
 */
static int mt7628_fastdata_enter_handler(struct mips_ejtag *ejtag_info,
		uint32_t handler_addr)
{
	if (ejtag_info->isa) {
		LOG_ERROR("mt7628 fastdata: MIPS32 ISA only");
		return ERROR_FAIL;
	}

	uint32_t jmp_code[] = {
		MIPS32_LUI(0, 15, UPPER16(handler_addr)),
		MIPS32_ORI(0, 15, 15, LOWER16(handler_addr)),
		MIPS32_JR(0, 15),
	};

	int retval = mt7628_pracc_force_all_ir(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	retval = mt7628_pracc_prime_at_text(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	uint32_t service_ctrl =
			ejtag_info->ejtag_ctrl & ~(EJTAG_CTRL_PRACC | EJTAG_CTRL_ROCC);

	/* The prime consumed the fetch at PRACC_TEXT; the first real
	 * instruction is serviced one word on, and the PC then advances by 8
	 * per instruction because of the interlock NOP. */
	uint32_t pc = MIPS32_PRACC_TEXT + 4;

	for (unsigned int i = 0; i < ARRAY_SIZE(jmp_code); i++) {
		retval = mt7628_pracc_exec_one(ejtag_info, service_ctrl,
				jmp_code[i], pc, 0, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("mt7628 fastdata: jump code failed at index %u", i);
			return retval;
		}
		pc += 8;
	}

	LOG_DEBUG("mt7628 fastdata: jumped to handler at 0x%8.8" PRIx32,
			handler_addr);
	return ERROR_OK;
}

/*
 * Walk the core back to PRACC_TEXT.
 *
 * There is no memory behind dmseg: the core executes whatever the probe hands
 * it, one instruction per processor access, and its PC only ever advances.  A
 * core that has drifted off PRACC_TEXT therefore cannot return on its own -
 * the probe has to feed it a branch.  Stock OpenOCD does this in
 * mips32_pracc_clean_text_jump(); the MT7628 executor needs the same thing,
 * with the mandatory interlock scan after every supplied instruction.
 *
 * Three NOPs drain the pipeline before the branch, matching stock.  The
 * interlock scan that follows each supplied instruction provides the branch
 * delay slot, so nothing extra is needed for it.
 *
 * @param pc address of the pending fetch, i.e. where the core will execute the
 *           first NOP of this sequence.
 */
static int mt7628_pracc_jump_to_text(struct mips_ejtag *ejtag_info,
		uint32_t service_ctrl, uint32_t pc)
{
	LOG_DEBUG("mt7628 PRACC: steering core from 0x%8.8" PRIx32
			" back to PRACC_TEXT", pc);

	for (unsigned int i = 0; i < 3; i++) {
		int retval = mt7628_pracc_exec_one(ejtag_info, service_ctrl,
				MIPS32_NOP, pc, 0, NULL);
		if (retval != ERROR_OK)
			return retval;

		/* Each supplied instruction costs two fetch slots: the
		 * instruction itself and the interlock NOP after it. */
		pc += 8;
	}

	return mt7628_pracc_exec_one(ejtag_info, service_ctrl,
			mt7628_rewrite_b_to_text(pc), pc, 0, NULL);
}

static int mt7628_pracc_exec(struct mips_ejtag *ejtag_info,
		struct pracc_queue_info *ctx, uint32_t *buf, bool check_last)
{
	if (ejtag_info->isa) {
		LOG_ERROR("mt7628 interlocked PRACC currently supports MIPS32 only");
		return ERROR_FAIL;
	}

	if (ctx->code_count <= 0)
		return ERROR_OK;

	int retval = mt7628_pracc_force_all_ir(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	uint32_t service_ctrl =
			ejtag_info->ejtag_ctrl &
			~(EJTAG_CTRL_PRACC | EJTAG_CTRL_ROCC);

	/*
	 * Entry prime: consume the pending fetch at PRACC_TEXT as a NOP.
	 * MT7628 requires this transaction to be completely zero-valued;
	 * using service_ctrl here reproduces the corrupted standalone-like
	 * ffffc008/ffffffff/ffffffff response.
	 */
	uint32_t ctrl;
	uint32_t data;
	uint32_t addr;
	bool prime_ok = false;
	unsigned int jump_count = 0;
	for (unsigned int attempt = 0; attempt < 32; attempt++) {
		ctrl = 0;
		data = 0;
		addr = 0;

		retval = mt7628_pracc_all_scan(ejtag_info, 0, 0,
				&ctrl, &data, &addr);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("mt7628 PRACC entry prime[%u]: ctrl=0x%8.8" PRIx32
				" data=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32,
				attempt, ctrl, data, addr);

		if ((ctrl & EJTAG_CTRL_PRACC) &&
				!(ctrl & EJTAG_CTRL_PRNW) &&
				addr == MIPS32_PRACC_TEXT) {
			prime_ok = true;
			break;
		}

		/*
		 * If the DR scan hit IDCODE instead of ALL, no CPU PRACC request
		 * was acknowledged. Re-establish the physical IR selection.
		 */
		if (ctrl == ejtag_info->tap->idcode && addr == 0) {
			LOG_DEBUG("mt7628 PRACC entry prime hit IDCODE; reselecting BYPASS -> ALL");
			retval = mt7628_pracc_force_all_ir(ejtag_info);
			if (retval != ERROR_OK)
				return retval;
			continue;
		}

		/*
		 * PRACC clear with no address is stale/empty ALL pipeline state.
		 * Keep draining with zero-valued ALL scans until the new debug
		 * exception presents its first processor access.
		 */
		if (!(ctrl & EJTAG_CTRL_PRACC) && addr == 0)
			continue;

		/*
		 * A live fetch somewhere else in the dmseg code area.  The core
		 * is in Debug Mode but its PC has drifted off PRACC_TEXT - for
		 * example because a reset re-entered at the debug exception
		 * vector while a program was mid-flight, or because an earlier
		 * transfer was aborted.  Steer it back rather than failing.
		 *
		 * The scan above already completed the fetch at addr with a
		 * NOP, so the core is now pending at addr + 4.
		 */
		if ((ctrl & EJTAG_CTRL_PRACC) &&
				!(ctrl & EJTAG_CTRL_PRNW) &&
				addr >= MIPS32_PRACC_TEXT &&
				addr < MIPS32_PRACC_PARAM_OUT) {
			if (jump_count >= 3) {
				LOG_ERROR("mt7628 PRACC entry: core still off "
						"PRACC_TEXT after %u jumps, last fetch 0x%8.8" PRIx32,
						jump_count, addr);
				return ERROR_FAIL;
			}
			jump_count++;

			retval = mt7628_pracc_jump_to_text(ejtag_info,
					service_ctrl, addr + 4);
			if (retval != ERROR_OK)
				return retval;
			continue;
		}

		/*
		 * Do not silently consume an unexpected live processor access.
		 */
		LOG_ERROR("mt7628 PRACC entry encountered unexpected request:"
				" ctrl=0x%8.8" PRIx32 " data=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32,
				ctrl, data, addr);
		return ERROR_FAIL;
	}

	if (!prime_ok) {
		LOG_ERROR("mt7628 PRACC entry timed out waiting for fetch at 0x%8.8" PRIx32,
				MIPS32_PRACC_TEXT);
		return ERROR_FAIL;
	}

	uint32_t pc = MIPS32_PRACC_TEXT + 4;

	for (int i = 0; i < ctx->code_count; i++) {
		uint32_t instr = ctx->pracc_list[i].instr;

		/*
		 * mips_ejtag_exit_debug() is a one-instruction PRACC program
		 * containing DERET and check_last=false.  Like every other MT7628
		 * PRACC instruction, the service scan supplies the instruction and a
		 * following ALL=0 transaction completes/advances it.  DERET is
		 * special only because that completion scan is not expected to
		 * return another PRACC fetch: the CPU is leaving Debug Mode.
		 */
		if (mt7628_is_deret(instr)) {
			uint32_t deret_ctrl;
			uint32_t deret_data;
			uint32_t deret_addr;

			if (i + 1 != ctx->code_count || check_last) {
				LOG_ERROR("mt7628 DERET encountered in unsupported PRACC program position");
				return ERROR_FAIL;
			}

			retval = mt7628_pracc_all_scan(ejtag_info, service_ctrl, instr,
					&deret_ctrl, &deret_data, &deret_addr);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("mt7628 PRACC DERET: pc=0x%8.8" PRIx32
					" -> ctrl=0x%8.8" PRIx32 " addr=0x%8.8" PRIx32,
					pc, deret_ctrl, deret_addr);

			if (!(deret_ctrl & EJTAG_CTRL_PRACC) ||
					(deret_ctrl & EJTAG_CTRL_PRNW) ||
					deret_addr != pc) {
				LOG_ERROR("mt7628 DERET service mismatch: ctrl=0x%8.8" PRIx32
						" addr=0x%8.8" PRIx32 " expected fetch=0x%8.8" PRIx32,
						deret_ctrl, deret_addr, pc);
				return ERROR_FAIL;
			}

			uint32_t complete_ctrl;
			uint32_t complete_data;
			uint32_t complete_addr;

			retval = mt7628_pracc_all_scan(ejtag_info, 0, 0,
					&complete_ctrl, &complete_data, &complete_addr);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("mt7628 PRACC DERET completion: ctrl=0x%8.8" PRIx32
					" data=0x%8.8" PRIx32 " addr=0x%8.8" PRIx32,
					complete_ctrl, complete_data, complete_addr);

			return ERROR_OK;
		}

		if (mt7628_is_unconditional_b(instr)) {
			/*
			 * OpenOCD PRACC helpers normally end in:
			 *
			 *     b PRACC_TEXT
			 *     <useful delay-slot instruction>
			 *
			 * The mandatory MT7628 zero interlock would replace that
			 * useful delay slot.  Preserve semantics by executing the
			 * original delay-slot instruction before the branch, then
			 * use the mandatory interlock NOP as the branch delay slot.
			 */
			if (i + 1 >= ctx->code_count) {
				LOG_ERROR("mt7628 final branch has no delay-slot instruction");
				return ERROR_FAIL;
			}
			if (i + 2 != ctx->code_count) {
				LOG_ERROR("mt7628 unsupported non-final PRACC branch at logical index %d", i);
				return ERROR_FAIL;
			}

			uint32_t delay_instr = ctx->pracc_list[i + 1].instr;
			uint32_t delay_store = ctx->pracc_list[i + 1].addr;

			retval = mt7628_pracc_exec_one(ejtag_info, service_ctrl,
					delay_instr, pc, delay_store, buf);
			if (retval != ERROR_OK)
				return retval;
			pc += 8;

			uint32_t branch = mt7628_rewrite_b_to_text(pc);
			retval = mt7628_pracc_exec_one(ejtag_info, service_ctrl,
					branch, pc, 0, buf);
			if (retval != ERROR_OK)
				return retval;

			/*
			 * mt7628_pracc_exec_one() consumed the branch delay-slot
			 * fetch with the mandatory NOP. The branch target fetch at
			 * PRACC_TEXT is now pending and intentionally left
			 * untouched for the next invocation's entry prime.
			 */
			LOG_DEBUG("mt7628 PRACC program complete; PRACC_TEXT fetch left pending");
			return ERROR_OK;
		}

		retval = mt7628_pracc_exec_one(ejtag_info, service_ctrl,
				instr, pc, ctx->pracc_list[i].addr, buf);
		if (retval != ERROR_OK)
			return retval;
		pc += 8;
	}

	if (!check_last) {
		LOG_DEBUG("mt7628 PRACC completed without final branch check");
		return ERROR_OK;
	}

	LOG_ERROR("mt7628 PRACC program ended without final return branch");
	return ERROR_FAIL;
}

int mips32_pracc_queue_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx,
					uint32_t *buf, bool check_last)
{
	if (ctx->retval != ERROR_OK) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (ejtag_info->isa && ejtag_info->endianness)
		for (int i = 0; i != ctx->code_count; i++)
			ctx->pracc_list[i].instr = SWAP16(ctx->pracc_list[i].instr);

	if (mips_ejtag_control_all_quirk(ejtag_info))
		return mt7628_pracc_exec(ejtag_info, ctx, buf, check_last);

	if (ejtag_info->mode == 0)
		return mips32_pracc_exec(ejtag_info, ctx, buf, check_last);

	union scan_in {
		uint8_t scan_96[12];
		struct {
			uint8_t ctrl[4];
			uint8_t data[4];
			uint8_t addr[4];
		} scan_32;

	} *scan_in = malloc(sizeof(union scan_in) * (ctx->code_count + ctx->store_count));
	if (!scan_in) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	unsigned num_clocks =
		((uint64_t)(ejtag_info->scan_delay) * adapter_get_speed_khz() + 500000) / 1000000;

	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ALL);

	int scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {
		jtag_add_clocks(num_clocks);
		mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, ctx->pracc_list[i].instr,
				       scan_in[scan_count++].scan_96);

		/* Check store address from previous instruction, if not the first */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			jtag_add_clocks(num_clocks);
			mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, 0, scan_in[scan_count++].scan_96);
		}
	}

	int retval = jtag_execute_queue();		/* execute queued scans */
	if (retval != ERROR_OK)
		goto exit;

	uint32_t fetch_addr = MIPS32_PRACC_TEXT;		/* start address */
	scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {				/* verify every pracc access */
		/* check pracc bit */
		ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
		uint32_t addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);
		if (!(ejtag_ctrl & EJTAG_CTRL_PRACC)) {
			LOG_ERROR("Error: access not pending  count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
			LOG_ERROR("Not a fetch/read access, count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (addr != fetch_addr) {
			LOG_ERROR("Fetch addr mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
					  addr, fetch_addr, scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		fetch_addr += 4;
		scan_count++;

		/* check if previous instruction is a store instruction at dmesg */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			uint32_t store_addr = ctx->pracc_list[i - 1].addr;
			ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
			addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);

			if (!(ejtag_ctrl & EJTAG_CTRL_PRNW)) {
				LOG_ERROR("Not a store/write access, count: %d", scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			if (addr != store_addr) {
				LOG_ERROR("Store address mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
							      addr, store_addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			int buf_index = (addr - MIPS32_PRACC_PARAM_OUT) / 4;
			buf[buf_index] = buf_get_u32(scan_in[scan_count].scan_32.data, 0, 32);
			scan_count++;
		}
	}
exit:
	free(scan_in);
	return retval;
}

static int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *buf)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16((addr + 0x8000)))); /* load  $8 with modified upper addr */
	pracc_add(&ctx, 0, MIPS32_LW(ctx.isa, 8, LOWER16(addr), 8));			/* lw $8, LOWER16(addr)($8) */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));	/* sw $8,PRACC_OUT_OFFSET($15) */
	pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf)
{
	if (count == 1 && size == 4)
		return mips32_pracc_read_u32(ejtag_info, addr, (uint32_t *)buf);

	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	uint32_t *data = NULL;
	if (size != 4) {
		data = malloc(256 * sizeof(uint32_t));
		if (!data) {
			LOG_ERROR("Out of memory");
			goto exit;
		}
	}

	uint32_t *buf32 = buf;
	uint16_t *buf16 = buf;
	uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;

		int this_round_count = (count > 256) ? 256 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR)); /* $15 = MIPS32_PRACC_BASE_ADDR */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 9, last_upper_base_addr));	/* upper memory addr to $9 */

		for (int i = 0; i != this_round_count; i++) {			/* Main code loop */
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {	/* if needed, change upper addr in $9 */
				pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 9, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4)				/* load from memory to $8 */
				pracc_add(&ctx, 0, MIPS32_LW(ctx.isa, 8, LOWER16(addr), 9));
			else if (size == 2)
				pracc_add(&ctx, 0, MIPS32_LHU(ctx.isa, 8, LOWER16(addr), 9));
			else
				pracc_add(&ctx, 0, MIPS32_LBU(ctx.isa, 8, LOWER16(addr), 9));

			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + i * 4,			/* store $8 at param out */
					  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + i * 4, 15));
			addr += size;
		}
		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
		pracc_add_li32(&ctx, 9, ejtag_info->reg9, 0);				/* restore $9 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		if (size == 4) {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf32, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;
			buf32 += this_round_count;
		} else {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;

			uint32_t *data_p = data;
			for (int i = 0; i != this_round_count; i++) {
				if (size == 2)
					*buf16++ = *data_p++;
				else
					*buf8++ = *data_p++;
			}
		}
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	free(data);
	return ctx.retval;
}

int mips32_cp0_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 8, cp0_reg, cp0_sel));		/* move cp0 reg / sel to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));	/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* restore $15 from DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(ejtag_info->reg8)));	/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(ejtag_info->reg8))); /* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_cp0_write(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add_li32(&ctx, 15, val, 0);				/* Load val to $15 */

	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, cp0_reg, cp0_sel));		/* write $15 to cp0 reg / sel */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/**
 * \b mips32_pracc_sync_cache
 *
 * Synchronize Caches to Make Instruction Writes Effective
 * (ref. doc. MIPS32 Architecture For Programmers Volume II: The MIPS32 Instruction Set,
 *  Document Number: MD00086, Revision 2.00, June 9, 2003)
 *
 * When the instruction stream is written, the SYNCI instruction should be used
 * in conjunction with other instructions to make the newly-written instructions effective.
 *
 * Explanation :
 * A program that loads another program into memory is actually writing the D- side cache.
 * The instructions it has loaded can't be executed until they reach the I-cache.
 *
 * After the instructions have been written, the loader should arrange
 * to write back any containing D-cache line and invalidate any locations
 * already in the I-cache.
 *
 * If the cache coherency attribute (CCA) is set to zero, it's a write through cache, there is no need
 * to write back.
 *
 * In the latest MIPS32/64 CPUs, MIPS provides the synci instruction,
 * which does the whole job for a cache-line-sized chunk of the memory you just loaded:
 * That is, it arranges a D-cache write-back (if CCA = 3) and an I-cache invalidate.
 *
 * The line size is obtained with the rdhwr SYNCI_Step in release 2 or from cp0 config 1 register in release 1.
 */
static int mips32_pracc_synchronize_cache(struct mips_ejtag *ejtag_info,
					 uint32_t start_addr, uint32_t end_addr, int cached, int rel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	/** Find cache line size in bytes */
	uint32_t clsiz;
	if (rel) {	/* Release 2 (rel = 1) */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR)); /* $15 = MIPS32_PRACC_BASE_ADDR */

		pracc_add(&ctx, 0, MIPS32_RDHWR(ctx.isa, 8, MIPS32_SYNCI_STEP)); /* load synci_step value to $8 */

		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));		/* store $8 to pracc_out */

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, &clsiz, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

	} else {			/* Release 1 (rel = 0) */
		uint32_t conf;
		ctx.retval = mips32_cp0_read(ejtag_info, &conf, 16, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

		uint32_t dl = (conf & MIPS32_CONFIG1_DL_MASK) >> MIPS32_CONFIG1_DL_SHIFT;

		/* dl encoding : dl=1 => 4 bytes, dl=2 => 8 bytes, etc... max dl=6 => 128 bytes cache line size */
		clsiz = 0x2 << dl;
		if (dl == 0)
			clsiz = 0;
	}

	if (clsiz == 0)
		goto exit;  /* Nothing to do */

	/* make sure clsiz is power of 2 */
	if (!IS_PWR_OF_2(clsiz)) {
		LOG_DEBUG("clsiz must be power of 2");
		ctx.retval = ERROR_FAIL;
		goto exit;
	}

	/* make sure start_addr and end_addr have the same offset inside de cache line */
	start_addr |= clsiz - 1;
	end_addr |= clsiz - 1;

	ctx.code_count = 0;
	ctx.store_count = 0;

	int count = 0;
	uint32_t last_upper_base_addr = UPPER16((start_addr + 0x8000));

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, last_upper_base_addr)); /* load upper memory base addr to $15 */

	while (start_addr <= end_addr) {						/* main loop */
		uint32_t upper_base_addr = UPPER16((start_addr + 0x8000));
		if (last_upper_base_addr != upper_base_addr) {		/* if needed, change upper addr in $15 */
			pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, upper_base_addr));
			last_upper_base_addr = upper_base_addr;
		}
		if (rel)			/* synci instruction, offset($15) */
			pracc_add(&ctx, 0, MIPS32_SYNCI(ctx.isa, LOWER16(start_addr), 15));

		else {
			if (cached == 3)	/* cache Hit_Writeback_D, offset($15) */
				pracc_add(&ctx, 0, MIPS32_CACHE(ctx.isa, MIPS32_CACHE_D_HIT_WRITEBACK,
							LOWER16(start_addr), 15));
			/* cache Hit_Invalidate_I, offset($15) */
			pracc_add(&ctx, 0, MIPS32_CACHE(ctx.isa, MIPS32_CACHE_I_HIT_INVALIDATE,
							LOWER16(start_addr), 15));
		}
		start_addr += clsiz;
		count++;
		if (count == 256 && start_addr <= end_addr) {			/* more ?, then execute code list */
			pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* to start */
			pracc_add(&ctx, 0, MIPS32_NOP);					/* nop in delay slot */

			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;

			ctx.code_count = 0;	/* reset counters for another loop */
			ctx.store_count = 0;
			count = 0;
		}
	}
	pracc_add(&ctx, 0, MIPS32_SYNC(ctx.isa));
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* restore $15 from DeSave*/

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int mips32_pracc_write_mem_generic(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, const void *buf)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	const uint32_t *buf32 = buf;
	const uint16_t *buf16 = buf;
	const uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;

		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));
			      /* load $15 with memory base address */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, last_upper_base_addr));

		for (int i = 0; i != this_round_count; i++) {
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {	/* if needed, change upper address in $15*/
				pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4) {
				pracc_add_li32(&ctx, 8, *buf32, 1);		/* load with li32, optimize */
				pracc_add(&ctx, 0, MIPS32_SW(ctx.isa, 8, LOWER16(addr), 15)); /* store word to mem */
				buf32++;

			} else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 0, *buf16));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SH(ctx.isa, 8, LOWER16(addr), 15)); /* store half word */
				buf16++;

			} else {
				pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 0, *buf8));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SB(ctx.isa, 8, LOWER16(addr), 15));	/* store byte */
				buf8++;
			}
			addr += size;
		}

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	int retval = mips32_pracc_write_mem_generic(ejtag_info, addr, size, count, buf);
	if (retval != ERROR_OK)
		return retval;

	/**
	 * If we are in the cacheable region and cache is activated,
	 * we must clean D$ (if Cache Coherency Attribute is set to 3) + invalidate I$ after we did the write,
	 * so that changes do not continue to live only in D$ (if CCA = 3), but to be
	 * replicated in I$ also (maybe we wrote the instructions)
	 */
	uint32_t conf = 0;
	int cached = 0;

	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff)))
		return retval; /*Nothing to do*/

	mips32_cp0_read(ejtag_info, &conf, 16, 0);

	switch (KSEGX(addr)) {
		case KUSEG:
			cached = (conf & MIPS32_CONFIG0_KU_MASK) >> MIPS32_CONFIG0_KU_SHIFT;
			break;
		case KSEG0:
			cached = (conf & MIPS32_CONFIG0_K0_MASK) >> MIPS32_CONFIG0_K0_SHIFT;
			break;
		case KSEG2:
		case KSEG3:
			cached = (conf & MIPS32_CONFIG0_K23_MASK) >> MIPS32_CONFIG0_K23_SHIFT;
			break;
		default:
			/* what ? */
			break;
	}

	/**
	 * Check cacheability bits coherency algorithm
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 */
	if (cached == 3 || cached == 0) {		/* Write back cache or write through cache */
		uint32_t start_addr = addr;
		uint32_t end_addr = addr + count * size;
		uint32_t rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
		if (rel > 1) {
			LOG_DEBUG("Unknown release in cache code");
			return ERROR_FAIL;
		}
		retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
	}

	return retval;
}

int mips32_pracc_write_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	uint32_t cp0_write_code[] = {
		MIPS32_MTC0(ctx.isa, 1, 12, 0),					/* move $1 to status */
		MIPS32_MTLO(ctx.isa, 1),						/* move $1 to lo */
		MIPS32_MTHI(ctx.isa, 1),						/* move $1 to hi */
		MIPS32_MTC0(ctx.isa, 1, 8, 0),					/* move $1 to badvaddr */
		MIPS32_MTC0(ctx.isa, 1, 13, 0),					/* move $1 to cause*/
		MIPS32_MTC0(ctx.isa, 1, 24, 0),					/* move $1 to depc (pc) */
	};

	/* load registers 2 to 31 with li32, optimize */
	for (int i = 2; i < 32; i++)
		pracc_add_li32(&ctx, i, regs[i], 1);

	for (int i = 0; i != 6; i++) {
		pracc_add_li32(&ctx, 1, regs[i + 32], 0);	/* load CPO value in $1 */
		pracc_add(&ctx, 0, cp0_write_code[i]);			/* write value from $1 to CPO register */
	}
	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));				/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 1, UPPER16((regs[1]))));		/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 1, 1, LOWER16((regs[1]))));	/* load lower half word in $1 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);

	ejtag_info->reg8 = regs[8];
	ejtag_info->reg9 = regs[9];
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	uint32_t cp0_read_code[] = {
		MIPS32_MFC0(ctx.isa, 8, 12, 0),					/* move status to $8 */
		MIPS32_MFLO(ctx.isa, 8),						/* move lo to $8 */
		MIPS32_MFHI(ctx.isa, 8),						/* move hi to $8 */
		MIPS32_MFC0(ctx.isa, 8, 8, 0),					/* move badvaddr to $8 */
		MIPS32_MFC0(ctx.isa, 8, 13, 0),					/* move cause to $8 */
		MIPS32_MFC0(ctx.isa, 8, 24, 0),					/* move depc (pc) to $8 */
	};

	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 1, 31, 0));				/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 1, PRACC_UPPER_BASE_ADDR));	/* $1 = MIP32_PRACC_BASE_ADDR */

	for (int i = 2; i != 32; i++)					/* store GPR's 2 to 31 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i * 4),
				  MIPS32_SW(ctx.isa, i, PRACC_OUT_OFFSET + (i * 4), 1));

	for (int i = 0; i != 6; i++) {
		pracc_add(&ctx, 0, cp0_read_code[i]);				/* load COP0 needed registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i + 32) * 4,			/* store $8 at PARAM OUT */
				  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + (i + 32) * 4, 1));
	}
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 8, 31, 0));			/* move DeSave to $8, reg1 value */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + 4,			/* store reg1 value from $8 to param out */
			  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + 4, 1));

	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 1, 31, 0));		/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));				/* load $15 in DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs, 1);

	ejtag_info->reg8 = regs[8];	/* reg8 is saved but not restored, next called function should restore it */
	ejtag_info->reg9 = regs[9];
	pracc_queue_free(&ctx);
	return ctx.retval;
}


/* fastdata upload/download requires an initialized working area
 * to load the download code; it should not be called otherwise
 * fetch order from the fastdata area
 * 1. start addr
 * 2. end addr
 * 3. data ...
 */


/*
 * Stock handler entry: feed the jump code one instruction per processor access,
 * then confirm the handler is asking for its first word from FASTDATA_AREA.
 * Drives the standalone CONTROL and ADDRESS registers throughout.
 */
static int mips32_fastdata_enter_handler(struct mips_ejtag *ejtag_info,
		const uint32_t *jmp_code, size_t jmp_count)
{
	for (size_t i = 0; i < jmp_count; i++) {
		int retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		mips32_pracc_finish(ejtag_info);
	}

	/* wait PrAcc pending bit for FASTDATA write, read address */
	int retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	if (ejtag_info->pa_addr != MIPS32_PRACC_FASTDATA_AREA) {
		LOG_ERROR("fastdata: handler did not reach FASTDATA_AREA - "
				"pa_addr=0x%8.8" PRIx32 " expected 0x%8.8" PRIx32
				" pa_ctrl=0x%8.8" PRIx32,
				ejtag_info->pa_addr, MIPS32_PRACC_FASTDATA_AREA,
				ejtag_info->pa_ctrl);
		return ERROR_FAIL;
	}

	LOG_DEBUG("fastdata: reached FASTDATA_AREA, pa_ctrl=0x%8.8" PRIx32,
			ejtag_info->pa_ctrl);
	return ERROR_OK;
}

/*
 * Wait for the handler's next FASTDATA read without servicing it.
 *
 * The stock path uses wait_for_pracc_rw(), which polls EJTAG CONTROL as an
 * individually selected register - the exact access this part corrupts.  The
 * ALL register answers the same question: PrAcc written as 1 leaves a pending
 * access alone, so this observes without consuming.
 *
 * Also flushes the queued FASTDATA scan first.  An IR select batched into the
 * same adapter transaction as a DR scan is what the quirk exists to avoid, so
 * the data scans and the register selects must not share a queue.
 *
 * Leaves the IR forced back onto FASTDATA, ready for the next data scan.
 */
static int mt7628_fastdata_wait(struct mips_ejtag *ejtag_info)
{
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	uint32_t observe_ctrl =
			(ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_ROCC) | EJTAG_CTRL_PRACC;
	int64_t then = timeval_ms();
	unsigned int poll = 0;

	while (1) {
		uint32_t ctrl = 0, daddr = 0;

		/*
		 * Re-force ALL before every read.  This TAP does not stay on
		 * the selected instruction, and the drift is not always the
		 * clean IDCODE that mt7628_pracc_prime_at_text() looks for -
		 * a FASTDATA scan leaves it answering 0xffff824f/0xffffffff,
		 * the corrupted-upper-half form of IDCODE.
		 */
		retval = mt7628_pracc_force_all_ir(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		retval = mt7628_pracc_all_scan(ejtag_info, observe_ctrl, 0,
				&ctrl, NULL, &daddr);
		if (retval != ERROR_OK)
			return retval;

		if ((ctrl & EJTAG_CTRL_PRACC) &&
				daddr == MIPS32_PRACC_FASTDATA_AREA)
			break;

		LOG_DEBUG("mt7628 fastdata wait[%u]: ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32, poll++, ctrl, daddr);

		if (timeval_ms() - then > 1000) {
			LOG_ERROR("mt7628 fastdata: no access pending at FASTDATA_AREA "
					"(ctrl=0x%8.8" PRIx32 " addr=0x%8.8" PRIx32 ")",
					ctrl, daddr);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return mt7628_pracc_force_ir(ejtag_info, EJTAG_INST_FASTDATA);
}

int mips32_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info, struct working_area *source,
		int write_t, uint32_t addr, int count, uint32_t *buf)
{
	uint32_t isa = ejtag_info->isa ? 1 : 0;

	uint32_t handler_code[] = {
		/* r15 points to the start of this code */
		MIPS32_SW(isa, 8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_SW(isa, 9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_SW(isa, 10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_SW(isa, 11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),
		/* start of fastdata area in t0 */
		MIPS32_LUI(isa, 8, UPPER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_ORI(isa, 8, 8, LOWER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_LW(isa, 9, 0, 8),						/* start addr in t1 */
		MIPS32_LW(isa, 10, 0, 8),						/* end addr to t2 */
					/* loop: */
		write_t ? MIPS32_LW(isa, 11, 0, 8) : MIPS32_LW(isa, 11, 0, 9),	/* from xfer area : from memory */
		write_t ? MIPS32_SW(isa, 11, 0, 9) : MIPS32_SW(isa, 11, 0, 8),	/* to memory      : to xfer area */

		MIPS32_BNE(isa, 10, 9, NEG16(3 << isa)),			/* bne $t2,t1,loop */
		MIPS32_ADDI(isa, 9, 9, 4),					/* addi t1,t1,4 */

		MIPS32_LW(isa, 8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_LW(isa, 9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_LW(isa, 10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_LW(isa, 11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),

		MIPS32_LUI(isa, 15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(isa, 15, 15, LOWER16(MIPS32_PRACC_TEXT) | isa),	/* isa bit for JR instr */
		MIPS32_JR(isa, 15),								/* jr start */
		MIPS32_MFC0(isa, 15, 31, 0),					/* move COP0 DeSave to $15 */
	};

	if (source->size < MIPS32_FASTDATA_HANDLER_SIZE)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	pracc_swap16_array(ejtag_info, handler_code, ARRAY_SIZE(handler_code));
		/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		int retval = mips32_pracc_write_mem(ejtag_info, source->address, 4,
				ARRAY_SIZE(handler_code), handler_code);
		if (retval != ERROR_OK) {
			LOG_ERROR("fastdata handler load failed");
			return retval;
		}
		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using 0x%.8" TARGET_PRIxADDR " for write handler", __func__, source->address);

	uint32_t jmp_code[] = {
		MIPS32_LUI(isa, 15, UPPER16(source->address)),			/* load addr of jump in $15 */
		MIPS32_ORI(isa, 15, 15, LOWER16(source->address) | isa),	/* isa bit for JR instr */
		MIPS32_JR(isa, 15),						/* jump to ram program */
		isa ? MIPS32_XORI(isa, 15, 15, 1) : MIPS32_NOP,	/* drop isa bit, needed for LW/SW instructions */
	};

	pracc_swap16_array(ejtag_info, jmp_code, ARRAY_SIZE(jmp_code));

	bool quirk = mips_ejtag_control_all_quirk(ejtag_info);
	int retval;

	if (quirk)
		retval = mt7628_fastdata_enter_handler(ejtag_info, source->address);
	else
		retval = mips32_fastdata_enter_handler(ejtag_info, jmp_code,
				ARRAY_SIZE(jmp_code));
	if (retval != ERROR_OK)
		return retval;

	if (quirk) {
		/* Forced through BYPASS and flushed on its own, like every
		 * other register select on this part. */
		retval = mt7628_pracc_force_ir(ejtag_info, EJTAG_INST_FASTDATA);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Send the load start address */
	uint32_t val = addr;
	if (!quirk)
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	if (quirk)
		retval = mt7628_fastdata_wait(ejtag_info);
	else
		retval = wait_for_pracc_rw(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* Send the load end address */
	val = addr + (count - 1) * 4;
	if (!quirk)
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	if (quirk) {
		retval = mt7628_fastdata_wait(ejtag_info);
		if (retval != ERROR_OK)
			return retval;
	}

	unsigned num_clocks = 0;	/* like in legacy code */
	if (ejtag_info->mode != 0)
		num_clocks = ((uint64_t)(ejtag_info->scan_delay) * adapter_get_speed_khz() + 500000) / 1000000;

	for (int i = 0; i < count; i++) {
		jtag_add_clocks(num_clocks);
		mips_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("fastdata load failed");
		return retval;
	}

	if (quirk) {
		/*
		 * count + 2 scans is exactly what the handler asks for: two
		 * address reads and one per word. If it consumed them all it is
		 * back at PRACC_TEXT with the entry fetch pending, which is the
		 * state the next PRACC prime expects.
		 *
		 * Observe without servicing (PrAcc written as 1). A FASTDATA
		 * read still pending here means the handler stalled mid
		 * transfer; servicing it would hand the core a zero to store
		 * into the destination, so fail instead of corrupting memory.
		 */
		uint32_t observe_ctrl =
				(ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_ROCC) | EJTAG_CTRL_PRACC;
		uint32_t ctrl = 0, daddr = 0;

		retval = mt7628_pracc_force_all_ir(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		retval = mt7628_pracc_all_scan(ejtag_info, observe_ctrl, 0,
				&ctrl, NULL, &daddr);
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("mt7628 fastdata: post-transfer ctrl=0x%8.8" PRIx32
				" addr=0x%8.8" PRIx32, ctrl, daddr);

		if (!(ctrl & EJTAG_CTRL_PRACC) || daddr != MIPS32_PRACC_TEXT) {
			LOG_ERROR("mt7628 fastdata: handler did not retire to PRACC_TEXT "
					"(ctrl=0x%8.8" PRIx32 " addr=0x%8.8" PRIx32
					"); transfer incomplete", ctrl, daddr);
			return ERROR_FAIL;
		}

		return ERROR_OK;
	}

	retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return retval;
}
