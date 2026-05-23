/*
 * Copyright (c) 2025 STMicroelectronics
 * Copyright (c) 2026 Movu robotics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * STM32WL3-specific support code for suspend-to-RAM
 */

#include <zephyr/toolchain.h>
#include <offsets_short.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/linker/sections.h>

/* Read RCC and PWRC base from Device Tree */
#include <zephyr/devicetree.h>

#define STM32WL3_RCC_CSR	(DT_REG_ADDR(DT_NODELABEL(rcc)) + 0x94)
#define STM32WL3_PWR_BASE	DT_REG_ADDR(DT_NODELABEL(pwrc))
/*
 * Wake-up flag registers (offsets within the PWR block):
 *   IWUF (0x10) – internal wakes (incl. the MRSUBG host timer = kernel tick)
 *   WUFA (0x38) – PORTA wake pins
 *   WUFB (0x48) – PORTB wake pins
 *
 * The original code only checked IWUF + WUFA (it labelled them SR1/SR3) and
 * MISSED WUFB. A wake from a PORTB pin (e.g. SW3/PB15) sets WUFB and
 * EXTSRR.DEEPSTOPF but neither IWUF nor WUFA, so this routine reported
 * "not a Deepstop wake" and the SoC cold-booted instead of restoring the
 * suspended context — losing all RAM state, and never running
 * post_resume_configuration() (so the wake pin's input event was never
 * dispatched either). Check all three flag registers, matching ST
 * SystemInit()'s (IWUF | WUFA | WUFB) condition.
 */
#define STM32WL3_PWR_IWUF	0x10
#define STM32WL3_PWR_WUFA	0x38
#define STM32WL3_PWR_WUFB	0x48

_ASM_FILE_PROLOGUE

GTEXT(pm_s2ram_mark_set)
SECTION_FUNC(TEXT, pm_s2ram_mark_set)
	/*
	 * Managed by hardware - nothing to do here.
	 */
	bx lr

/**
 * @brief	Check whether SoC is waking up from Deepstop
 * @returns	1 if SoC is waking up from Deepstop, 0 otherwise.
 * @note	Registers are cleared by hardware upon reset, or
 * 		the SoC PM code layer upon entry in Deepstop, so
 *		this function does not clear the registers.
 */
GTEXT(pm_s2ram_mark_check_and_clear)
SECTION_FUNC(TEXT, pm_s2ram_mark_check_and_clear)
	/*
	 * Check for Deepstop exit on wakeup event:
	 *	- RCC_CSR is zero (no reset cause = came out of Deepstop)
	 *	- At least one wake flag is set: IWUF | WUFA | WUFB
	 *	  (WUFB covers PORTB pins, which the old SR1/SR3 check missed)
	 *
	 * Note that we don't have to clear any register since
	 * they are automatically updated on reset/wake-up, and the
	 * SoC PM resume path later reads+clears WUFA/WUFB itself.
	 */
	ldr	r0, =STM32WL3_RCC_CSR
	ldr	r0, [r0]
	cmp	r0, #0
	bne	not_deepstop_wakeup
	ldr	r3, =STM32WL3_PWR_BASE
	ldr	r1, [r3, #STM32WL3_PWR_IWUF]
	ldr	r0, [r3, #STM32WL3_PWR_WUFA]
	ldr	r2, [r3, #STM32WL3_PWR_WUFB]
	orrs	r0, r0, r1
	orrs	r0, r0, r2
	beq	not_deepstop_wakeup
	/* All conditions met: this is a wakeup from Deepstop. */
	movs	r0, #1
	bx	lr
not_deepstop_wakeup:
	movs	r0, #0
	bx	lr
