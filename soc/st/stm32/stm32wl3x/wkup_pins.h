/*
 * Copyright (c) 2026 Movu robotic.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SOC_ST_STM32_STM32WL3X_WKUP_PINS_H_
#define ZEPHYR_SOC_ST_STM32_STM32WL3X_WKUP_PINS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Re-arm every gpio-keys child marked `wakeup-source;` against PWRC.
 *
 * Idempotent — safe to call from SYS_INIT and from each Deepstop resume.
 * Must run AFTER gpio_keys has configured its pins (pull-ups settled), or
 * PWRC latches a floating starting level and the first real falling edge
 * is never seen.
 */
void wl3_wkup_pins_arm(void);

/* Synthesise an input-press event for every wake-up pin whose latched
 * WUFA/WUFB bit is set in the supplied masks. Called from the PM resume
 * path because the GPIO INTC is power-gated during Deepstop, so
 * gpio_keys' native IRQ path doesn't see the edge that woke the SoC.
 */
void wl3_wkup_pins_dispatch_resume(uint32_t wuf_porta, uint32_t wuf_portb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ST_STM32_STM32WL3X_WKUP_PINS_H_ */
