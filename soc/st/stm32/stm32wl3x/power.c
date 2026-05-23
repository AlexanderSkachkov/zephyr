/*
 * Copyright (c) 2025 STMicroelectronics.
 * Copyright (c) 2026 Movu robotic.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * STM32WB0 Deepstop implementation for Power Management framework
 *
 * TODO:
 *	- document the control flow on PM transitions
 *	- assertions around system configuration
 *	  (e.g., valid slow clock selected, RTC enabled, ...)
 *	- ...
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/sys_clock.h>
#include <zephyr/init.h>
#include <zephyr/arch/common/pm_s2ram.h>
#include <zephyr/drivers/timer/system_timer.h>

/* Private headers in zephyr/drivers/... */
#include <clock_control/clock_stm32_ll_common.h>

#include "wkup_pins.h"

#include <soc.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_cortex.h>
#include <stm32_ll_system.h>
#include <stm32_ll_usart.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

static uint32_t rcc_apb0enr_vr, rcc_apb1enr_vr, rcc_apb2enr_vr, rcc_ahbenr_vr;

/* CPU-side NVIC state saved around Deepstop. Per RM0511 §5.4.2, the
 * VDD12i power domain is reset on every Deepstop entry, which wipes
 * NVIC->ISER (interrupt enables) and NVIC->IP (priorities). Zephyr's
 * arch_pm_s2ram_suspend saves CPU GPRs but not these — so without the
 * save/restore here, every IRQ the kernel enabled at boot
 * (GPIOA/GPIOB for gpio_keys, MRSUBG_TIMER_CPU_WKUP for the radio
 * timer that drives kernel ticks, MRSUBG for the radio driver, etc.) silently
 * stops firing after the first wake.
 *
 * WL3 has 26 IRQs (0..25), all fitting in ISER[0]. IP[0..7] covers all
 * IRQ priorities (4 IRQs per 32-bit word x 8 = 32 priorities).
 *
 * See STM32CubeFW WL3 v1.4.0's
 * Projects/NUCLEO-WL33CC/Demonstrations/MRSUBG/MRSUBG_Timer/System/
 * Startup/device_context_switch.c (cpuPeriphContextSave/Restore).
 */
static uint32_t nvic_iser_vr;
static uint32_t nvic_ip_vr[8];

/* GPIO bank config saved around Deepstop. Per RM0511 §5.4.2 the VDD12i
 * power domain is reset on Deepstop entry, which clears the GPIO config
 * registers (MODER/OTYPER/OSPEEDR/PUPDR/ODR/AFR). GPIORET holds the pads at
 * their pre-Deepstop levels for the duration of the sleep, but once it is
 * released on wake the pads follow these (now-reset) registers again — so
 * any output (status LEDs, regulator/peripheral enables, ...) drops to its
 * reset level unless the bank config is rewritten after retention is
 * released (see post_resume_configuration for the load-bearing ordering).
 * The STM32 GPIO driver has no PM save/restore of its own, so it is done
 * here, mirroring the NVIC/RCC save/restore. PWRC can only wake on PORTA/
 * PORTB and WL3 exposes no other GPIO ports, so two banks cover the device.
 */
struct wl3_gpio_bank_ctx {
	uint32_t moder;
	uint32_t otyper;
	uint32_t ospeedr;
	uint32_t pupdr;
	uint32_t odr;
	uint32_t afr[2];
};

static struct wl3_gpio_bank_ctx gpioa_ctx_vr, gpiob_ctx_vr;

static void wl3_gpio_bank_save(struct wl3_gpio_bank_ctx *ctx, const GPIO_TypeDef *gpio)
{
	ctx->moder   = gpio->MODER;
	ctx->otyper  = gpio->OTYPER;
	ctx->ospeedr = gpio->OSPEEDR;
	ctx->pupdr   = gpio->PUPDR;
	ctx->odr     = gpio->ODR;
	ctx->afr[0]  = gpio->AFR[0];
	ctx->afr[1]  = gpio->AFR[1];
}

static void wl3_gpio_bank_restore(const struct wl3_gpio_bank_ctx *ctx, GPIO_TypeDef *gpio)
{
	/* Restore the output data and pin attributes before MODER, so that
	 * the instant a pin is switched back to output it already drives the
	 * intended level rather than glitching through a stale one.
	 */
	gpio->ODR     = ctx->odr;
	gpio->OTYPER  = ctx->otyper;
	gpio->OSPEEDR = ctx->ospeedr;
	gpio->PUPDR   = ctx->pupdr;
	gpio->AFR[0]  = ctx->afr[0];
	gpio->AFR[1]  = ctx->afr[1];
	gpio->MODER   = ctx->moder;
}

/* Callback for arch_pm_s2ram_suspend */
static int suspend_system_to_deepstop(void)
{
	/* Enable SLEEPDEEP to allow entry in Deepstop */
	LL_LPM_EnableDeepSleep();

	/* Complete all memory transactions */
	__DSB();

	/* Attempt entry in Deepstop */
	__WFI();

	/* Make sure no meaningful instruction is
	 * executed during the two cycles latency
	 * it takes to power-gate the CPU.
	 */
	__NOP();
	__NOP();

	/* This code is reached only if the device did not
	 * enter Deepstop mode (e.g., because an interrupt
	 * became pending during preparatory work).
	 * Disable SLEEPDEEP and return the appropriate error.
	 */
	LL_LPM_EnableSleep();

	return -EBUSY;
}

/* Backup system state to save and configure power
 * controller before entry in Deepstop mode
 */
static void prepare_for_deepstop_entry(void)
{

	/* Save the clock configuration. */
	rcc_apb0enr_vr = RCC->APB0ENR;
	rcc_apb1enr_vr = RCC->APB1ENR;
	rcc_apb2enr_vr = RCC->APB2ENR;

	/* Device PM (soc/st/stm32/common/gpioport_mgr.c SUSPEND ->
	 * clock_control_off) has already gated the GPIO bank clocks by the
	 * time we reach here. The bank config registers keep their values
	 * across the gating — only the AHB bus clock is gated — but they can
	 * neither be READ for the bank save below nor WRITTEN on the wake-side
	 * restore while the clock is off. Re-enable both bank clocks and fold
	 * that into the saved AHBENR, so the clock is restored on wake (in
	 * post_resume_configuration, before wl3_gpio_bank_restore runs). */
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	rcc_ahbenr_vr  = RCC->AHBENR;

	/* Save NVIC state — wiped by VDD12i reset on Deepstop entry. */
	nvic_iser_vr = NVIC->ISER[0];
	for (uint8_t i = 0; i < ARRAY_SIZE(nvic_ip_vr); i++) {
		nvic_ip_vr[i] = NVIC->IPR[i];
	}

	/* Save GPIO bank config — also wiped by the VDD12i reset; rewritten on
	 * wake after GPIO retention is released (see post_resume_configuration).
	 */
	wl3_gpio_bank_save(&gpioa_ctx_vr, GPIOA);
	wl3_gpio_bank_save(&gpiob_ctx_vr, GPIOB);

	/* The MRSUBG CPU wake-up timer drives the Zephyr system tick on this
	 * SoC (see zephyr/drivers/timer/stm32wl3_radio_timer.c), so its
	 * PWRC wake source — PWR_WAKEUP_SUBGHOST — MUST stay armed or the
	 * kernel can never advance time while in Deepstop and the system
	 * hangs after the first sleep.
	 *
	 * Radio-side sources (LPAWUR / SUBG / COMP) and LPUART RX wake are
	 * intentionally NOT enabled here — MRSUBG (radio frontend) was
	 * generating spurious IRQs that fired __WFI immediately on each
	 * Deepstop entry, preventing any GPIO wake from being seen. A build
	 * that needs incoming-radio / UART-RX wake should gate those sources
	 * on per-source Kconfigs (e.g. CONFIG_STM32WL3_PM_WAKE_SUBG,
	 * CONFIG_STM32WL3_PM_WAKE_LPUART).
	 */
	LL_PWR_EnableInternWU(PWR_WAKEUP_SUBGHOST);

	if (!IS_ENABLED(CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP)) {
		LL_PWR_EnableGPIORET();
		LL_PWR_EnableDBGRET();
	}

	/* Per RM0511 §5.8.1: "a Deepstop entry sequence cannot happen if a
	 * wakeup flag is already active when the system requests Deepstop."
	 * WUFA/WUFB get latched on every falling edge of an armed wake-up
	 * pin (PWRC_EWUA/EWUB), EVEN WHILE THE DEVICE IS AWAKE — so any
	 * button press during awake operation leaves WUFA bits set that then
	 * block the next Deepstop entry. Clearing must therefore be the LAST
	 * thing before __WFI so the window where a stray edge can re-latch
	 * the flag is minimised (we can't fully eliminate it, but a few
	 * microseconds is acceptable).
	 */
	LL_PWR_ClearWakeupSource(LL_PWR_WAKEUP_PORTA, LL_PWR_WAKEUP_ALL);
	LL_PWR_ClearWakeupSource(LL_PWR_WAKEUP_PORTB, LL_PWR_WAKEUP_ALL);
	LL_PWR_ClearInternalWakeupSource(LL_PWR_WAKEUP_ALL);

}

/* Restore SoC-level configuration lost in Deepstop
 * This function must be called right after wakeup.
 */
static void post_resume_configuration(void)
{

	__ASSERT_NO_MSG(LL_PWR_GetDeepstopSeqFlag() == 1);

	/* VTOR has been reset to its default value: restore it.
	 * (Note that RAM_VR.AppBase was filled during SoC init)
	 */
	SCB->VTOR = RAM_VR.AppBase;

	/* Restore the clock configuration first. The GPIO banks must be clocked
	 * for the bank restore below to take effect; the saved AHBENR has their
	 * enable bits forced on (see prepare_for_deepstop_entry) precisely so
	 * the clock is live again here.
	 */
	RCC->AHBENR  = rcc_ahbenr_vr;
	RCC->APB0ENR = rcc_apb0enr_vr;
	RCC->APB1ENR = rcc_apb1enr_vr;
	RCC->APB2ENR = rcc_apb2enr_vr;

	/* Release GPIO retention, then rewrite the bank config that the VDD12i
	 * reset wiped. GPIORET held the pad levels through Deepstop, but the
	 * config registers themselves come back reset, so once retention is
	 * released the pads follow those reset registers — every output that
	 * nothing re-drives drops to its reset level. On a heartbeat/timer wake
	 * the application re-drives its outputs immediately so it is invisible;
	 * on a button-only wake nothing re-drives them, which is what made the
	 * status LED go dark on a button press. Rewriting the saved bank config
	 * here re-establishes those outputs. wl3_gpio_bank_restore writes ODR
	 * before MODER so each pin is switched to output already holding its
	 * intended level; the few-instruction window between the release and
	 * the rewrite is harmless for the affected outputs.
	 *
	 * This all runs before the kernel's resume_devices phase (PM flow:
	 * suspend_devices -> pm_state_set [here] -> resume_devices ->
	 * exit_post_ops), so gpio_keys et al. reconfigure on top of the
	 * restored state normally.
	 */
	LL_PWR_DisableGPIORET();
	LL_PWR_DisableDBGRET();
	wl3_gpio_bank_restore(&gpioa_ctx_vr, GPIOA);
	wl3_gpio_bank_restore(&gpiob_ctx_vr, GPIOB);

	/* Restore NVIC state. Without this, every IRQ the kernel had
	 * enabled at boot is gone (VDD12i power-down clears NVIC->ISER),
	 * and gpio_keys / radio_timer / radio IRQs all silently stop
	 * firing after the first wake.
	 */
	NVIC->ISER[0] = nvic_iser_vr;
	for (uint8_t i = 0; i < ARRAY_SIZE(nvic_ip_vr); i++) {
		NVIC->IPR[i] = nvic_ip_vr[i];
	}

	/* Wait until the HSE is ready */
	while(LL_RCC_HSE_IsReady() == 0U);

	/* If the MRSUBG timer is active, wait until the timebase is fully restored (3 slow clock periods after HSE Ready) */
	if(LL_MRSUBG_TIMER_IsEnabledCPUWakeupTimer(MR_SUBG_GLOB_RETAINED) || LL_MRSUBG_TIMER_IsEnabledRFIPWakeupTimer(MR_SUBG_GLOB_RETAINED))
	{
		while(LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC) == 0);
	}

	/* Read AND clear the PWRC wake-up source flags. Capture both ports
	 * into locals so we can both (a) decide whether a wake-up pin caused
	 * this resume (workaround below) and (b) hand the bitmasks to the
	 * input-event synthesiser. GetClearWakeupSource consumes the bits,
	 * so this read must happen exactly once.
	 */
	uint32_t wuf_porta = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTA);
	uint32_t wuf_portb = HAL_PWR_GetClearWakeupSource(LL_PWR_WAKEUP_PORTB);

	if ((wuf_porta | wuf_portb) != 0u) {
		/* A wake-up pin fired. The PWRC wake-up path doesn't re-fire a
		 * GPIO IRQ — the gpio_keys driver won't see the press — so
		 * synthesise an input event for each fired wake pin. The event
		 * is delivered through the input subsystem, which wakes any
		 * thread waiting on it (e.g. an application k_msgq_get) on its
		 * own; there is deliberately no sys_clock_announce() here. The
		 * MRSUBG counter is free-running and retained across Deepstop,
		 * so sys_clock_elapsed() already reports the true elapsed time
		 * and the kernel's timeout accounting stays correct without any
		 * nudge — announcing a delta here would only corrupt the tick
		 * and spuriously expire unrelated timeouts.
		 */
		wl3_wkup_pins_dispatch_resume(wuf_porta, wuf_portb);
	}

	/* Re-arm happens in pm_state_exit_post_ops, AFTER the kernel has
	 * resumed devices and gpio_keys has restored the pin pull-ups.
	 * Arming earlier (here, or in post_resume_configuration) races with
	 * the GPIO retention/reconfigure and PWRC ends up watching a
	 * floating pin — the falling edge on the next press is never seen.
	 */
}

/* Power Management subsystem callbacks */
void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	/* Ignore substate: STM32WB0 has only one low-power mode */
	ARG_UNUSED(substate_id);

	if (state != PM_STATE_SUSPEND_TO_RAM) {

		/* Deepstop is a suspend-to-RAM state.
		 * Something is wrong if a different
		 * power state has been requested.
		 */
		LOG_ERR("Unsupported power state %u", state);
	}

	prepare_for_deepstop_entry();

	/* Select Deepstop low-power mode and suspend system */
	LL_PWR_SetPowerMode(LL_PWR_MODE_DEEPSTOP);

	if (arch_pm_s2ram_suspend(suspend_system_to_deepstop) >= 0) {
		post_resume_configuration();
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	/* GPIO retention is now released inside post_resume_configuration()
	 * (BEFORE the kernel resumes devices) so that gpio_keys et al. can
	 * actually reconfigure their pins. Also handle the case where the
	 * __WFI in suspend_system_to_deepstop aborted: GPIORET was enabled in
	 * prepare_for_deepstop_entry but post_resume_configuration was never
	 * reached. Disable defensively here too — SET_BIT / CLEAR_BIT on the
	 * same bit twice is a no-op.
	 */
	LL_PWR_DisableGPIORET();
	LL_PWR_DisableDBGRET();

	__enable_irq();

	/* Re-arm wake-up pins NOW — at this point the kernel has finished
	 * resuming devices, gpio_keys has restored its pin configuration
	 * (GPIO_INPUT with pull-up), and the lines are sitting at their
	 * steady-state high level. Arming PWRC against that stable state
	 * is what lets the falling edge from the next button press actually
	 * register. Calls are idempotent.
	 */
	wl3_wkup_pins_arm();
}
