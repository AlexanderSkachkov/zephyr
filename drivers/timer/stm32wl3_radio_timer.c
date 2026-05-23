/*
 * Copyright (c) 2025 STMicroelectronics
 * Copyright (c) 2026 Movu robotics
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <cmsis_core.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/init.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys_clock.h>
#include <zephyr/sys/util.h>
#include "stm32wl3x_hal_mrsubg_timer.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(radio_timer_driver);

#define MAX_HS_STARTUP_TIME		DT_PROP(DT_NODELABEL(radio_timer), max_hs_startup_time)
#define CPU_WKUP_PRIO			1
#define TIMER_ROUNDING			8
#define LSI_ENABLED 			DT_NODE_HAS_STATUS(DT_NODELABEL(clk_lsi), okay)
#define LSE_ENABLED 			DT_NODE_HAS_STATUS(DT_NODELABEL(clk_lse), okay)

BUILD_ASSERT(!(LSI_ENABLED && LSE_ENABLED),
	"LSI and LSE cannot be used together!");

/* Last announced absolute time (32-bit, matches HW counter width so the
 * subtraction in the ISR / sys_clock_elapsed wraps correctly).
 */
static volatile uint32_t announced_machine_time;
static volatile uint32_t slow_clock_freq 	= LSE_VALUE;
static volatile uint32_t fast_clock_freq 	= HSE_VALUE / 3;

/* Cached WAKEUP_OFFSET (in slow-clock periods) — used as the floor for the
 * minimum schedule so we never program a wake-up time that lands inside the
 * HSE startup window. Set in init from DT.
 */
static uint16_t wakeup_offset_slow_ticks;

static void radio_timer_cpu_wkup_isr(void *args)
{
	uint32_t now, dmachine_time;

	ARG_UNUSED(args);

	/* Disable host timer */
	LL_MRSUBG_TIMER_DisableCPUWakeupTimer(MR_SUBG_GLOB_RETAINED);
	/* Clear the interrupt (write-1-to-clear) and synchronize so the
	 * NVIC sees the line de-asserted before the ISR returns.
	 */
	LL_MRSUBG_TIMER_ClearFlag_CPUWakeup(MR_SUBG_GLOB_MISC);
	__DSB();

	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		now = LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC);
		/* 32-bit unsigned wraparound is intended here. */
		dmachine_time = now - announced_machine_time;
		announced_machine_time = now;
		sys_clock_announce(dmachine_time / 16u);
	} else {
		sys_clock_announce(1);
	}
}

#if(LSI_ENABLED)
static inline void calibrate_slow_clock()
{
	uint16_t scm_counter_currval;
	/* wait for SCM to leave reset state value */
  	while (LL_MRSUBG_TIMER_GetSCM(MR_SUBG_GLOB_MISC) == 0);
	scm_counter_currval  = LL_MRSUBG_TIMER_GetSCM(MR_SUBG_GLOB_MISC);
	slow_clock_freq 	 = ((32ull * (uint64_t)fast_clock_freq) / scm_counter_currval);
}

static void interpolate_time(uint32_t *ticks)
{
	calibrate_slow_clock();
	*ticks *= (slow_clock_freq * 1000 / LSE_VALUE);
	*ticks /= 1000;
}
#endif

void sys_clock_set_timeout(int32_t ticks, bool idle)
{

	ARG_UNUSED(idle);

	if (ticks == K_TICKS_FOREVER) {
		/* Nothing scheduled — make sure no stale wake survives, otherwise
		 * the SoC could exit deepstop at an unrelated time and confuse
		 * the kernel's tick accounting.
		 */
		LL_MRSUBG_TIMER_DisableCPUWakeupTimer(MR_SUBG_GLOB_RETAINED);
		return;
	}

#if(LSI_ENABLED)
	/* Here we can find the calibrated value for the expected timeout ticks */
	/* Since LSI is not stabile and not realy 32kHz we need to find it */
	interpolate_time(&ticks);
#endif

	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		uint32_t current_time, target_machine_time;
		uint32_t min_ticks;

		/* The wake-up time must land outside the HSE-startup window the
		 * HW reserves before each wake, otherwise (with PM=y in deepstop)
		 * the wake can fire before HSE is ready and miss / hang. Floor
		 * is WAKEUP_OFFSET (slow ticks) plus a safety margin.
		 */
		min_ticks = (uint32_t)wakeup_offset_slow_ticks + 32u;
		ticks = MAX((int32_t)min_ticks, ticks);

		target_machine_time = 16u * ticks;

		current_time = LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC);
		LL_MRSUBG_TIMER_SetCPUWakeupTime(MR_SUBG_GLOB_RETAINED, current_time + target_machine_time + TIMER_ROUNDING);
		LL_MRSUBG_TIMER_EnableCPUWakeupTimer(MR_SUBG_GLOB_RETAINED);
	}
}

uint32_t sys_clock_elapsed(void)
{
	uint32_t now, diff;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	now = LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC);
	/* 32-bit unsigned wraparound is intended. */
	diff = (now - announced_machine_time) / 16u;
#if(LSI_ENABLED)
	interpolate_time(&diff);
#endif
	return diff;
}

uint32_t sys_clock_cycle_get_32(void)
{
	return sys_clock_cycle_get_64() & UINT32_MAX;
}

uint64_t sys_clock_cycle_get_64(void)
{
#if(LSI_ENABLED)
	uint32_t ticks;
	ticks = (LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC) / 16u);
	interpolate_time(&ticks);
	return (uint64_t)ticks;
#else
	return LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC) / 16u;
#endif
}

void sys_clock_disable(void)
{
	irq_disable(MRSUBG_TIMER_CPU_WKUP_IRQn);
	LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
}

void sys_clock_idle_exit(void)
{
	irq_enable(MRSUBG_TIMER_CPU_WKUP_IRQn);
}

static int radio_clock_driver_init(void)
{

	uint16_t offset_time;

	IRQ_CONNECT(MRSUBG_TIMER_CPU_WKUP_IRQn,
				CPU_WKUP_PRIO,
				radio_timer_cpu_wkup_isr,
				NULL,
				0);

	/* Peripheral clock enable */
	if (!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_MRSUBG)) {
		/* Radio reset */
		LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_MRSUBG);
		LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_MRSUBG);

		/* Enable Radio peripheral clock */
		LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
	}

	/* Wait to be sure that the Radio Timer is active */
 	while (LL_MRSUBG_TIMER_GetAbsoluteTime(MR_SUBG_GLOB_MISC) < 0x10);

	/* Make sure no stale wake from a previous boot survives in the
	 * RETAINED block (CPU_WAKEUPTIME persists across reset). The timer
	 * gets armed properly on the first sys_clock_set_timeout call.
	 */
	LL_MRSUBG_TIMER_DisableCPUWakeupTimer(MR_SUBG_GLOB_RETAINED);
	LL_MRSUBG_TIMER_ClearFlag_CPUWakeup(MR_SUBG_GLOB_MISC);
	__DSB();

#if(LSI_ENABLED)
	calibrate_slow_clock();
#endif

	offset_time = ((MAX_HS_STARTUP_TIME * slow_clock_freq) / 1000000ull);
	if (offset_time >= UINT8_MAX)
	{
		offset_time = UINT8_MAX;
	}
	if (offset_time < 16)
	{
		offset_time = 20;
	}
	wakeup_offset_slow_ticks = offset_time;

	LL_MRSUBG_TIMER_SetWakeupOffset(MR_SUBG_GLOB_RETAINED, (uint8_t)offset_time);

	/* Enable the IRQ in NVIC. Without this the wake-up interrupt only
	 * starts firing after the first sys_clock_idle_exit() call, which is
	 * fragile (depends on PM exit path running before the first arm).
	 */
	irq_enable(MRSUBG_TIMER_CPU_WKUP_IRQn);

	return 0;
}

SYS_INIT(radio_clock_driver_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);