/*
 * Copyright (c) 2026 Movu robotic.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * STM32WL3 PWRC wake-up-pin helper for gpio-keys.
 *
 * Builds a compile-time table of all gpio-keys children that have
 * `wakeup-source;` set and whose gpio controller is gpioa or gpiob (the
 * only ports PWRC can wake on), and provides two services to the SoC PM
 * path:
 *
 *   wl3_wkup_pins_arm()              – called from SYS_INIT and from
 *                                      pm_state_exit_post_ops to (re-)enable
 *                                      each pin as a Deepstop wake source.
 *
 *   wl3_wkup_pins_dispatch_resume()  – called from post_resume_configuration
 *                                      after Deepstop; turns the latched
 *                                      WUFA/WUFB bits into input subsystem
 *                                      key-press events so userspace sees
 *                                      the press that woke the SoC (the GPIO
 *                                      INTC is power-gated during Deepstop,
 *                                      so gpio_keys' own IRQ never fires for
 *                                      the wake edge).
 *
 * PORTA and PORTB wake pins both work across Deepstop cycles. NOTE for
 * future PORTB wake pins: a PORTB wake sets only WUFB + EXTSRR.DEEPSTOPF
 * (not WUFA/IWUF), so the SoC s2ram resume-detection in s2ram_marking.S
 * MUST include WUFB. It originally checked only IWUF/WUFA and therefore
 * cold-booted on a PORTB wake (wiping RAM state and skipping this
 * dispatch), which looked like "PB15 wakes but produces no input event".
 * Fixed in s2ram_marking.S; keep WUFB in that check if you touch it.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>

#include <stm32_ll_pwr.h>
#include <stm32wl3x_hal_pwr.h>

#include "wkup_pins.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

struct wl3_wkup_pin {
	const struct device *gpio_keys_dev;
	uint8_t port; /* PWR_WAKEUP_PORTA / PWR_WAKEUP_PORTB */
	uint8_t pin;  /* 0..15 */
	uint16_t code;
};

#define WKUP_GPIOA_NODE DT_NODELABEL(gpioa)
#define WKUP_GPIOB_NODE DT_NODELABEL(gpiob)

/* Resolve the gpio controller of a gpio-keys child to a PWRC port number.
 * Returns 0xFF for any controller PWRC can't wake on; entries with port
 * 0xFF are skipped at runtime.
 */
#define WKUP_GPIO_PORT(node_id)                                                                    \
	COND_CODE_1(DT_SAME_NODE(DT_GPIO_CTLR(node_id, gpios), WKUP_GPIOA_NODE),                    \
		    (PWR_WAKEUP_PORTA),                                                            \
		    (COND_CODE_1(DT_SAME_NODE(DT_GPIO_CTLR(node_id, gpios), WKUP_GPIOB_NODE),      \
				 (PWR_WAKEUP_PORTB), (0xFFu))))

#define WKUP_ENTRY(node_id)                                                                        \
	{                                                                                          \
		.gpio_keys_dev = DEVICE_DT_GET(DT_PARENT(node_id)),                                \
		.port = WKUP_GPIO_PORT(node_id),                                                   \
		.pin = DT_GPIO_PIN(node_id, gpios),                                                \
		.code = DT_PROP(node_id, zephyr_code),                                             \
	},

#define WKUP_ENTRY_IF_WAKEUP(node_id)                                                              \
	IF_ENABLED(DT_PROP(node_id, wakeup_source), (WKUP_ENTRY(node_id)))

#define WKUP_FROM_KEYS_NODE(node_id) DT_FOREACH_CHILD_STATUS_OKAY(node_id, WKUP_ENTRY_IF_WAKEUP)

static const struct wl3_wkup_pin wkup_pins[] = {
	DT_FOREACH_STATUS_OKAY(gpio_keys, WKUP_FROM_KEYS_NODE)
};

void wl3_wkup_pins_arm(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(wkup_pins); i++) {
		const struct wl3_wkup_pin *p = &wkup_pins[i];

		if (p->port == 0xFFu) {
			continue;
		}

		/* Buttons are GPIO_ACTIVE_LOW with GPIO_PULL_UP — press
		 * yields a falling edge. PWRC wake polarity is configured
		 * per pin, so this is set every arm.
		 */
		HAL_PWR_EnableWakeUpPin((uint32_t)p->port, (uint32_t)(1U << p->pin),
					PWR_WUP_FALLEDG);
	}
}

void wl3_wkup_pins_dispatch_resume(uint32_t wuf_porta, uint32_t wuf_portb)
{
	for (size_t i = 0; i < ARRAY_SIZE(wkup_pins); i++) {
		const struct wl3_wkup_pin *p = &wkup_pins[i];
		uint32_t mask = (1U << p->pin);
		uint32_t fired;

		if (p->port == PWR_WAKEUP_PORTA) {
			fired = wuf_porta & mask;
		} else if (p->port == PWR_WAKEUP_PORTB) {
			fired = wuf_portb & mask;
		} else {
			continue;
		}

		if (fired == 0u) {
			continue;
		}

		/* Synthesise the press only. The matching release will
		 * normally be picked up by gpio_keys' GPIO IRQ once devices
		 * have resumed (the user is still holding the button at this
		 * point in the timeline by tens of microseconds).
		 */
		(void)input_report_key(p->gpio_keys_dev, p->code, 1, true, K_FOREVER);
	}
}

/* Initial arm. APPLICATION priority is load-bearing: PRE_KERNEL_2 runs
 * before GPIO drivers configure pin pull-ups, and arming PWRC against a
 * floating starting level latches the wrong state — the first real
 * falling edge after that is never seen.
 */
static int wl3_wkup_pins_init(void)
{
	wl3_wkup_pins_arm();
	return 0;
}

SYS_INIT(wl3_wkup_pins_init, APPLICATION, 90);
