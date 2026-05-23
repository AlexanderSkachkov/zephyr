/*
 * Copyright (c) 2024 STMicroelectronics
 * Copyright (c) 2026 Movu robotics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_radio.h>
#include <stm32_ll_system.h>
#include <stm32_ll_utils.h>

#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/arch/common/sys_bitops.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

/* Driver definitions */
#define RCC_REG(_reg_offset) (DT_REG_ADDR(STM32_CLOCK_CONTROL_NODE) + (_reg_offset))
#define RADIO_CTRL_IRQn		21	/* Not provided by CMSIS; must be declared manually */

#define CLOCK_FREQ_64MHZ	(64000000U)
#define CLOCK_FREQ_32MHZ	(32000000U)
#define CLOCK_FREQ_16MHZ	(16000000U)

/* Device tree node definitions */
#define DT_RCC_SLOWCLK_NODE	DT_PHANDLE(STM32_CLOCK_CONTROL_NODE, slow_clock)
#define DT_LSI_NODE		DT_NODELABEL(clk_lsi)

/* Device tree properties definitions */
#define STM32_WL3_CLKSYS_PRESCALER		\
	DT_PROP(STM32_CLOCK_CONTROL_NODE, clksys_prescaler)

#if DT_NODE_HAS_PROP(STM32_CLOCK_CONTROL_NODE, slow_clock)

#	if !DT_NODE_HAS_STATUS_OKAY(DT_RCC_SLOWCLK_NODE)
#		error slow-clock source is not enabled
#	endif

#	if DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_LSI_NODE)
#		define STM32_WL3_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_LSI
#	elif DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_NODELABEL(clk_lse))
#		define STM32_WL3_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_LSE
#	elif DT_SAME_NODE(DT_RCC_SLOWCLK_NODE, DT_NODELABEL(clk_16mhz_div512))
#		define STM32_WL3_SLOWCLK_SRC	LL_RCC_LSCO_CLKSOURCE_HSI64M_DIV2048
#	else
#		error Invalid device selected as slow-clock
#	endif

#endif /* DT_NODE_HAS_PROP(STM32_CLOCK_CONTROL_NODE, slow_clock) */

#if DT_NODE_HAS_PROP(DT_LSI_NODE, runtime_measurement_interval)
	#define STM32_WL3_RUNTIME_LSI_CALIBRATION 1
	#define STM32_WL3_LSI_RUNTIME_CALIB_INTERVAL	\
		DT_PROP(DT_LSI_NODE, runtime_measurement_interval)
#endif /* DT_NODE_HAS_PROP(clk_lsi, runtime_calibration_settings) */

/* Verify device tree properties are correct */
BUILD_ASSERT(!IS_ENABLED(STM32_SYSCLK_SRC_HSE) || STM32_WL3_CLKSYS_PRESCALER != 64,
	"clksys-prescaler cannot be 64 when SYSCLK source is Direct HSE");

#if defined(STM32_LSI_ENABLED)
/* Check clock configuration allows MR_BLE IP to work.
 * This IP is required to perform LSI measurements.
 */
#	if defined(STM32_SYSCLK_SRC_HSI)
		/* When using HSI without PLL, the "16MHz" output is not actually 16MHz, since
		 * the RC64M generator is imprecise. In this configuration, MR_BLE is broken.
		 * The CPU and MR_BLE must be running at 32MHz for MR_BLE to work with HSI.
		 */
		BUILD_ASSERT(STM32_HCLK_FREQUENCY >= CLOCK_FREQ_32MHZ,
			"System clock frequency must be at least 32MHz to use LSI");
#	else
		/* In PLL or Direct HSE mode, the clock is stable, so 16MHz can be used. */
		BUILD_ASSERT(STM32_HCLK_FREQUENCY >= CLOCK_FREQ_16MHZ,
			"System clock frequency must be at least 16MHz to use LSI");
#	endif /* STM32_SYSCLK_SRC_HSI */

/**
 * @brief Variable holding the "current frequency of LSI", according
 * to the measurement process. This variable is updated each time
 * a new measurement of the LSI frequency is performed.
 */
static volatile uint32_t stm32wl3_lsi_frequency = STM32_LSI_FREQ;
static volatile uint32_t last_scm_counter = 0;

/**
 * @brief Perform a measurement of the LSI frequency and updates
 * the @p stm32wl3_lsi_frequency global variable based on the results.
 *
 * @param wait_event	Semaphore to wait for completion of the measurement
 *			If NULL, RADIO_CONTROL registers are polled instead.
 */
static void measure_lsi_frequency()
{
	uint32_t scm_counter_currval;

	/* Read calibration results */
	scm_counter_currval = LL_MRSUBG_TIMER_GetSCM(MR_SUBG_GLOB_MISC);

	if ((scm_counter_currval != 0) && (scm_counter_currval != last_scm_counter))
  	{
		stm32wl3_lsi_frequency = ((32ull * CLOCK_FREQ_16MHZ) / scm_counter_currval);
	}
}
#endif /* STM32_LSI_ENABLED */

/** @brief Verifies if provided domain / bus clock is currently active */
int enabled_clock(uint32_t src_clk)
{
	int r = 0;

	switch (src_clk) {
	case STM32_SRC_SYSCLK:
		break;
	case STM32_SRC_LSE:
		if (!IS_ENABLED(STM32_LSE_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
	case STM32_SRC_LSI:
		if (!IS_ENABLED(STM32_LSI_ENABLED)) {
			r = -ENOTSUP;
		}
		break;
	case STM32_SRC_CLKSLOWMUX:
		break;
	case STM32_SRC_CLK16MHZ:
		break;
	case STM32_SRC_CLK32MHZ:
		break;
	default:
		return -ENOTSUP;
	}

	return r;
}

static int stm32_clock_control_on(const struct device *dev, clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	const mem_addr_t reg = RCC_REG(pclken->bus);
	volatile uint32_t temp;

	ARG_UNUSED(dev);
	if (!IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Attempting to change domain clock */
		return -ENOTSUP;
	}

	sys_set_bits(reg, pclken->enr);

	/* Read back register to be blocked by RCC
	 * until peripheral clock enabling is complete
	 */
	temp = sys_read32(reg);
	UNUSED(temp);

	return 0;
}

static int stm32_clock_control_off(const struct device *dev, clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	const mem_addr_t reg = RCC_REG(pclken->bus);

	ARG_UNUSED(dev);
	if (!IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Attempting to change domain clock */
		return -ENOTSUP;
	}

	sys_clear_bits(reg, pclken->enr);

	return 0;
}

static int stm32_clock_control_configure(const struct device *dev,
					 clock_control_subsys_t sub_system,
					 void *data)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)sub_system;
	const uint32_t shift = STM32_DT_CLKSEL_SHIFT_GET(pclken->enr);
	mem_addr_t reg = RCC_REG(STM32_DT_CLKSEL_REG_GET(pclken->enr));
	int err;

	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	err = enabled_clock(pclken->bus);
	if (err < 0) {
		/* Attempting to configure an unavailable or invalid clock */
		return err;
	}

	sys_clear_bits(reg, STM32_DT_CLKSEL_MASK_GET(pclken->enr) << shift);
	sys_set_bits(reg, STM32_DT_CLKSEL_VAL_GET(pclken->enr) << shift);

	return 0;
}

static int get_apb0_periph_clkrate(struct stm32_pclken *pclken,
	uint32_t *rate, uint32_t slow_clock, uint32_t sysclk, uint32_t clk_sys)
{
	switch (pclken->enr) {
	/* Slow clock peripherals: RTC & IWDG */
	case LL_APB0_GRP1_PERIPH_RTC:
	case LL_APB0_GRP1_PERIPH_WDG:
	case LL_APB0_GRP1_PERIPH_LCSC:
		*rate = slow_clock;
		break;

	/* SYSCLK peripherals: all timers */
#if defined(TIM2)
	case LL_APB0_GRP1_PERIPH_TIM2:
#endif
#if defined(TIM16)
	case LL_APB0_GRP1_PERIPH_TIM16:
#endif
		*rate = sysclk;
		break;

	/* CLK_SYS peripherals: SYSCFG */
	case LL_APB0_GRP1_PERIPH_SYSCFG:
		*rate = clk_sys;
		break;
	default:
		return -ENOTSUP;
	}

	if (pclken->div) {
		*rate /= (pclken->div + 1);
	}

	return 0;
}

static int get_apb1_periph_clkrate(struct stm32_pclken *pclken, uint32_t *rate, uint32_t clk_sys)
{
	switch (pclken->enr) {
#if defined(SPI1)
	case LL_APB1_GRP1_PERIPH_SPI1:
		*rate = clk_sys;
		break;
#endif
#if defined(SPI3)
	case LL_APB1_GRP1_PERIPH_SPI3:
		switch (LL_RCC_GetSPI3I2SClockSource()) {
		case LL_RCC_SPI3_I2S_CLKSOURCE_CLK_ROOT_DIV:
			*rate = CLOCK_FREQ_16MHZ;
			break;
		case LL_RCC_SPI3_I2S_CLKSOURCE_CLK_ROOT:
			*rate = CLOCK_FREQ_32MHZ;
			break;
#if defined(LL_RCC_SPI3_I2S_CLKSOURCE_CLK_RC64MPLL)
		case LL_RCC_SPI3_I2S_CLKSOURCE_CLK_RC64MPLL:
			*rate = CLOCK_FREQ_64MHZ;
			break;
#endif
		default:
			CODE_UNREACHABLE;
		}
		break;
#endif
	case LL_APB1_GRP1_PERIPH_I2C1:
#if defined(I2C2)
	case LL_APB1_GRP1_PERIPH_I2C2:
#endif
		*rate = CLOCK_FREQ_16MHZ;
		break;

	case LL_APB1_GRP1_PERIPH_ADCDIG:
	case LL_APB1_GRP1_PERIPH_ADCANA:
	case (LL_APB1_GRP1_PERIPH_ADCDIG | LL_APB1_GRP1_PERIPH_ADCANA):
	/* ADC has two enable bits - accept all combinations. */
		*rate = CLOCK_FREQ_16MHZ;
		break;

	case LL_APB1_GRP1_PERIPH_USART1:
		*rate = CLOCK_FREQ_16MHZ;
		break;

	case LL_APB1_GRP1_PERIPH_LPUART1:
#if !defined(RCC_CFGR_LPUCLKSEL)
		*rate = CLOCK_FREQ_16MHZ;
#else
		switch (LL_RCC_GetLPUARTClockSource()) {
		case LL_RCC_LPUCLKSEL_CLK16M:
			*rate = CLOCK_FREQ_16MHZ;
			break;
		case LL_RCC_LPUCLKSEL_CLKLSE:
			*rate = STM32_LSE_FREQ;
			break;
		default:
			CODE_UNREACHABLE;
		}
#endif
		break;
	default:
		return -ENOTSUP;
	}

	if (pclken->div) {
		*rate /= (pclken->div + 1);
	}

	return 0;
}

static int stm32_clock_control_get_subsys_rate(const struct device *dev,
						clock_control_subsys_t sub_system,
						uint32_t *rate)
{

	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);
	uint32_t sysclk, slow_clock, clk_sys;
#if defined(STM32_LSI_ENABLED)
	const uint32_t clk_lsi = stm32wl3_lsi_frequency;
#else
	const uint32_t clk_lsi = 0;
#endif


	ARG_UNUSED(dev);

	/* Obtain SYSCLK frequency by checking which source drives high-speed clock tree.
	 * If Direct HSE is enabled, the high-speed tree is clocked by HSE @ 32MHz.
	 * Otherwise, the high-speed tree is clocked by the RC64MPLL clock @ 64MHz.
	 *
	 * NOTE: it is NOT possible to use the usual 'SystemCoreClock * Prescaler' approach on
	 * STM32WL3 because the prescaler configuration is not affected by input clock variation:
	 * setting CLKSYSDIV = 1 results in 32MHz CLK_SYS, regardless of SYSCLK being 32 or 64MHZ.
	 */
	if (LL_RCC_DIRECT_HSE_IsEnabled()) {
		sysclk = STM32_HSE_FREQ;
	} else {
		sysclk = STM32_HSI_FREQ;
	}

	/* Obtain CLK_SYS (AHB0) frequency by using the CLKSYSDIV prescaler value.
	 *
	 * NOTE: LL_RCC_GetRC64MPLLPrescaler is strictly identical to LL_RCC_GetDirectHSEPrescaler
	 * and can be used regardless of which source is driving the high-speed clock tree.
	 *
	 * NOTE: the prescaler value must be interpreted as if source clock is 64MHz, regardless
	 * of which source is actually driving the high-speed clock tree. This allows using the
	 * following formula for calculations.
	 *
	 * NOTE: (x >> y) is equivalent to (x / 2^y) or (x / (1 << y)).
	 */
	clk_sys = CLOCK_FREQ_64MHZ >> LL_RCC_GetRC64MPLLPrescaler();

	/* Obtain slow clock tree source by reading RCC_CFGR->LCOSEL.
	 * From this, we can deduce at which frequency the slow clock tree is running.
	 */
	switch (LL_RCC_LSCO_GetSource()) {
	case LL_RCC_LSCO_CLKSOURCE_LSE:
		slow_clock = STM32_LSE_FREQ;
		break;
	case LL_RCC_LSCO_CLKSOURCE_LSI:
		slow_clock = clk_lsi;
		break;
	case LL_RCC_LSCO_CLKSOURCE_HSI64M_DIV2048:
		slow_clock = CLOCK_FREQ_64MHZ / 2048;
		break;
	default:
		__ASSERT(0, "Illegal slow clock source!");
		CODE_UNREACHABLE;
	}

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB0:
	/* All peripherals on AHB0 are clocked by CLK_SYS. */
		*rate = clk_sys;
		break;
	case STM32_CLOCK_BUS_APB0:
		return get_apb0_periph_clkrate(pclken, rate, slow_clock,
					       sysclk, clk_sys);
	case STM32_CLOCK_BUS_APB1:
		return get_apb1_periph_clkrate(pclken, rate, clk_sys);
	case STM32_SRC_SYSCLK:
		*rate = sysclk;
		break;
	case STM32_SRC_LSE:
		*rate = STM32_LSE_FREQ;
		break;
	case STM32_SRC_LSI:
		*rate = clk_lsi;
		break;
	case STM32_SRC_CLKSLOWMUX:
		*rate = slow_clock;
		break;
	case STM32_SRC_CLK16MHZ:
		*rate = CLOCK_FREQ_16MHZ;
		break;
	case STM32_SRC_CLK32MHZ:
		*rate = CLOCK_FREQ_32MHZ;
		break;
	default:
	case STM32_CLOCK_BUS_APB2:
		/* The only periperhal on APB2 is the MR_BLE radio. However,
		 * it is clocked by two sources that run at different frequencies,
		 * and we are unable to determine which one this API's caller cares
		 * about. For this reason, return ENOTSUP anyways.
		 *
		 * Note that since the only driver that might call this API is the
		 * Bluetooth driver, and since it can already determine both frequencies
		 * very easily, this should not pose any problem.
		 */
		return -ENOTSUP;
	}

	if (pclken->div) {
		*rate /= (pclken->div + 1);
	}

	return 0;
}

static enum clock_control_status stm32_clock_control_get_status(const struct device *dev,
								clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)sub_system;

	ARG_UNUSED(dev);

	if (IN_RANGE(pclken->bus, STM32_PERIPH_BUS_MIN, STM32_PERIPH_BUS_MAX)) {
		/* Bus / gated clock */
		if ((sys_read32(RCC_REG(pclken->bus)) & pclken->enr) == pclken->enr) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	} else {
		/* Domain clock */
		if (enabled_clock(pclken->bus) == 0) {
			return CLOCK_CONTROL_STATUS_ON;
		} else {
			return CLOCK_CONTROL_STATUS_OFF;
		}
	}
}

static DEVICE_API(clock_control, stm32_clock_control_api) = {
	.on = stm32_clock_control_on,
	.off = stm32_clock_control_off,
	.get_rate = stm32_clock_control_get_subsys_rate,
	.get_status = stm32_clock_control_get_status,
	.configure = stm32_clock_control_configure,
};

static void set_up_fixed_clock_sources(void)
{
	if (IS_ENABLED(STM32_HSE_ENABLED)) {
		/* Enable HSE */
		LL_RCC_HSE_Enable();
		while (LL_RCC_HSE_IsReady() != 1) {
			/* Wait for HSE ready */
		}
	}

	if (IS_ENABLED(STM32_HSI_ENABLED)) {
		/* Enable HSI if not enabled */
		if (LL_RCC_HSI_IsReady() != 1) {
			/* Enable HSI */
			LL_RCC_HSI_Enable();
			while (LL_RCC_HSI_IsReady() != 1) {
			/* Wait for HSI ready */
			}
		}
	}

	if (IS_ENABLED(STM32_LSI_ENABLED)) {
		LL_RCC_LSI_Enable();
		while (LL_RCC_LSI_IsReady() != 1) {
			/* Wait for LSI ready */
		}
	}

	if (IS_ENABLED(STM32_LSE_ENABLED)) {
#if STM32_LSE_DRIVING
		/* Configure driving capability */
		LL_RCC_LSE_SetDriveCapability(STM32_LSE_DRIVING << RCC_CSSWCR_LSEDRV_Pos);
#endif
		/* Unconditionally disable pull-up & pull-down on LSE pins */
		LL_PWR_SetNoPullB(LL_PWR_GPIO_BIT_12 | LL_PWR_GPIO_BIT_13);

		if (IS_ENABLED(STM32_LSE_BYPASS)) {
			/* Configure LSE bypass */
			LL_RCC_LSE_EnableBypass();
		}

		/* Enable LSE Oscillator (32.768 kHz) */
		LL_RCC_LSE_Enable();
		while (!LL_RCC_LSE_IsReady()) {
			/* Wait for LSE ready */
		}
	}
}

/* The STM32WL3 prescaler division factor defines vary depending on
 * whether SYSCLK runs at 32MHz (Direct HSE) or 64MHz (RC64MPLL).
 * The following helper macro wraps this difference.
 */
#if defined(STM32_SYSCLK_SRC_HSE)
#define LL_PRESCALER(x) LL_RCC_DIRECT_HSE_DIV_##x
#else
#define LL_PRESCALER(x) LL_RCC_RC64MPLL_DIV_##x
#endif

/**
 * @brief Converts the Kconfig STM32_WL3_CLKSYS_PRESCALER option
 * to a LL_RCC_RC64MPLL_DIV_x value understandable by the LL.
 */
static uint32_t kconfig_to_ll_prescaler(uint32_t kcfg_pre)
{
	switch (kcfg_pre) {
	case 1:
		return LL_PRESCALER(1);
	case 2:
		return LL_PRESCALER(2);
	case 4:
		return LL_PRESCALER(4);
	case 8:
		return LL_PRESCALER(8);
	case 16:
		return LL_PRESCALER(16);
	case 32:
		return LL_PRESCALER(32);
#if !defined(STM32_SYSCLK_SRC_HSE)
	/* A prescaler value of 64 is only valid when running
	 * off RC64MPLL because CLK_SYS must be at least 1MHz
	 */
	case 64:
		return LL_RCC_RC64MPLL_DIV_64;
#endif
	}
	CODE_UNREACHABLE;
}
#undef LL_PRESCALER	/* Undefine helper macro */

#if defined(STM32_LSI_ENABLED) && CONFIG_STM32WL3_LSI_RUNTIME_MEASUREMENT_INTERVAL != 0

/**
 * Since WL33xx series has extra devices on the SlowClock bus, we need to
 * reserve more space for the callbacks.
 */
#if defined(CONFIG_SOC_STM32WL33XX)
	#define NUM_SLOW_CLOCK_PERIPHERALS 6
#else
	#define NUM_SLOW_CLOCK_PERIPHERALS 4
#endif

/**
 * Reserve one slot for each slow clock peripheral to ensure each
 * peripheral's driver can register a callback to cope with clock drift.
 */
static lsi_update_cb_t lsi_update_callbacks[NUM_SLOW_CLOCK_PERIPHERALS];

int stm32wl3_register_lsi_update_callback(lsi_update_cb_t cb)
{
	for (size_t i = 0; i < ARRAY_SIZE(lsi_update_callbacks); i++) {
		if (lsi_update_callbacks[i] == NULL) {
			lsi_update_callbacks[i] = cb;
			return 0;
		}
	}
	return -ENOMEM;
}

static void lsi_rt_measure_loop(void)
{
	uint32_t old, new;

	while (1) {
		/* Sleep until calibration interval elapses */
		k_sleep(K_MSEC(CONFIG_STM32WL3_LSI_RUNTIME_MEASUREMENT_INTERVAL));

		old = stm32wl3_lsi_frequency;

		/* Ensure the MR_BLE IP clock is enabled. */
		if (!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_MRSUBG) || \
				!LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_LPAWUR)) {
			LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
			LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_LPAWUR);
		}

		/* Perform measurement, making sure we sleep on the semaphore
		 * signaled by the "measurement complete" interrupt handler
		 */
		measure_lsi_frequency();

		new = stm32wl3_lsi_frequency;

		/* If LSI frequency changed, invoke all registered callbacks */
		if (new != old) {
			for (size_t i = 0; i < ARRAY_SIZE(lsi_update_callbacks); i++) {
				if (lsi_update_callbacks[i]) {
					lsi_update_callbacks[i](stm32wl3_lsi_frequency);
				}
			}
		}
	}
}

#define LSI_RTM_THREAD_STACK_SIZE 256
#define LSI_RTM_THREAD_PRIORITY K_LOWEST_APPLICATION_THREAD_PRIO

K_KERNEL_THREAD_DEFINE(lsi_rt_measurement_thread,
		LSI_RTM_THREAD_STACK_SIZE,
		lsi_rt_measure_loop,
		NULL, NULL, NULL,
		LSI_RTM_THREAD_PRIORITY, /* No options */ 0,
		/* No delay (automatic start by kernel) */ 0);

#endif

int stm32_clock_control_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* Set flash latency according to target CLK_SYS frequency:
	 *  - 1 wait state when CLK_SYS > 32MHz (i.e., 64MHz configuration)
	 *  - 0 wait states otherwise (CLK_SYS <= 32MHz)
	 */
	LL_FLASH_SetLatency(
	(STM32_HCLK_FREQUENCY >= CLOCK_FREQ_32MHZ)
			? LL_FLASH_LATENCY_1
			: LL_FLASH_LATENCY_0
	);

	/* Unconditionally enable SYSCFG clock for other drivers */
	LL_APB0_GRP1_EnableClock(LL_APB0_GRP1_PERIPH_SYSCFG);

	/* Set up indiviual enabled clocks */
	set_up_fixed_clock_sources();

	/* Set up the slow clock mux */
#if defined(STM32_WL3_SLOWCLK_SRC)
	LL_RCC_LSCO_SetSource(STM32_WL3_SLOWCLK_SRC);
#endif

#if defined(STM32_SYSCLK_SRC_HSE)
	/* Select Direct HSE as SYSCLK source */
	LL_RCC_DIRECT_HSE_Enable();

	while (LL_RCC_DIRECT_HSE_IsEnabled() == 0) {
		/* Wait until Direct HSE is ready */
	}
#elif defined(STM32_SYSCLK_SRC_HSI) || defined(STM32_SYSCLK_SRC_PLL)
	/* Select RC64MPLL (HSI/PLL) block as SYSCLK source. */
	LL_RCC_DIRECT_HSE_Disable();

#	if defined(STM32_SYSCLK_SRC_PLL)
BUILD_ASSERT(IS_ENABLED(STM32_HSE_ENABLED),
	"STM32WL3 PLL requires HSE to be enabled!");

	/* Turn on the PLL part of RC64MPLL block */
	LL_RCC_RC64MPLL_Enable();
	while (LL_RCC_RC64MPLL_IsReady() == 0) {
		/* Wait until PLL is ready */
	}

#	endif /* STM32_SYSCLK_SRC_PLL */
#endif /* STM32_SYSCLK_SRC_* */

	/* Set CLK_SYS prescaler */
	LL_RCC_SetRC64MPLLPrescaler(
		kconfig_to_ll_prescaler(STM32_WL3_CLKSYS_PRESCALER));

	SystemCoreClock = STM32_HCLK_FREQUENCY;

#if defined(STM32_LSI_ENABLED)
	/* Enable MRSUBG clock for LSI measurement.
	 * This is needed because we use part of the MRSUBG hardware
	 * to perform this measurement. Just in case enable LPAWUR too.
	 */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_LPAWUR);

	/* Perform a measure of the LSI frequency */
	measure_lsi_frequency();

#endif

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_LPAWUR);

	return 0;
}

/**
 * @brief Reset & Clock Controller device
 * Note that priority is intentionally set to 1,
 * so that RCC init runs just after SoC init.
 */
DEVICE_DT_DEFINE(STM32_CLOCK_CONTROL_NODE,
		    &stm32_clock_control_init,
		    NULL, NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &stm32_clock_control_api);
