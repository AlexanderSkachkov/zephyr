/*
 * Copyright (c) 2020 STMicroelectronics
 * Copyright (c) 2026 Movu robotics
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the STM32WL family processors.
 *
 * Based on reference manual:
 *   STM32WL3x advanced ARM ® -based 32-bit MCUs
 *
 * Chapter 2.6: Memory organization
 */


#ifndef _STM32WL3X_SOC_H_
#define _STM32WL3X_SOC_H_

#ifndef _ASMLANGUAGE

#include <stm32wl3x.h>

/** SMPS modes */
#define STM32WL3_SMPS_MODE_OFF		    0
#define STM32WL3_SMPS_MODE_PRECHARGE    1
#define STM32WL3_SMPS_MODE_RUN		    2

/** Active SMPS mode (provided here for usage in drivers) */
#define SMPS_MODE	_CONCAT(STM32WL3_SMPS_MODE_,			\
				DT_STRING_UNQUOTED(			                \
					DT_INST(0, st_stm32wl3_pwr),	        \
					smps_mode))

#endif /* !_ASMLANGUAGE */

#endif /* _STM32WL3X_SOC_H_ */
