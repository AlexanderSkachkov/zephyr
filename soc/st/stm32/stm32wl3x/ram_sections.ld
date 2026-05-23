/*
 * Copyright (c) 2024 STMicroelectronics
 * Copyright (c) 2026 Movu robotics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** Refer to `soc.c` for more information about these areas. */
SECTION_PROLOGUE(.ram_vr, 0x20000000 (NOLOAD), )
{
	/* For historical reasons, leave the first word of
	 * SRAM0 unused, even though it could store data.
	 * The structure MUST start at address 0x2000_0004.
	 */
	. += 4;

	KEEP(*(.ram_vr));
} GROUP_LINK_IN(RAMABLE_REGION)