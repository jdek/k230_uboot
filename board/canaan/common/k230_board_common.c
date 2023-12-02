/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <asm/asm.h>
#include <asm/io.h>
#include <asm/types.h>
#include <lmb.h>
#include <cpu_func.h>
#include <stdio.h>
#include <common.h>
#include <command.h>
#include <image.h>
#include <gzip.h>
#include <asm/spl.h>
#include "sysctl.h"

#include <linux/kernel.h>
#include "k230_board_common.h"
#include <env_internal.h>

//weak
int mmc_get_env_dev(void)
{
    return 1;
}
//weak
enum env_location arch_env_get_location(enum env_operation op, int prio)
{
    if(0 != prio){
        return ENVL_UNKNOWN;
    }

	return ENVL_MMC;
}
#ifndef CONFIG_SPL_BUILD
int board_early_init_f(void)
{
    return 0;
}

__weak int board_init(void)
{
	#define USB_IDPULLUP0 		(1<<4)
	#define USB_DMPULLDOWN0 	(1<<8)
	#define USB_DPPULLDOWN0 	(1<<9)

    u32 usb0_test_ctl3 = readl((void*)USB0_TEST_CTL3);
	u32 usb1_test_ctl3 = readl((void*)USB1_TEST_CTL3);

	usb0_test_ctl3 |= USB_IDPULLUP0;
	usb1_test_ctl3 |= USB_IDPULLUP0;

	writel(usb0_test_ctl3, (void*)USB0_TEST_CTL3);
	writel(usb1_test_ctl3, (void*)USB1_TEST_CTL3);

    #define SD_HOST_REG_VOL_STABLE      (1<<4)
    #define SD_CARD_WRITE_PROT          (1<<6)
    u32 sd0_ctrl = readl((void*)SD0_CTRL);
    sd0_ctrl |= SD_HOST_REG_VOL_STABLE | SD_CARD_WRITE_PROT;
    writel(sd0_ctrl, (void*)SD0_CTRL);
	return 0;
}
#endif
