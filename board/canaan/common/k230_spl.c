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
#include <asm-generic/sections.h>

#include <linux/kernel.h>
#include <init.h>
#include "k230_board_common.h"
#include "mmc.h"

//weak
void board_boot_order(u32 *spl_boot_list)
{
    spl_boot_list[0] = BOOT_DEVICE_MMC2;
    spl_boot_list[1] = BOOT_DEVICE_MMC1;
}

//weak
void spl_board_prepare_for_boot(void){
    cache_flush();
    icache_disable();
    dcache_disable();
    // csi_l2cache_flush_invalid();
    asm volatile(".long 0x0170000b\n":::"memory");
}

static void device_disable(void)
{
    uint32_t value;

    // disable ai power
    if (readl(0x9110302c) & 0x2)
        writel(0x30001, 0x91103028);
    // disable vpu power
    if (readl(0x91103080) & 0x2)
        writel(0x30001, 0x9110307c);
    // disable dpu power
    if (readl(0x9110310c) & 0x2)
        writel(0x30001, 0x91103108);
    // disable disp power
    if (readl(0x91103040) & 0x2)
        writel(0x30001, 0x9110303c);
    // check disable status
    value = 1000000;
    while (!(readl(0x9110302c) & 0x1) || !(readl(0x91103080) & 0x1) ||
        !(readl(0x9110310c) & 0x1) || !(readl(0x91103040) & 0x1) && value)
        value--;
    // disable ai clk
    value = readl(0x91100008);
    value &= ~((1 << 0));
    writel(value, 0x91100008);
    // disable vpu clk
    value = readl(0x9110000c);
    value &= ~((1 << 0));
    writel(value, 0x9110000c);
    // disable dpu clk
    value = readl(0x91100070);
    value &= ~((1 << 0));
    writel(value, 0x91100070);
    // disable mclk
    value = readl(0x9110006c);
    value &= ~((1 << 0) | (1 << 1) | (1 << 2));
    writel(value, 0x9110006c);
}

//weak;
int spl_board_init_f(void)
{
    int ret = 0;

    device_disable();

    record_boot_time_info_to_sram("ds");
    ddr_init_training();
    record_boot_time_info_to_sram("dd");
    /* Clear the BSS. */
    //record_boot_time_info_to_sram("bs");
    memset(__bss_start, 0, (ulong)&__bss_end - (ulong)__bss_start);
    //record_boot_time_info_to_sram("be");

   

    // /* load/boot image from boot device */
    ret = k230_img_load_boot_sys(BOOT_SYS_UBOOT);
    if(ret )
        printf("uboot boot failed\n");
    //while(1);
    //board_init_r(NULL, 0);
    return ret;
}
