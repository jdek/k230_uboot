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
#include <mmc.h>
#include <spi_flash.h>
#include <dm/device-internal.h>
#include <spl.h>
#include <spi.h>
#include <nand.h>
#include <linux/mtd/mtd.h>
#include <linux/delay.h>

#define RAM_SIZE (1 << 29)

unsigned long get_CONFIG_CIPHER_ADDR(void)
{
    return round_down((RAM_SIZE - 0x1000000)/3, 0x100000);//25MB
}

#define OPENSBI_DTB_ADDR 0x2000000LU

#define SUPPORT_MMC_LOAD_BOOT

static int k230_boot_decomp_to_load_addr(image_header_t *pUh, ulong des_len, ulong data , ulong *plen)
{
    int ret = 0;
    K230_dbg("imge: %s load to %x  compress =%x  src %lx len=%lx \n", image_get_name(pUh), image_get_load(pUh), image_get_comp(pUh), data, *plen);    

    //设计要求必须gzip压缩，调试可以不压缩；
    if (image_get_comp(pUh) == IH_COMP_GZIP) {        
        ret = gunzip((void *)(ulong)image_get_load(pUh), des_len, (void *)data, plen);
        if(ret){
            printf("unzip fialed ret =%x\n", ret);
            return -1;
        }
    }else if(image_get_comp(pUh) == IH_COMP_NONE){
        memmove((void *)(ulong)image_get_load(pUh), (void *)data, *plen);
    }
    flush_cache(image_get_load(pUh), *plen);
    K230_dbg("imge: %s load to %x  compress =%x  src %lx len=%lx  \n", image_get_name(pUh), image_get_load(pUh), image_get_comp(pUh), data, *plen);    
    return ret;
}
static int  de_reset_big_core(ulong core_run_addr)
{
    /*
    p/x *(uint32_t*)0x9110100c
    set  *(uint32_t*)0x9110100c=0x10001000    //清 done bit
    set *(uint32_t*)0x9110100c=0x10001       //设置 reset bit
    p/x *(uint32_t*)0x9110100c
    set   *(uint32_t*)0x9110100c=0x10000       //清 reset bit
    p/x *(uint32_t*)0x9110100c
    */
    writel(core_run_addr, (void*)0x91102104ULL);//cpu1_hart_rstvec 设置大核的解复位向量，复位后程序执行位置；
    //printf("0x91102104 =%x 0x9110100c=%x\n", readl( (void*)0x91102104ULL), readl( (void*)0x9110100cULL));

    //writel(0x80199805, (void*)0x91100004); //1.6Ghz

    writel(0x10001000, (void*)0x9110100cULL); //清 done bit
    writel(0x10001, (void*)0x9110100cULL); //设置 reset bit
    //printf("0x9110100c =%x\n", readl( (void*)0x9110100cULL));    
    writel(0x10000, (void *)0x9110100cULL); ////清 reset bit  
    //printf("0x9110100c =%x\n", readl( (void*)0x9110100cULL));
    //printf("reset big hart\n");
    return 0;
}

/**
 * @brief 
 * 
 * @param pUh  image_header_t *
 * @return int 
 */
static int k230_boot_uboot_uimage(image_header_t *pUh)
{
    int ret = 0;
    void (*uboot)(ulong hart, void *dtb);
    ulong len = image_get_data_size(pUh);
    ulong data = image_get_data(pUh);


    ret = k230_boot_decomp_to_load_addr(pUh, 0x6000000,  data, &len );
    if(ret == 0){
        icache_disable();
        dcache_disable();
        // csi_l2cache_flush_invalid();
        asm volatile(".long 0x0170000b\n":::"memory");

        uboot = (void (*)(ulong, void *))(ulong)image_get_load(pUh);
        //do_timeinfo(0,0,0,0); 
        #if 1 // Hand-over to the big core...
        de_reset_big_core(image_get_load(pUh));
        while(1) udelay(100);
        #else
        uboot(0, (void*)OPENSBI_DTB_ADDR);
        #endif
    }
    return 0;
}

//mmc


#ifdef SUPPORT_MMC_LOAD_BOOT
static int k230_load_uboot_from_mmc_or_sd(ulong buff)
{
    static struct blk_desc *pblk_desc;
    ulong blk_s = CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
    struct mmc *mmc;
    int ret = 0;
    image_header_t *pUh = (void *)buff;
    ulong data_sect = 0;

        if(mmc_init_device(1)) //mmc_init_device
            return 1;

        mmc = find_mmc_device(1);
        if(NULL == mmc)
            return 2;
        if(mmc_init(mmc))
            return 3;

        pblk_desc = mmc_get_blk_desc(mmc);
        if(NULL == pblk_desc)
            return 3;

    ret = blk_dread(pblk_desc, blk_s , HD_BLK_NUM, (char *)buff);
    if(ret != HD_BLK_NUM)
        return 4;

    if (!image_check_magic(pUh)) {
        printf("bad magic blk=0x%lx buff=0x%lx", blk_s, buff);
        return 5;
    }

    data_sect = DIV_ROUND_UP(image_get_image_size(pUh), BLKSZ) - HD_BLK_NUM;
	
    ret = blk_dread(pblk_desc, blk_s + HD_BLK_NUM, data_sect, (char *)buff + HD_BLK_NUM * BLKSZ);
    if(ret != data_sect)
        return 6;
    return 0;
}

#endif  //SUPPORT_MMC_LOAD_BOOT

int k230_img_load_uboot(void)
{
    int ret = k230_load_uboot_from_mmc_or_sd(CONFIG_CIPHER_ADDR);
    if (ret) {
        printf("u-boot load error ret=%x\n", ret);
        return ret;
    }
    return k230_boot_uboot_uimage((void *)CONFIG_CIPHER_ADDR);
}
