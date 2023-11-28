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

static int k230_check_and_get_plain_data(firmware_head_s *pfh, ulong *pplain_addr);
static int k230_boot_rtt_uimage(image_header_t *pUh);

#define LINUX_KERNEL_IMG_MAX_SIZE  (25*1024*1024)
#define RAM_SIZE (1 << 29)

unsigned long get_CONFIG_CIPHER_ADDR(void)
{
    return round_down((RAM_SIZE - 0x1000000)/3, 0x100000);//25MB
}
unsigned long get_CONFIG_PLAIN_ADDR(void)
{
    return round_down((RAM_SIZE - 0x1000000)/3*2, 0x100000);
}

#define USE_UBOOT_BOOTARGS 
#define OPENSBI_DTB_ADDR 0x2000000LU
#define RAMDISK_ADDR (0x2000000LU + 0X100000)

#define SUPPORT_MMC_LOAD_BOOT

#ifdef USE_UBOOT_BOOTARGS
//weak function
char *board_fdt_chosen_bootargs(void){
    char *bootargs = env_get("bootargs");
    if(NULL == bootargs)
        bootargs = "root=/dev/mmcblk1p6 loglevel=8 rw rootdelay=4 rootfstype=ext4 console=ttyS0,115200 crashkernel=256M-:128M earlycon=sbi";
    //printf("%s\n",bootargs);
    return bootargs;
}
#endif 
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
static int k230_boot_rtt_uimage(image_header_t *pUh)
{
    int ret = 0;
    //小核是0，大核是1；
    ulong len = image_get_size(pUh);
    ulong data;

    image_multi_getimg(pUh, 0, &data, &len);
    ret = k230_boot_decomp_to_load_addr(pUh, 0x6000000, data, &len );
    if( ret == 0){
        de_reset_big_core(image_get_load(pUh));
    }
    return 0;
}

static int k230_boot_linux_uimage(image_header_t *pUh)
{
    int ret = 0;
    void (*kernel)(ulong hart, void *dtb);

    //小核是0，大核是1；
    ulong len = image_get_size(pUh);
    ulong data;
    ulong dtb;
    ulong rd,rd_len;
    ulong img_load_addr = 0;


    //record_boot_time_info("gd");
    image_multi_getimg(pUh, 0, &data, &len);
    img_load_addr = (ulong)image_get_load(pUh);

    ret = k230_boot_decomp_to_load_addr(pUh, (ulong)pUh-img_load_addr,  data, &len );
    if( ret == 0){
          //dtb
        image_multi_getimg(pUh, 2, &dtb, &len);
        image_multi_getimg(pUh, 1, &rd, &rd_len);
        #ifdef USE_UBOOT_BOOTARGS
        len = fdt_shrink_to_minimum((void*)dtb,0x100);
        ret = fdt_chosen((void*)dtb);
        #endif 
        memmove((void*)OPENSBI_DTB_ADDR, (void *)dtb, len);

        #ifndef CONFIG_SPL_BUILD
        //run_command("fdt addr 0x91e7000;fdt print;", 0);
        #endif 

        if(rd_len > 0x100 )
            memmove((void*)RAMDISK_ADDR, (void *)rd, rd_len);

        K230_dbg("dtb %lx rd=%lx l=%lx  %lx %lx ci%lx %lx \n", dtb,rd, data, OPENSBI_DTB_ADDR, RAMDISK_ADDR, get_CONFIG_CIPHER_ADDR(),get_CONFIG_PLAIN_ADDR());

        cleanup_before_linux();//flush cache，
        kernel = (void (*)(ulong, void *))img_load_addr;
        //do_timeinfo(0,0,0,0); 
        kernel(0, (void*)OPENSBI_DTB_ADDR);

    }
    return ret;
}
/**
 * @brief 
 * 
 * @param pUh  image_header_t *
 * @return int 
 */
static int k230_boot_paramter_uimage(image_header_t *pUh)
{
    int ret = 0;
    ulong len = image_get_data_size(pUh);
    ulong data = image_get_data(pUh);

    ret = k230_boot_decomp_to_load_addr(pUh, 0x6000000,  data, &len );
    return ret;
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
int k230_img_boot_sys_bin(firmware_head_s * fhBUff)
{
    int ret = 0;
    image_header_t * pUh = NULL;
    ulong plain_addr  = 0;

    //解密
    //record_boot_time_info("ds");
    ret = k230_check_and_get_plain_data((firmware_head_s *)fhBUff,  &plain_addr);
    if (ret )
        return ret;

    pUh = (image_header_t *) (plain_addr + 4);//4字节是版本号
    if (!image_check_magic(pUh)){
        printf("bad magic \n");
        return -3;
    }        
    //解压缩，引导；
     if ( (0 == strcmp(image_get_name(pUh), "linux") ) || (0 == strcmp(image_get_name(pUh), "Linux") ) ){ 
        ret = k230_boot_linux_uimage(pUh);
    }else if(0 == strcmp(image_get_name(pUh), "rtt")){        
        ret = k230_boot_rtt_uimage(pUh);       
    }else if(0 == strcmp(image_get_name(pUh), "uboot")){        
        ret = k230_boot_uboot_uimage(pUh);       
    } else  {        
        k230_boot_paramter_uimage(pUh);
        return 0;
    }
    return 0;
}

//去掉k230 fireware头信息，完整性校验，解密；
static int k230_check_and_get_plain_data(firmware_head_s *pfh, ulong *pplain_addr)
{

    uint32_t otp_msc = 0;    
    int ret = 0;

    if(pfh->magic != MAGIC_NUM){
        printf("magic error %x : %x \n", MAGIC_NUM, pfh->magic);
        return CMD_RET_FAILURE;
    }

    if(pfh->crypto_type == NONE_SECURITY){ 
        if(otp_msc & 0x1){
            printf(" NONE_SECURITY not support  %x \n", pfh->crypto_type);
            return -5;
        }       
        if(pplain_addr) 
            *pplain_addr = (ulong)pfh + sizeof(*pfh) ; 

        ret = 0;    
    } else  {
        printf("error crypto type =%x\n", pfh->crypto_type);
        return -9;
    } 

    return ret;
}

//mmc


#ifdef SUPPORT_MMC_LOAD_BOOT
static ulong get_blk_start_by_boot_firmre_type(en_boot_sys_t sys)
{
    ulong blk_s = IMG_PART_NOT_EXIT;
    switch (sys){
	case BOOT_SYS_LINUX:
		blk_s = LINUX_SYS_IN_IMG_OFF_SEC;
		break;
	case BOOT_SYS_RTT:
		blk_s = RTT_SYS_IN_IMG_OFF_SEC;
		break;
    case BOOT_SYS_UBOOT:
		blk_s = CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR;
		break;
    default:break;
	} 
    return blk_s;
}

//dev ,linux ,buff
//sysctl_boot_mode_e bmode, en_boot_sys_t sys, ulong buff
static int k230_load_sys_from_mmc_or_sd(en_boot_sys_t sys, ulong buff)//(ulong offset ,ulong buff)
{
    static struct blk_desc *pblk_desc = NULL;
    ulong blk_s  = get_blk_start_by_boot_firmre_type(sys);
    struct mmc * mmc=NULL;
    int ret = 0;
    firmware_head_s *pfh = (firmware_head_s *)buff;
    ulong data_sect = 0;

    if( IMG_PART_NOT_EXIT == blk_s )
        return IMG_PART_NOT_EXIT;

    if(NULL == pblk_desc){
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
    }

    ret = blk_dread(pblk_desc, blk_s , HD_BLK_NUM, (char *)buff);
    if(ret != HD_BLK_NUM)
        return 4;

    if(pfh->magic != MAGIC_NUM){
        K230_dbg("pfh->magic 0x%x != 0x%x blk=0x%lx buff=0x%lx  ", pfh->magic, MAGIC_NUM, blk_s, buff);
        return 5;
    }

    data_sect = DIV_ROUND_UP(pfh->length + sizeof(*pfh), BLKSZ) - HD_BLK_NUM;
	
    ret = blk_dread(pblk_desc, blk_s + HD_BLK_NUM, data_sect, (char *)buff + HD_BLK_NUM * BLKSZ);
    if(ret != data_sect)
        return 6;
    return 0;
}

#endif  //SUPPORT_MMC_LOAD_BOOT

int k230_img_load_sys_from_dev(en_boot_sys_t sys, ulong buff)
{
    return k230_load_sys_from_mmc_or_sd(sys, buff);
}

static int k230_img_load_boot_sys_auot_boot(en_boot_sys_t sys)
{
    int ret = 0;
    ret += k230_img_load_boot_sys(BOOT_SYS_LINUX);
    return ret;
}
/**
 * @brief 
 * 
 * @param bmode 
 * @param sys 
 * @param buff 
 * @return int 
 */
int k230_img_load_boot_sys(en_boot_sys_t sys)
{
    int ret = 0;

    if(sys == BOOT_SYS_AUTO)
        ret =  k230_img_load_boot_sys_auot_boot(sys);
    else {
        ret = k230_img_load_sys_from_dev(sys, CONFIG_CIPHER_ADDR);
        if(ret){
            if(ret != IMG_PART_NOT_EXIT)
                printf("sys %x  load error ret=%x\n", sys, ret);
            return ret;
        }
        ret = k230_img_boot_sys_bin((firmware_head_s * )CONFIG_CIPHER_ADDR);
    }
    return ret;
}

