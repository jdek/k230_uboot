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
#ifndef __K230_BOARD_H__
#define __K230_BOARD_H__
//uboo使用的是 128M@128M  0x8000000---0x100000000
//假设：密文32MB；明文32M,解压缩后32M, ----
/*
    +0x8000000=0x10000000 128MB
    +0x6100000=0xe100000  16MB@97MB uboot
    +0x4100000=0xc100000  32MB@65MB  plaint  2/3
    +0x2100000=0xa1000000 32MB@33MB  cipher  1/3
0x8000000                 kernel 
*/
unsigned long get_CONFIG_CIPHER_ADDR(void);
unsigned long get_CONFIG_PLAIN_ADDR(void);
#define CONFIG_CIPHER_ADDR  get_CONFIG_CIPHER_ADDR()
#define CONFIG_PLAIN_ADDR   get_CONFIG_PLAIN_ADDR()

#define BLKSZ 512
#define HD_BLK_NUM   DIV_ROUND_UP(sizeof(image_header_t), BLKSZ)

void ddr_init_training(void);

void cache_flush(void);
void record_boot_time_info_to_sram(char *prompt);
//引导系统前需要调用一直这个函数，后面只能调用这个函数
void record_boot_time_info(char *prompt);
int do_timeinfo(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[]);

int k230_img_load_uboot(void);
#endif 
