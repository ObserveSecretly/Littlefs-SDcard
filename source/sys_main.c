/** @file sys_main.c 
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
#include "system.h"
#include "pinmux.h"
#include "sci.h"
#include "spi.h"
#include "lfs.h"
#include "SDCard.h"
/* USER CODE END */

/* Include Files */

#include "sys_common.h"
#include "stdio.h"

/* USER CODE BEGIN (1) */
SDC_CFG SD_cfg;
lfs_t lfs;
lfs_file_t file[3];
lfs_dir_t dir[2];

uint8_t lfs_read_buf[512];
uint8_t lfs_prog_buf[512];
uint8_t lfs_lookahead_buf[16];  // 128/8=16
uint8_t lfs_file_buf[512];
uint8 blen[10];

/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
static int user_provided_block_device_read(
    const struct lfs_config *lfsc, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    mmc_read_sect((block + off), (uint8_t*)buffer, size/512);
    return LFS_ERR_OK;
}

static int user_provided_block_device_prog(
    const struct lfs_config *lfsc, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    mmc_write_sect((block + off), (uint8_t*)buffer, size/512);
    return LFS_ERR_OK;
}

static int user_provided_block_device_erase(const struct lfs_config *lfsc, lfs_block_t block)
{
    return LFS_ERR_OK;
}

static int user_provided_block_device_sync(const struct lfs_config *lfsc)
{
    return LFS_ERR_OK;
}

int lfs_block_count(void *p, lfs_block_t block)
{
    p += 1;
    return 0;
}

struct lfs_config cfg = {
    // block device operations
    .read  = user_provided_block_device_read,
    .prog  = user_provided_block_device_prog,
    .erase = user_provided_block_device_erase,
    .sync  = user_provided_block_device_sync,

    // block device configuration
    .read_size = 512,
    .prog_size = 512,//每次读写均为512byte
    .block_size = 512,//每次可擦除512byte数据
    .block_count = 61046780,
    .lookahead = 128,//必须是32的整数倍


    .read_buffer = lfs_read_buf,
    .prog_buffer = lfs_prog_buf,
    .lookahead_buffer = lfs_lookahead_buf,
    .file_buffer = lfs_file_buf,
};

void sciDisplayText(sciBASE_t *sci, uint8 *text);
void lfs_free_space(void);
void DisplaySD_cfg(void);
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    uint32 i,r1;
    uint8 lfs_write_buff1[] =
            "    Little file system\nThis file's path is E:/test .\nThis is a test: whether file system work if file size larger than 512 byte.\n    TEST BEGIN\ndata1  data2  data3  data4  data5\ndata6  data7  data8  data9  data10\ndata11 data12 data13 data14 data15\ndata16 data17 data18 data19 data20\ndata21 data22 data23 data24 data25\ndata26 data27 data28 data29 data30\ndata31 data32 data33 data34 data35\ndata36 data37 data38 data39 data40\ndata41 data42 data43 data44 data45\ndata46 data47 data48 data49 data50\ndata51 data52 data53 data54 data55\ndata56 data57 data58 data59 data60\ndata61 data62 data63 data64 data65\ndata66 data67 data68 data69 data70\ndata71 data72 data73 data74 data75\ndata76 data77 data78 data79 data80\nOK, test successful.\n\r";
    uint8 lfs_write_buff2[] = "    Little file system\nThis file's path is E:/data/data1\n\r";
    uint8 lfs_write_buff3[] = "    Little file system\nThis file's path is E:/data/data2\n\r";
    uint8 lfs_write_buff4[] = "    Little file system\nsomething wrong";
    uint8 lfs_read_buff[1024];

    muxInit();
    sciInit();
    spiInit();

    for(i=0;i<100;i++){
        r1=SDC_init();
        if(r1){
            break;
        }
    }

    DisplaySD_cfg();
    // print the boot count
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }
/*
    err = lfs_dir_open(&lfs,&dir[0],"E:");
    if(err == LFS_ERR_NOTDIR){
        lfs_mkdir(&lfs,"E:");
    }

    err = lfs_dir_open(&lfs,&dir[1],"E:/data");
    if(err == LFS_ERR_NOTDIR){
        lfs_mkdir(&lfs,"E:/data");
    }
*/
    // open/read a file

    lfs_file_open(&lfs, &file[0], "test", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&lfs, &file[0], lfs_write_buff1, sizeof(lfs_write_buff1));
    lfs_file_rewind(&lfs, &file[0]);
    lfs_file_read(&lfs, &file[0], lfs_read_buff, sizeof(lfs_read_buff));
    sciDisplayText(sciREG,lfs_read_buff);
    lfs_file_close(&lfs, &file[0]);

    lfs_file_open(&lfs, &file[1], "data1", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&lfs, &file[1], lfs_write_buff2, sizeof(lfs_write_buff2));
    lfs_file_rewind(&lfs, &file[1]);
    lfs_file_read(&lfs, &file[1], lfs_read_buff, sizeof(lfs_read_buff));
    sciDisplayText(sciREG,lfs_read_buff);
    lfs_file_close(&lfs, &file[1]);

    lfs_file_open(&lfs, &file[2], "data2", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_write(&lfs, &file[2], lfs_write_buff3, sizeof(lfs_write_buff3));
    lfs_file_rewind(&lfs, &file[2]);
    lfs_file_read(&lfs, &file[2], lfs_read_buff, sizeof(lfs_read_buff));
    sciDisplayText(sciREG,lfs_read_buff);
    lfs_file_close(&lfs, &file[2]);

    lfs_free_space();

    lfs_unmount(&lfs);

/* USER CODE END */
    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */

void sciDisplayText(sciBASE_t *sci, uint8 *text)
{
    while(*text != '\0')
    {
        while ((sciREG->FLR & 0x4) == 4); /* wait until busy */
        sciSendByte(sciREG,*text++);      /* send out text   */
    };
}

void lfs_free_space(void)
{
    lfs_size_t use_space,free_space;
    int err = lfs_traverse(&lfs,lfs_block_count,&use_space);
    free_space = cfg.block_count - use_space;
    sprintf(blen,"%d",free_space);
    sciDisplayText(sciREG,"Free space size:\r");
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," blocks\r");

    free_space = free_space / 2048;
    sprintf(blen,"%d",free_space);
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," Mb\r");    //显示写入数据时block大小
}

void DisplaySD_cfg(void)
{
    SDC_rdCfg(&SD_cfg);

    sprintf(blen,"%d",SD_cfg.write_blen);
    sciDisplayText(sciREG,"SDcard's write length is:");
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," byte.\r");    //显示写入数据时block大小

    sprintf(blen,"%d",SD_cfg.read_blen);
    sciDisplayText(sciREG,"SDcard's read length is:");
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," byte.\r");    //显示读取数据时block大小

    sprintf(blen,"%d",SD_cfg.sector_size);
    sciDisplayText(sciREG,"A Sector has ");
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," blocks.\r");//显示SD卡的容量

    sprintf(blen,"%d",SD_cfg.capacity);
    sciDisplayText(sciREG,"SDcard's capacity is:");
    sciDisplayText(sciREG,&blen[0]);
    sciDisplayText(sciREG," Mb.\n\r");//显示SD卡的容量
}
