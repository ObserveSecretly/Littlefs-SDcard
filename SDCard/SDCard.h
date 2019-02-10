/*----------------------------------------------------------------------------
 * Name:    sdcard.c
 * Purpose: MCBTMS570 low level SDCard definitions
 * Version: V1.00
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#ifndef __SDCARD_H
#define __SDCARD_H
#include "sys_common.h"

//typedef unsigned int   uint32_t;
//typedef unsigned short uint16_t;
//typedef unsigned char  uint8_t;

/* MMC device configuration */      /* see FILE_CONFIG.H */
typedef struct sdc_cfg {
   uint32 sernum;
   uint32 capacity;
   uint32 sector_size;
   uint16 read_blen;
   uint16 write_blen;
} SDC_CFG;

extern  SDC_CFG SDC_cfg;

extern int            SDC_init(void);
extern void           SDC_UnInit (void);
extern int            SDC_rdCfg (SDC_CFG *cfg);
extern void           SPI_hiSpeed         (int on);
extern void           SPI_ssel            (unsigned long ssel);
extern unsigned char  SPI_send            (unsigned char outb);
void print_r1(unsigned char r1);
extern int mmc_read_sect  (uint32_t sect, uint8_t *buf, uint32_t cnt);
extern int mmc_write_sect (uint32_t sect, uint8_t *buf, uint32_t cnt);

#endif
