/*----------------------------------------------------------------------------
 * Name:    sdcard.c
 * Purpose: MCBTMS570 low level SDCard functions
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

#include "SDCard.h"
#include "spi.h"
#include "pinmux.h"

SDC_CFG SDC_cfg;

/* Local variables */
static uint8_t CardType;


/* SSPxSR - bit definitions. */
#define TFE     0x01
#define TNF     0x02
#define RNE     0x04
#define RFF     0x08
#define BSY     0x10


/* SD/MMC Commands */
#define GO_IDLE_STATE    (0x40 + 0)
#define SEND_OP_COND     (0x40 + 1)
#define SEND_CSD         (0x40 + 9)
#define SEND_CID         (0x40 + 10)
#define STOP_TRAN        (0x40 + 12)
#define SET_BLOCKLEN     (0x40 + 16)
#define READ_BLOCK       (0x40 + 17)
#define READ_MULT_BLOCK  (0x40 + 18)
#define WRITE_BLOCK      (0x40 + 24)
#define WRITE_MULT_BLOCK (0x40 + 25)
#define APP_CMD          (0x40 + 55)
#define CRC_ON_OFF       (0x40 + 59)
#define SD_SEND_OP_COND  (0x40 + 41)
#define CMD8             (0x40 + 8)

/* Wait timeouts, in multiples of 1 byte send over SPI */
#define WR_TOUT           1600000        /* ~ 200 ms with SPI clk 80MHz */
#define STOP_TOUT         400000        /* ~  50 ms with SPI clk 80MHz */
#define CMD_TOUT          4000          /* ~ 0.5 ms with SPI clk 80MHz */

/*----------------------------------------------------------------------------
 *      MMC Driver Functions
 *----------------------------------------------------------------------------
 *  Required functions for MMC driver module:
 *   - int mmc_init        ()
 *   - int mmc_read_sect   (uint32_t sect, uint8_t *buf, uint32_t cnt)
 *   - int mmc_write_sect  (uint32_t sect, uint8_t *buf, uint32_t cnt)
 *   - int mmc_read_config (MMCFG *cfg)
 *---------------------------------------------------------------------------*/

/* Card Type definitions */
#define CARD_NONE       0
#define CARD_MMC        1
#define CARD_SD         2


/* Local Function Prototypes */
static uint8_t mmc_command     (uint8_t cmd, uint32_t arg);
static int     mmc_read_bytes  (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t len);
static int     mmc_read_block  (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t cnt);
static int     mmc_write_block (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t cnt);


/*--------------------------- mmc_init --------------------------------------*/

#include "sci.h"

void print_r1(unsigned char r1){

	////
	   while(!sciIsTxReady(sciREG));
	   sciSend		(sciREG, 4, (unsigned char *) "R1: ");
	      //> 9, 55
	      //< 9 48
	      if(((0xF0 & r1)>> 4) < 9){
	    	  while(!sciIsTxReady(sciREG));
	      	  sciSendByte(sciREG, ((0xF0 & r1)>>4) + 48);
	      }
	      else{
	    	  while(!sciIsTxReady(sciREG));
	      	  sciSendByte(sciREG, ((0xF0 & r1)>>4) + 55);
	      }
	      if((0x0F & r1) < 9){
	    	  while(!sciIsTxReady(sciREG));
	      	  sciSendByte(sciREG, (0x0F & r1) + 48);
	      }
	      else{
	       	  while(!sciIsTxReady(sciREG));
	       	  sciSendByte(sciREG, (0x0F & r1) + 55);
	      }
	      while(!sciIsTxReady(sciREG));
	      sciSend		(sciREG, 1, (unsigned char *) "\r");
	   ////
}

int SDC_init (void) {
	
	sciSend		(sciREG, 28, (unsigned char *) "SDC initialization started.\r");

   /* Unlock pinmxing registers */
	muxInit();
//   Gladiator_IOMM_UNLOCK();
	
   /* Set muxing for MibSPI3 */
//   Gladiator_PINMUX_MIBSPI3_MUX_PINS();
//   Gladiator_PINMUX_SPI2_MUX_PINS();
//   Gladiator_PINMUX_SPI4_MUX_PINS();
   /* Lock pinmux registers */
//   Gladiator_IOMM_LOCK();
	
   /* Initialize and enable the Flash Card. */
   uint32_t i,r1;

   /* Initialize SPI interface and enable Flash Card SPI mode. */
   SPI_ssel (1);
   SPI_hiSpeed (0);

   /* Send SPI Command with card not selected at 400 KBit. */
   for (i = 0; i < 16; i++) {
      SPI_send (0xFF);
   }

   /* Reset the card, send CMD0. */
   SPI_ssel (0);
   
   r1 = mmc_command (GO_IDLE_STATE, 0);
   print_r1(r1);
   for (i = 0; i < 100; i++) {
      if (r1 == 0x01) {
         break;
      }
      r1 = SPI_send (0xFF);
   }

   SPI_ssel (1);

   print_r1(r1);

   if (r1 != 0x01) {
	   sciSend		(sciREG, 28, (unsigned char *) "Failed to Reset Flash Card.\r");

      /* Failed to Reset Flash Card. */
      return (0);
   }
   SPI_hiSpeed (1);

   SPI_ssel(0);
   r1 = mmc_command(CMD8,0x000001AA);
   for (i = 0; i < 100; i++) {
         if (r1 == 0x01) {
            break;
         }
         r1 = SPI_send (0xFF);
      }
   SPI_ssel (1);
   print_r1(r1);
   for(i = 0; i < 4; i++){
	   r1 = SPI_send (0xFF);
	   print_r1(r1);
   }

   CardType = CARD_NONE;
   /* Check if SD card, send ACMD41 */
   for (i = 0; i < 50000; i++) {
      SPI_ssel (0);
      r1 = mmc_command (APP_CMD, 0);
      SPI_ssel (1);


      if (r1 & 0x04) {
         /* Illegal Command. */
    	 sciSend		(sciREG, 17, (unsigned char *) "Illegal Command.\r");
         break;
      }
      if (r1 == 0x01) {
         SPI_ssel (0);
         //r1 = mmc_command (SD_SEND_OP_COND, 0);
         r1 = mmc_command (SD_SEND_OP_COND, 0x40000000);
         SPI_ssel (1);
         //print_r1(r1);
         if (r1 == 0x00) {
            /* OK, SD card initialized. */
        	sciSend		(sciREG, 25, (unsigned char *) "OK, SD card initialized.\r");
            CardType = CARD_SD;
            break;
         }
      }
   }
   print_r1(r1);
   if (CardType == CARD_NONE) {
      /* Initialize MMC Card, send CMD1. */
      for (i = 0; i < 50000; i++) {
         SPI_ssel (0);
         r1 = mmc_command (SEND_OP_COND, 0);
         SPI_ssel (1);
         //print_r1(r1);

         if (r1 != 0x01) {
            break;
         }
      }
      if (r1 != 0x00) {
         /* Failed to Initialize the card. */
    	  sciSend		(sciREG, 33, (unsigned char *) "Failed to Initialize Flash Card.\r");

         return (0);
      }
      CardType = CARD_MMC;
   }

   /* Set block length in the Flash Card to 512 bytes. */
   SPI_ssel (0);
   r1 = mmc_command (SET_BLOCKLEN, 512);
   SPI_ssel (1);
   if (r1 != 0x00) {
      return (0);
   }
   /* Turn Off CRC option. */
   SPI_ssel (0);
   r1 = mmc_command (CRC_ON_OFF, 0);
   SPI_ssel (1);
   if (r1 != 0x00) {
      return (0);
   }
   /* Success, card initialized. */
   return (1);
}

void SDC_UnInit (void) {

   SPI_ssel (1);
   SPI_hiSpeed (0);
}

/*--------------------------- mmc_command -----------------------------------*/

static uint8_t mmc_command (uint8_t cmd, uint32_t arg) {
   /* Send a Command to Flash card and get a Response. */
   uint32_t r1,i;

   SPI_send (0xFF);
   SPI_send (cmd);
   SPI_send (arg >> 24);
   SPI_send (arg >> 16);
   SPI_send (arg >> 8);
   SPI_send (arg);
   /* Checksum, should only be valid for the first command.CMD0 */
   if(cmd == CMD8){
	   SPI_send (0x87);
   }
   else{
	   SPI_send (0x95);
   }
   
   /* Response will come after 1 - 8 retries. */
   for (i = 0; i < 8; i++) {
      r1 = SPI_send (0xFF);
      if (r1 != 0xFF) {
         break;
      }
   }


   return (r1);
}


/*--------------------------- mmc_read_block --------------------------------*/

static int mmc_read_block (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t cnt) {
   /* Read a 'cnt' of data blocks from Flash Card. */
   uint32_t i;

   if (mmc_command (cmd, arg) != 0x00) {
      /* R1 status error. */
      return (0);
   }

   for (  ; cnt; buf += 512, cnt--) {
      for (i = CMD_TOUT; i; i--) {
         if (SPI_send (0xFF) == 0xFE) {
            /* Data Start token. */
            break;
         }
      }
      if (i == 0) {
         /* Sector Read Tiomeout. */
         return (0);
      }

      for (i = 0; i < 512; i++) {
         buf[i] = SPI_send (0xFF);
      }
      /* Read also a 16-bit CRC. */
      SPI_send (0xFF);
      SPI_send (0xFF);
   }
   return (1);
}


/*--------------------------- mmc_read_bytes --------------------------------*/

static int mmc_read_bytes (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t len) {
   /* Read a 'len' bytes from Flash Card. */
   uint32_t i;

   if (mmc_command (cmd, arg) != 0x00) {
      /* R1 status error. */
      return (0);
   }

   for (i = CMD_TOUT; i; i--) {
      if (SPI_send (0xFF) == 0xFE) {
         /* Data Start token. */
         break;
      }
   }
   if (i == 0) {
      /* Data Read Tiomeout. */
      return (0);
   }

   for (i = 0; i < len; i++) {
      buf[i] = SPI_send (0xFF);
   }
   /* Read also a 16-bit CRC. */
   SPI_send (0xFF);
   SPI_send (0xFF);
   return (1);
}


/*--------------------------- mmc_write_block -------------------------------*/

static int mmc_write_block (uint8_t cmd, uint32_t arg, uint8_t *buf, uint32_t cnt) {
   /* Write a 'cnt' of data blocks to Flash Card. */
   uint32_t i;
   uint8_t  tkn;

   if (mmc_command (cmd, arg) != 0x00) {
      /* R1 status error. */
      return (0);
   }

   /* Select token, for single write or multiple write. */
   tkn = 0xFE;
   if (cnt > 1) {
      tkn = 0xFC;
   }

   for (  ; cnt; buf += 512, cnt--) {
      /* Send Data Start token. */
      SPI_send (tkn);
      /* Send data. */
      for (i = 0; i < 512; i++) {
         SPI_send (buf[i]);
      }
      /* Send also a 16-bit CRC. */
      SPI_send (0xFF);
      SPI_send (0xFF);
      /* Check data response. */
      if ((SPI_send (0xFF) & 0x0F) != 0x05) {
         return (0);
      }
      /* Wait while Flash Card is busy. */
      for (i = WR_TOUT; i; i--) {
         if (SPI_send (0xFF) == 0xFF) {
            /* Sector Write finished. */
            break;
         }
      }
      if (i == 0) {
         /* Sector Write Tiomeout. */
         return (0);
      }
   }
   return (1);
}


/*--------------------------- mmc_read_sect ---------------------------------*/

int mmc_read_sect (uint32_t sect, uint8_t *buf, uint32_t cnt) {
   /* Read single/multiple sectors from Flash Memory Card. */
   uint32_t  i;
   int retv;

   SPI_ssel (0);
   if (cnt > 1) {
      /* Multiple Block Read. */
      retv = mmc_read_block (READ_MULT_BLOCK, sect * 512, buf, cnt);

      mmc_command (STOP_TRAN, 0);
      /* Wait while Flash Card is busy. */
      for (i = STOP_TOUT; i; i--) {
         if (SPI_send (0xFF) == 0xFF) {
            goto x;
         }
      }
      retv = 0;
   }
   else {
      /* Single Block Read. */
      retv = mmc_read_block (READ_BLOCK, sect * 512, buf, 1);
   }
x: SPI_ssel (1);
   return (retv);
}


/*--------------------------- mmc_write_sect --------------------------------*/

int mmc_write_sect (uint32_t sect, uint8_t *buf, uint32_t cnt) {
   /* Write a 512 byte sector to Flash Card. */
   uint32_t  i;
   int retv;

   SPI_ssel (0);
   if (cnt > 1) {
      /* Multiple Block Write. */
      retv = mmc_write_block (WRITE_MULT_BLOCK, sect * 512, buf, cnt);

      mmc_command (STOP_TRAN, 0);
      /* Wait while Flash Card is busy. */
      for (i = STOP_TOUT; i; i--) {
         if (SPI_send (0xFF) == 0xFF) {
            goto x;
         }
      }
      retv = 0;
   }
   else {
      /* Single Block Write. */
      retv = mmc_write_block (WRITE_BLOCK, sect * 512, buf, 1);
   }
x: SPI_ssel (1);
   return (retv);
}


/*--------------------------- mmc_read_config -------------------------------*/

int SDC_rdCfg (SDC_CFG *cfg) {
   /* Read MMC/SD Card device configuration. */
   uint8_t buf[16],*bp;
   int retv;
   uint32_t v,c_size;

   /* Read the CID - Card Identification. */
   SPI_ssel (0);
   retv = mmc_read_bytes (SEND_CID, 0, buf, 16);
   SPI_ssel (1);
   if (retv == 0) {
      /* Read CID failed. */
      return (0);
   }
   /* CID register structure for SD is different than for MMC Card. */
   if (CardType == CARD_SD) {
      bp = &buf[9];
   }
   else {
      bp = &buf[10];
   }
   cfg->sernum = bp[0]<<24 | bp[1]<<16 | bp[2]<<8 | bp[3];

   /* Read the CSD - Card Specific Data. */
   SPI_ssel (0);
   retv = mmc_read_bytes (SEND_CSD, 0, buf, 16);
   SPI_ssel (1);
   if (retv == 0) {
      /* Read CSD failed. */
      return (0);
   }
   /* Read Block length */
   v = buf[5] & 0x0F;
   cfg->read_blen = 1 << v;

   /* Write Block length */
   v = ((buf[12] << 8 | buf[13]) >> 6) & 0x0F;
   cfg->write_blen = 1 << v;

   /* Total Number of blocks */
//   v = ((buf[6] << 16 | buf[7] << 8 | buf[8]) >> 6) & 0x0FFF;
//   m = ((buf[9] << 8  | buf[10]) >> 7) & 0x07;
//   cfg->blocknr = (v + 1) << (m + 2);

   /* Memory Capacity */
   c_size = buf[9] + (buf[8]<<8) +  ((buf[7] & 0x3F) << 16) + 1;
   cfg->capacity = c_size >> 1;

   /* Erase Sector size */
   v = ((buf[10] << 1 | buf[11] >> 7) & 0x7F);
   cfg->sector_size = v + 1;
   return (1);
}

/*------------------------------------------------------------------------------
  Set a SPI clock to low/high speed for SD/MMC
 *------------------------------------------------------------------------------*/
void SPI_hiSpeed (int on) {

  spiREG3->FMT0 &= 0xFFFF00FF;   // mask out baudrate prescaler
  if (on == 1) {
                                 // Max. 5 MBit used for Data Transfer.
    spiREG3->FMT0 |= 7 << 8;     // baudrate prescale 80MHz / (7+1) = 10MBit
  } else {
                                 // Max. 312.5 kBit used in Card Initialization.
    spiREG3->FMT0 |= 255 << 8;   // baudrate prescale 80MHz / (255+1) = 312.5kBit
  }
}

/*------------------------------------------------------------------------------
  Enable/Disable SPI Chip Select
 *------------------------------------------------------------------------------*/
void SPI_ssel (unsigned long ssel) {
  if (ssel) {
      spiREG3->PC3 =    (uint32)((uint32)1U << 0U)  /* SCS[0] */
                      | (uint32)((uint32)1U << 1U)  /* SCS[1] */
                      | (uint32)((uint32)1U << 2U)  /* SCS[2] */
                      | (uint32)((uint32)1U << 3U)  /* SCS[3] */
                      | (uint32)((uint32)1U << 4U)  /* SCS[4] */
                      | (uint32)((uint32)1U << 5U)  /* SCS[5] */
                      | (uint32)((uint32)0U << 8U)  /* ENA */
                      | (uint32)((uint32)0U << 9U)  /* CLK */
                      | (uint32)((uint32)0U << 10U)  /* SIMO */
                      | (uint32)((uint32)0U << 11U); /* SOMI */

  } else {
      spiREG3->PC3 =    (uint32)((uint32)1U << 0U)  /* SCS[0] */
                      | (uint32)((uint32)0U << 1U)  /* SCS[1] */
                      | (uint32)((uint32)1U << 2U)  /* SCS[2] */
                      | (uint32)((uint32)1U << 3U)  /* SCS[3] */
                      | (uint32)((uint32)1U << 4U)  /* SCS[4] */
                      | (uint32)((uint32)1U << 5U)  /* SCS[5] */
                      | (uint32)((uint32)0U << 8U)  /* ENA */
                      | (uint32)((uint32)0U << 9U)  /* CLK */
                      | (uint32)((uint32)0U << 10U)  /* SIMO */
                      | (uint32)((uint32)0U << 11U); /* SOMI */
  }
}


/*------------------------------------------------------------------------------
  Write and Read a byte on SPI interface
 *------------------------------------------------------------------------------*/
unsigned char SPI_send (unsigned char outb) {

	while ((spiREG3->FLG & 0x0200) == 0); // Wait until TXINTFLG is set for previous transmission
	spiREG3->DAT1 = outb | 0x100D0000;    // transmit register address

	while ((spiREG3->FLG & 0x0100) == 0); // Wait until RXINTFLG is set when new value is received
	return((unsigned char)spiREG3->BUF);  // Return received value
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
