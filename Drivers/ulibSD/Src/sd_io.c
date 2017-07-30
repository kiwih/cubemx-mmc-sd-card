/*
 *  File: sd_io.c
 *  Author: Nelson Lombardo
 *  Year: 2015
 *  e-mail: nelson.lombardo@gmail.com
 *  License at the end of file.
 */

#include "sd_io.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

// For use with uControllers   
/******************************************************************************
 Private Methods Prototypes - Direct work with SD card
******************************************************************************/

/**
    \brief Simple function to calculate power of two.
    \param e Exponent.
    \return Math function result.
*/
uint32_t __SD_Power_Of_Two(uint8_t e);

/**
     \brief Assert the SD card (SPI CS low).
 */
inline void __SD_Assert (void);

/**
    \brief Deassert the SD (SPI CS high).
 */
inline void __SD_Deassert (void);

/**
    \brief Change to max the speed transfer.
    \param throttle
 */
void __SD_Speed_Transfer (uint8_t throttle);

/**
    \brief Send SPI commands.
    \param cmd Command to send.
    \param arg Argument to send.
    \return R1 response.
 */
uint8_t __SD_Send_Cmd(uint8_t cmd, uint32_t arg);

/**
    \brief Write a data block on SD card.
    \param dat Storage the data to transfer.
    \param token Inidicates the type of transfer (single or multiple).
 */
SDRESULTS __SD_Write_Block(SD_DEV *dev, void *dat, uint8_t token);

/**
    \brief Get the total numbers of sectors in SD card.
    \param dev Device descriptor.
    \return Quantity of sectors. Zero if fail.
 */
uint32_t __SD_Sectors (SD_DEV *dev);

void SPI_Freq_High(void) {
  //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  //HAL_SPI_Init(&hspi1);
  hspi1.Instance->I2SPR = 16;
}

void SPI_Freq_Low(void) {
  //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  //HAL_SPI_Init(&hspi1);
  hspi1.Instance->I2SPR = 256;
}

uint32_t spiTimerTickStart;
uint32_t spiTimerTickDelay;

void SPI_Timer_On(uint32_t waitTicks) {
    spiTimerTickStart = HAL_GetTick();
    spiTimerTickDelay = waitTicks;
}

uint8_t SPI_Timer_Status() {
    return ((HAL_GetTick() - spiTimerTickStart) < spiTimerTickDelay);
}

//Beacuse we're using the HAL ticks we don't do anything in this
void SPI_Timer_Off() {
    return;
}

//R/W a single byte on SPI
uint8_t SPI_RW(uint8_t txDat) {
    uint8_t rxDat;
    HAL_SPI_TransmitReceive(&hspi1, &txDat, &rxDat, 1, 50);
    return rxDat;
}

//Flush the SPI by transmitting a bunch of 0xFF
void SPI_Release (void) {
    uint16_t idx;
    for (idx=512; idx && (SPI_RW(0xFF)!=0xFF); idx--);
}

inline void SPI_CS_Low (void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

inline void SPI_CS_High (void){
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

/******************************************************************************
 Private Methods - Direct work with SD card
******************************************************************************/

uint32_t __SD_Power_Of_Two(uint8_t e)
{
    uint32_t partial = 1;
    uint8_t idx;
    for(idx=0; idx!=e; idx++) partial *= 2;
    return(partial);
}

inline void __SD_Assert(void){
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

inline void __SD_Deassert(void){
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

void __SD_Speed_Transfer(uint8_t throttle) {
    if(throttle == HIGH) SPI_Freq_High();
    else SPI_Freq_Low();
}

uint8_t __SD_Send_Cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc, res;
    // ACMD«n» is the command sequense of CMD55-CMD«n»
    if(cmd & 0x80) {
        cmd &= 0x7F;
        res = __SD_Send_Cmd(CMD55, 0);
        if (res > 1) return (res);
    }

    // Select the card
    __SD_Deassert();
    SPI_RW(0xFF);
    __SD_Assert();
    SPI_RW(0xFF);

    // Send complete command set
    SPI_RW(cmd);                        // Start and command index
    SPI_RW((uint8_t)(arg >> 24));          // Arg[31-24]
    SPI_RW((uint8_t)(arg >> 16));          // Arg[23-16]
    SPI_RW((uint8_t)(arg >> 8 ));          // Arg[15-08]
    SPI_RW((uint8_t)(arg >> 0 ));          // Arg[07-00]

    // CRC?
    crc = 0x01;                         // Dummy CRC and stop
    if(cmd == CMD0) crc = 0x95;         // Valid CRC for CMD0(0)
    if(cmd == CMD8) crc = 0x87;         // Valid CRC for CMD8(0x1AA)
    SPI_RW(crc);

    // Receive command response
    // Wait for a valid response in timeout of 5 milliseconds
    SPI_Timer_On(5);
    do {
        res = SPI_RW(0xFF);
    } while((res & 0x80)&&(SPI_Timer_Status()==TRUE));
    SPI_Timer_Off();
    // Return with the response value
    return(res);
}

SDRESULTS __SD_Write_Block(SD_DEV *dev, void *dat, uint8_t token)
{
    uint16_t idx;
    uint8_t line;
    // Send token (single or multiple)
    SPI_RW(token);
    // Single block write?
    if(token != 0xFD)
    {
        // Send block data
        for(idx=0; idx!=SD_BLK_SIZE; idx++) SPI_RW(*((uint8_t*)dat + idx));
        /* Dummy CRC */
        SPI_RW(0xFF);
        SPI_RW(0xFF);
        // If not accepted, returns the reject error
        if((SPI_RW(0xFF) & 0x1F) != 0x05) return(SD_REJECT);
    }
#ifdef SD_IO_WRITE_WAIT_BLOCKER
    // Waits until finish of data programming (blocked)
    while(SPI_RW(0xFF)==0);
    return(SD_OK);
#else
    // Waits until finish of data programming with a timeout
    SPI_Timer_On(SD_IO_WRITE_TIMEOUT_WAIT);
    do {
        line = SPI_RW(0xFF);
    } while((line==0)&&(SPI_Timer_Status()==TRUE));
    SPI_Timer_Off();
#ifdef SD_IO_DBG_COUNT
    dev->debug.write++;
#endif
    if(line==0) return(SD_BUSY);
    else return(SD_OK);
#endif
}

uint32_t __SD_Sectors (SD_DEV *dev)
{
    uint8_t csd[16];
    uint8_t idx;
    uint32_t ss = 0;
    uint16_t C_SIZE = 0;
    uint8_t C_SIZE_MULT = 0;
    uint8_t READ_BL_LEN = 0;
    if(__SD_Send_Cmd(CMD9, 0)==0) 
    {
        // Wait for response
        while (SPI_RW(0xFF) == 0xFF);
        for (idx=0; idx!=16; idx++) csd[idx] = SPI_RW(0xFF);
        // Dummy CRC
        SPI_RW(0xFF);
        SPI_RW(0xFF);
        SPI_Release();
        if(dev->cardtype & SDCT_SD1)
        {
            ss = csd[0];
            // READ_BL_LEN[83:80]: max. read data block length
            READ_BL_LEN = (csd[5] & 0x0F);
            // C_SIZE [73:62]
            C_SIZE = (csd[6] & 0x03);
            C_SIZE <<= 8;
            C_SIZE |= (csd[7]);
            C_SIZE <<= 2;
            C_SIZE |= ((csd[8] >> 6) & 0x03);
            // C_SIZE_MULT [49:47]
            C_SIZE_MULT = (csd[9] & 0x03);
            C_SIZE_MULT <<= 1;
            C_SIZE_MULT |= ((csd[10] >> 7) & 0x01);
        }
        else if(dev->cardtype & SDCT_SD2)
        {
            // C_SIZE [69:48]
            C_SIZE = (csd[7] & 0x3F);
            C_SIZE <<= 8;
            C_SIZE |= (csd[8] & 0xFF);
            C_SIZE <<= 8;
            C_SIZE |= (csd[9] & 0xFF);
            // C_SIZE_MULT [--]. don't exits
            C_SIZE_MULT = 0;
        }
        ss = (C_SIZE + 1);
        ss *= __SD_Power_Of_Two(C_SIZE_MULT + 2);
        ss *= __SD_Power_Of_Two(READ_BL_LEN);
        ss /= SD_BLK_SIZE;
        return (ss);
    } else return (0); // Error
}

/******************************************************************************
 Public Methods - Direct work with SD card
******************************************************************************/

SDRESULTS SD_Init(SD_DEV *dev)
{
    
    uint8_t n, cmd, ct, ocr[4];
    uint8_t idx;
    uint8_t init_trys;
    ct = 0;
    for(init_trys=0; ((init_trys!=SD_INIT_TRYS)&&(!ct)); init_trys++)
    {
        // Initialize SPI for use with the memory card
        //we assume SPI has already been initialised suitably
        //SPI_Init();                                 
        SPI_CS_High();
        SPI_Freq_Low();

        // 80 dummy clocks
        for(idx = 0; idx < 10; idx++) {
            SPI_RW(0xFF);
        }

        SPI_Timer_On(500);
        while(SPI_Timer_Status()==TRUE);
        SPI_Timer_Off();

        dev->mount = FALSE;
        SPI_Timer_On(500);
        while ((__SD_Send_Cmd(CMD0, 0) != 1)&&(SPI_Timer_Status()==TRUE));
        
        SPI_Timer_Off();
        // Idle state
        if (__SD_Send_Cmd(CMD0, 0) == 1) {                      
            // SD version 2?
            if (__SD_Send_Cmd(CMD8, 0x1AA) == 1) {
                // Get trailing return value of R7 resp
                for (n = 0; n < 4; n++) ocr[n] = SPI_RW(0xFF);
                // VDD range of 2.7-3.6V is OK?  
                if ((ocr[2] == 0x01)&&(ocr[3] == 0xAA))
                {
                    // Wait for leaving idle state (ACMD41 with HCS bit)...
                    SPI_Timer_On(1000);
                    while ((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(ACMD41, 1UL << 30)));
                    SPI_Timer_Off(); 
                    // CCS in the OCR?
                    if ((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(CMD58, 0) == 0))
                    {
                        for (n = 0; n < 4; n++) ocr[n] = SPI_RW(0xFF);
                        // SD version 2?
                        ct = (ocr[0] & 0x40) ? SDCT_SD2 | SDCT_BLOCK : SDCT_SD2;
                    }
                }
            } else {
                // SD version 1 or MMC?
                if (__SD_Send_Cmd(ACMD41, 0) <= 1)
                {
                    // SD version 1
                    ct = SDCT_SD1; 
                    cmd = ACMD41;
                } else {
                    // MMC version 3
                    ct = SDCT_MMC; 
                    cmd = CMD1;
                }
                // Wait for leaving idle state
                SPI_Timer_On(250);
                while((SPI_Timer_Status()==TRUE)&&(__SD_Send_Cmd(cmd, 0)));
                SPI_Timer_Off();
                if(SPI_Timer_Status()==FALSE) ct = 0;
                if(__SD_Send_Cmd(CMD59, 0))   ct = 0;   // Deactivate CRC check (default)
                if(__SD_Send_Cmd(CMD16, 512)) ct = 0;   // Set R/W block length to 512 bytes
            }
        }
    }
    if(ct) {
        dev->cardtype = ct;
        dev->mount = TRUE;
        dev->last_sector = __SD_Sectors(dev) - 1;
#ifdef SD_IO_DBG_COUNT
        dev->debug.read = 0;
        dev->debug.write = 0;
#endif
        __SD_Speed_Transfer(HIGH); // High speed transfer
    }
    SPI_Release();
    return (ct ? SD_OK : SD_NOINIT);
}

SDRESULTS SD_Read(SD_DEV *dev, void *dat, uint32_t sector, uint16_t ofs, uint16_t cnt)
{
    SDRESULTS res;
    uint8_t tkn;
    uint16_t remaining;
    res = SD_ERROR;
    if ((sector > dev->last_sector)||(cnt == 0)) return(SD_PARERR);
    // Convert sector number to byte address (sector * SD_BLK_SIZE)
    if (__SD_Send_Cmd(CMD17, sector * SD_BLK_SIZE) == 0) {
        SPI_Timer_On(100);  // Wait for data packet (timeout of 100ms)
        do {
            tkn = SPI_RW(0xFF);
        } while((tkn==0xFF)&&(SPI_Timer_Status()==TRUE));
        SPI_Timer_Off();
        // Token of single block?
        if(tkn==0xFE) { 
            // Size block (512 bytes) + CRC (2 bytes) - offset - bytes to count
            remaining = SD_BLK_SIZE + 2 - ofs - cnt;
            // Skip offset
            if(ofs) { 
                do { 
                    SPI_RW(0xFF); 
                } while(--ofs);
            }
            // I receive the data and I write in user's buffer
            do {
                *(uint8_t*)dat = SPI_RW(0xFF);
                dat++;
            } while(--cnt);
            // Skip remaining
            do { 
                SPI_RW(0xFF); 
            } while (--remaining);
            res = SD_OK;
        }
    }
    SPI_Release();
#ifdef SD_IO_DBG_COUNT
    dev->debug.read++;
#endif
    return(res);
}

#ifdef SD_IO_WRITE
SDRESULTS SD_Write(SD_DEV *dev, void *dat, uint32_t sector)
{
   // uControllers
    // Query ok?
    if(sector > dev->last_sector) return(SD_PARERR);
    // Single block write (token <- 0xFE)
    // Convert sector number to bytes address (sector * SD_BLK_SIZE)
    if(__SD_Send_Cmd(CMD24, sector * SD_BLK_SIZE)==0)
        return(__SD_Write_Block(dev, dat, 0xFE));
    else
        return(SD_ERROR);
}
#endif

SDRESULTS SD_Status(SD_DEV *dev)
{
    return(__SD_Send_Cmd(CMD0, 0) ? SD_OK : SD_NORESPONSE);
}

// «sd_io.c» is part of:
/*----------------------------------------------------------------------------/
/  ulibSD - Library for SD cards semantics            (C)Nelson Lombardo, 2015
/-----------------------------------------------------------------------------/
/ ulibSD library is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/

// Derived from Mister Chan works on FatFs code (http://elm-chan.org/fsw/ff/00index_e.html):
/*----------------------------------------------------------------------------/
/  FatFs - FAT file system module  R0.11                 (C)ChaN, 2015
/-----------------------------------------------------------------------------/
/ FatFs module is a free software that opened under license policy of
/ following conditions.
/
/ Copyright (C) 2015, ChaN, all right reserved.
/
/ 1. Redistributions of source code must retain the above copyright notice,
/    this condition and the following disclaimer.
/
/ This software is provided by the copyright holder and contributors "AS IS"
/ and any warranties related to this software are DISCLAIMED.
/ The copyright owner or contributors be NOT LIABLE for any damages caused
/ by use of this software.
/----------------------------------------------------------------------------*/