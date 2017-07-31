#ifndef _USER_DISKIO_SPI_H
#define _USER_DISKIO_SPI_H

#include "integer.h" //from FatFs middleware library
#include "diskio.h" //from FatFs middleware library
#include "ff_gen_drv.h" //from FatFs middleware library

extern inline DSTATUS USER_SPI_initialize (BYTE pdrv);
extern inline DSTATUS USER_SPI_status (BYTE pdrv);
extern inline DRESULT USER_SPI_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  extern inline DRESULT USER_SPI_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);  
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  extern inline DRESULT USER_SPI_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

#endif