#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Calibration data persisted in flash. Keep it small and word-aligned. */
typedef struct {
    uint32_t magic;
    float    slope;
    float    offset_mm;
    uint32_t cal_count;
    uint32_t crc;
} __attribute__((packed)) CalRecord;

#define FLASH_STORE_MAGIC      0xCA11B4ABU   /* arbitrary identifier */

/* STM32F446RE flash layout (512 KB total, single bank):
 *
 *   Sector 0  0x0800_0000  16 KB
 *   Sector 1  0x0800_4000  16 KB
 *   Sector 2  0x0800_8000  16 KB
 *   Sector 3  0x0800_C000  16 KB
 *   Sector 4  0x0801_0000  64 KB
 *   Sector 5  0x0802_0000 128 KB
 *   Sector 6  0x0804_0000 128 KB
 *   Sector 7  0x0806_0000 128 KB  <-- used for persistent storage
 *
 * Sector 7 is the last sector, keeping it well clear of application code.
 * Update your linker script to cap FLASH LENGTH at 384K (0x60000) so the
 * toolchain cannot place code or read-only data in this sector.
 *
 * Example linker script change:
 *   FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 384K
 */
#define FLASH_STORE_SECTOR     FLASH_SECTOR_7
#define FLASH_STORE_ADDRESS    0x08060000U
#define FLASH_STORE_SECTOR_SIZE (128U * 1024U)   /* 128 KB */

/* The F446 HAL erase API requires a voltage range that controls the
 * parallelism of the erase operation.  FLASH_VOLTAGE_RANGE_3 means the
 * MCU supply is 2.7–3.6 V (true for the Nucleo-64 3.3 V rail) and allows
 * 32-bit wide program/erase operations — the fastest and most reliable
 * choice for this board.  Change to FLASH_VOLTAGE_RANGE_2 (2.1–3.6 V) if
 * your application can run from a lower supply. */
#define FLASH_STORE_VOLTAGE    FLASH_VOLTAGE_RANGE_3

/* Returns 1 if a valid record was loaded into *out, 0 otherwise. */
uint8_t FlashStore_Load(CalRecord *out);

/* Erases the sector and writes the record. Returns 1 on success. */
uint8_t FlashStore_Save(const CalRecord *rec);

#endif /* FLASH_STORE_H */
