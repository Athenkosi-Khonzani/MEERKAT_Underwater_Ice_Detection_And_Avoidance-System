#include "FlashStore.h"
#include <stddef.h>      /* offsetof */
#include <string.h>      /* memcpy   */

/* --------------------------------------------------------------------------
 * Internal CRC-32 (ISO 3309 / Ethernet polynomial 0xEDB88320, reflected).
 * Replace with HAL_CRC if you have the CRC peripheral free.
 * -------------------------------------------------------------------------- */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc >> 1) ^ (0xEDB88320U & -(crc & 1U));
    }
    return ~crc;
}

/* --------------------------------------------------------------------------
 * FlashStore_Load
 *
 * Reads the CalRecord sitting at FLASH_STORE_ADDRESS, validates the magic
 * number and CRC, and copies it into *out on success.
 *
 * Returns 1 if a valid record was found, 0 otherwise.
 * -------------------------------------------------------------------------- */
uint8_t FlashStore_Load(CalRecord *out)
{
    if (out == NULL) return 0;

    /* Flash is memory-mapped; a plain pointer dereference is sufficient. */
    const CalRecord *stored = (const CalRecord *)FLASH_STORE_ADDRESS;

    if (stored->magic != FLASH_STORE_MAGIC)
        return 0;

    uint32_t expected = crc32((const uint8_t *)stored, offsetof(CalRecord, crc));
    if (stored->crc != expected)
        return 0;

    memcpy(out, stored, sizeof(CalRecord));
    return 1;
}

/* --------------------------------------------------------------------------
 * FlashStore_Save
 *
 * Stamps the magic number, computes the CRC, erases Sector 7 (128 KB on
 * the F446RE), then writes the struct as 32-bit words.
 *
 * Returns 1 on success, 0 on any HAL error.
 * -------------------------------------------------------------------------- */
uint8_t FlashStore_Save(const CalRecord *rec)
{
    if (rec == NULL) return 0;

    /* Build the record locally so we can compute the CRC over a known buffer. */
    CalRecord tmp = *rec;
    tmp.magic = FLASH_STORE_MAGIC;
    tmp.crc   = crc32((const uint8_t *)&tmp, offsetof(CalRecord, crc));

    HAL_FLASH_Unlock();

    /* Clear any leftover error flags from a previous operation. */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGSERR);

    /* Erase Sector 7 (128 KB -- takes ~1-2 seconds on the F446RE). */
    FLASH_EraseInitTypeDef erase = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .Sector       = FLASH_STORE_SECTOR,
        .NbSectors    = 1,
        .VoltageRange = FLASH_STORE_VOLTAGE   /* defined in flash_store.h -- RANGE_3 for 3.3 V Nucleo */
    };
    uint32_t sector_err = 0;
    if (HAL_FLASHEx_Erase(&erase, &sector_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0;
    }

    /* Write the struct as 32-bit words. */
    const uint32_t *src = (const uint32_t *)&tmp;
    uint32_t addr = FLASH_STORE_ADDRESS;
    for (uint32_t i = 0; i < sizeof(CalRecord) / 4; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              addr + i * 4, src[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0;
        }
    }

    HAL_FLASH_Lock();
    return 1;
}
