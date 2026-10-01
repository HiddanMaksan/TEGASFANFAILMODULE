/**
  ******************************************************************************
  * @file    at24cm01.c
  * @brief   AT24CM01 (1 Mbit / 128 KB) I2C EEPROM driver.
  ******************************************************************************
  * @attention
  *
  * The AT24CM01 is a 1 Mbit serial EEPROM organized as 131,072 x 8 bits.
  * It uses 17-bit word addressing, so a 2-byte (MSB first) address is
  * required before every read/write operation.
  *
  *  I2C device address (7-bit):  1 0 1 0  A2 A1 A0
  *      - A2..A0 are strapped on the package.
  *      - Typical base: 0x50 (A2=A1=A0=0)
  *
  *  Write cycle time (tWR): 5 ms max.
  *
  ******************************************************************************
  */

#include "at24cm01.h"
#include <string.h>

/* ========================================================================== */
/*  Private defines                                                           */
/* ========================================================================== */

/** 7-bit device base address (A2=A1=A0=0). Adjust for your board strap. */
#define AT24CM01_I2C_ADDR_7BIT      0x50u

/** Full 8-bit I2C address for HAL calls (7-bit << 1). */
#define AT24CM01_I2C_ADDR_8BIT      ((uint16_t)(AT24CM01_I2C_ADDR_7BIT << 1))

/** Total memory size: 1 Mbit = 128 KB = 131072 bytes. */
#define AT24CM01_TOTAL_SIZE_BYTES   (128u * 1024u)

/** Page size in bytes. AT24CM01 uses 256-byte pages. */
#define AT24CM01_PAGE_SIZE          256u

/** Maximum write cycle time (tWR) in ms. */
#define AT24CM01_WRITE_CYCLE_MS     5u

/** HAL timeout for EEPROM transactions (ms). */
#define AT24CM01_I2C_TIMEOUT_MS     100u

/* ========================================================================== */
/*  Private helpers                                                           */
/* ========================================================================== */

/**
  * @brief  Build the 2-byte word address into a small buffer (MSB first).
  * @param  mem_addr  17-bit memory address (0 .. 0x1FFFF).
  * @param  out       Buffer of at least 2 bytes.
  */
static void AT24CM01_PackAddress(uint32_t mem_addr, uint8_t *out)
{
    out[0] = (uint8_t)((mem_addr >> 8) & 0xFFu);
    out[1] = (uint8_t)( mem_addr       & 0xFFu);
}

/**
  * @brief  Poll the device until it ACKs (write cycle finished).
  * @param  hi2c  Pointer to I2C handle.
  * @retval HAL_OK if the device ACKed within the timeout, HAL_ERROR otherwise.
  */
HAL_StatusTypeDef AT24CM01_WaitReady(I2C_HandleTypeDef *hi2c)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < AT24CM01_WRITE_CYCLE_MS)
    {
        if (HAL_I2C_IsDeviceReady(hi2c,
                                  AT24CM01_I2C_ADDR_8BIT,
                                  1u,
                                  AT24CM01_WRITE_CYCLE_MS) == HAL_OK)
        {
            return HAL_OK;
        }
    }

    return HAL_ERROR;
}

/* ========================================================================== */
/*  Public API                                                                */
/* ========================================================================== */

/**
  * @brief  Initialise the WP (Write Protect) pin, if present.
  * @note   This is a no-op stub here -- the actual GPIO is initialised in
  *         main.c (MX_GPIO_Init / AT24CM01_WP_Init). Provided so that the
  *         main application can call it from USER CODE BEGIN 2.
  */
void AT24CM01_WP_Init(void)
{
    /* WP is driven by main.c's GPIO init (PB WP_Pin).
     * Nothing to do here -- kept as a hook for future board revisions. */
}

/**
  * @brief  Write a block of bytes to the EEPROM.
  * @param  hi2c      Pointer to I2C handle (e.g. &hi2c1).
  * @param  mem_addr  17-bit start address (0 .. 0x1FFFF).
  * @param  data      Pointer to source buffer.
  * @param  len       Number of bytes to write.
  * @retval HAL_OK on success, error otherwise.
  * @note   The function handles page boundaries automatically: writes are
  *         split so that no single I2C transaction crosses a 256-byte page.
  */
HAL_StatusTypeDef AT24CM01_Write(I2C_HandleTypeDef *hi2c,
                                 uint32_t           mem_addr,
                                 const uint8_t     *data,
                                 uint16_t           len)
{
    if ((hi2c == NULL) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    if ((mem_addr + len) > AT24CM01_TOTAL_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    uint8_t  hdr[2];
    uint16_t written = 0u;

    while (written < len)
    {
        /* Bytes remaining until the next page boundary. */
        uint16_t page_offset = (uint16_t)((mem_addr + written) % AT24CM01_PAGE_SIZE);
        uint16_t chunk       = AT24CM01_PAGE_SIZE - page_offset;

        if (chunk > (len - written))
        {
            chunk = (uint16_t)(len - written);
        }

        /* Build address header. */
        AT24CM01_PackAddress(mem_addr + written, hdr);

        /* Transmit: [addr_hi][addr_lo][data...] in a single I2C transaction. */
        if (HAL_I2C_Master_Transmit(hi2c,
                                    AT24CM01_I2C_ADDR_8BIT,
                                    hdr,
                                    2u,
                                    AT24CM01_I2C_TIMEOUT_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }

        if (HAL_I2C_Master_Transmit(hi2c,
                                    AT24CM01_I2C_ADDR_8BIT,
                                    (uint8_t *)&data[written],
                                    chunk,
                                    AT24CM01_I2C_TIMEOUT_MS) != HAL_OK)
        {
            return HAL_ERROR;
        }

        /* Wait for the internal write cycle to finish. */
        if (AT24CM01_WaitReady(hi2c) != HAL_OK)
        {
            return HAL_ERROR;
        }

        written = (uint16_t)(written + chunk);
    }

    return HAL_OK;
}

/**
  * @brief  Read a block of bytes from the EEPROM.
  * @param  hi2c      Pointer to I2C handle.
  * @param  mem_addr  17-bit start address (0 .. 0x1FFFF).
  * @param  data      Pointer to destination buffer.
  * @param  len       Number of bytes to read.
  * @retval HAL_OK on success, error otherwise.
  */
HAL_StatusTypeDef AT24CM01_Read(I2C_HandleTypeDef *hi2c,
                                uint32_t           mem_addr,
                                uint8_t           *data,
                                uint16_t           len)
{
    if ((hi2c == NULL) || (data == NULL) || (len == 0u))
    {
        return HAL_ERROR;
    }

    if ((mem_addr + len) > AT24CM01_TOTAL_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    uint8_t hdr[2];
    AT24CM01_PackAddress(mem_addr, hdr);

    /* Random read: write 2-byte address, then read len bytes. */
    if (HAL_I2C_Master_Transmit(hi2c,
                                AT24CM01_I2C_ADDR_8BIT,
                                hdr,
                                2u,
                                AT24CM01_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Master_Receive(hi2c,
                               AT24CM01_I2C_ADDR_8BIT,
                               data,
                               len,
                               AT24CM01_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}
