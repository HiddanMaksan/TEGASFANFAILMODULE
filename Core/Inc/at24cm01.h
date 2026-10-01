/*
 * at24cm01.h
 *
 * Driver interface for the AT24CM01-XHM-B I2C EEPROM (1 Mbit / 128 KB).
 *
 * Address scheme
 * --------------
 * The AT24CM01 is a 17-bit-address device: it has two 64 KB blocks,
 * selected by the B0 bit (bit 3 of the first address byte). The HAL
 * driver takes the device address already left-shifted, so:
 *
 *     device byte = (0x50 | B0) << 1
 *     B0 = (memAddr >> 16) & 0x01
 *
 * The two bytes that follow the device byte are the low 16 bits of
 * memAddr (MEMADD_SIZE_16BIT, MSB first). Do NOT try to send 24-bit
 * word addresses -- the chip does not accept them.
 *
 * Write-Protect (WP)
 * ------------------
 * WP is wired to a plain GPIO (WP_Pin / WP_GPIO_Port, configured by
 * CubeMX's MX_GPIO_Init). WP HIGH  -> writes blocked.
 *                          WP LOW   -> writes allowed.
 *
 * The AT24CM01_Write() function asserts/deasserts WP itself around each
 * write burst and always re-protects on the way out, so most application
 * code never has to touch WP directly.
 */

#ifndef AT24CM01_H
#define AT24CM01_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Geometry ------------------------------------------------------- */
#define AT24CM01_BASE_ADDR7     0x50u             /* A2=A1=A0=0          */
#define AT24CM01_MEM_SIZE       (128u * 1024u)    /* 1 Mbit = 128 KB     */
#define AT24CM01_PAGE_SIZE      256u              /* bytes per page      */
#define AT24CM01_I2C_TIMEOUT    100u              /* HAL timeout, ms     */
#define AT24CM01_WRITE_MAX_MS   10u               /* datasheet tWR, ms   */

/* --- Write-Protect control ----------------------------------------- */
void AT24CM01_WP_Init(void);
void AT24CM01_WriteProtect_Enable(void);
void AT24CM01_WriteProtect_Disable(void);
bool AT24CM01_WriteProtect_IsEnabled(void);

/* --- Bus / device helpers ------------------------------------------ */
bool              AT24CM01_IsPresent(I2C_HandleTypeDef *hi2c, uint32_t memAddr);
HAL_StatusTypeDef AT24CM01_WaitReady(I2C_HandleTypeDef *hi2c);
/* --- Data access ---------------------------------------------------- */
HAL_StatusTypeDef AT24CM01_Write(I2C_HandleTypeDef *hi2c, uint32_t memAddr,
                                 const uint8_t *data, uint16_t len);
HAL_StatusTypeDef AT24CM01_Read (I2C_HandleTypeDef *hi2c, uint32_t memAddr,
                                 uint8_t *data, uint16_t len);

/* --- Self-test ------------------------------------------------------ */
bool AT24CM01_SelfTest(I2C_HandleTypeDef *hi2c, uint32_t memAddr, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* AT24CM01_H */
