/*
 * Copyright (c) 2015 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef ILI9341_LCD_H__
#define ILI9341_LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

// ILI9341 LCD size is QVGA.
#define ILI9341_LCD_WIDTH     320
#define ILI9341_LCD_HEIGHT    240

/**@brief Function to initiate LCD.
 */
void ili9341_lcd_init(void);

/**@brief Function to turn on LCD.
 */
void ili9341_lcd_on(void);

/**@brief Function to turn off LCD.
 */
void ili9341_lcd_off(void);

/**@brief Function to put a graphics image on LCD.
 */
void ili9341_lcd_put_gfx(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *p_lcd_data);

#ifdef __cplusplus
}
#endif

#endif
