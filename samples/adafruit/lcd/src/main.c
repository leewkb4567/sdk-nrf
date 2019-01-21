/*
 * Copyright (c) 2018 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include "ili9341_lcd.h"
#include "touch_ft6206.h"

int main(void)
{
	ili9341_lcd_init();

	ili9341_lcd_on();

	touch_ft6206_init();

	int pressed = 0;
	touch_pos_t touch_pos;

	ili9341_lcd_fill(LCD_COLOR_WHITE);

	while(1) {
		touch_pos = touch_ft6206_get();

		// is touch panel pressed?
		if (touch_pos.z) {
			pressed = 1;
			// yes, draw a dot
			ili9341_lcd_put_dot(touch_pos.x, touch_pos.y, LCD_COLOR_BLACK);
		} else {
			if (pressed) {
				pressed = 0;
				// no, clear screen
				ili9341_lcd_fill(LCD_COLOR_WHITE);
			}
		}
		
		k_sleep(50);
	}
}
