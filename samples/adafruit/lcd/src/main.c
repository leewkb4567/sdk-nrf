/*
 * Copyright (c) 2018 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include "ili9341_lcd.h"

int main(void)
{
	ili9341_lcd_init();

	ili9341_lcd_on();

	ili9341_lcd_cls();

	return 0;
}
