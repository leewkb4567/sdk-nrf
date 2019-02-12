/*
 * Copyright (c) 2015 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef TOUCH_FT6206_H__
#define TOUCH_FT6206_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int x;
	int y;
	int z;
} touch_pos_t;

/**@brief Function to initiate touch panel.
 */
void touch_ft6206_init(void);

/**@brief Function to get touch panel position.
 */
touch_pos_t touch_ft6206_get(void);

#ifdef __cplusplus
}
#endif

#endif
