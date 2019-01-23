#
# Copyright (c) 2019 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
#

zephyr_include_directories(.)

FILE(GLOB lvgl_sources */*.c */*/*.c)
zephyr_sources(${lvgl_sources})
