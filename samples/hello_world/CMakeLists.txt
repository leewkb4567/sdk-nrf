cmake_minimum_required(VERSION 3.8.2)

if(${BOARD} STREQUAL "nrf52840_pca10090")
	set(CONF_FILE nrf52840_pca10090.conf)
endif()

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(hello_world)

target_sources(app PRIVATE src/main.c)
