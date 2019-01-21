/*
 * Copyright (c) 2015 - 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <spi.h>
#include "Adafruit_ILI9341.h"
#include "ili9341_lcd.h"

#if defined(CONFIG_BOARD_NRF52840_PCA10056)
#define GPIO_PORT				DT_GPIO_P1_DEV_NAME
#else
#define GPIO_PORT				DT_GPIO_P0_DEV_NAME
#endif
#define SPI_PORT				DT_SPI_0_NAME

// Adafruit LCD pin assignments
#if defined(CONFIG_BOARD_NRF52840_PCA10056)
#define N_RESET     21
#define TFT_CS      6
#define TFT_DC      7
#define TFT_SCK     15
#define TFT_MOSI    13
#define TFT_MISO    14
#elif defined(CONFIG_BOARD_NRF9160_PCA10090)
#define N_RESET     24
#define TFT_CS      10
#define TFT_DC      9
#define TFT_SCK     13
#define TFT_MOSI    11
#define TFT_MISO    12
#else
#define N_RESET     21
#define TFT_CS      16
#define TFT_DC      17
#define TFT_SCK     25
#define TFT_MOSI    23
#define TFT_MISO    24
#endif

// LCD dot size
#define LCD_DOT_W	9
#define LCD_DOT_H	9

//                   //                   //                   //                   //
// m_orientation = 0 // m_orientation = 1 // m_orientation = 2 // m_orientation = 3 //
//                   //                   //                   //                   //
// 0---> X ---> 240  // 320               //               320 // 240 <--- Y <---0  //
// | +----+          // ^ +----+          //         +----+ ^  //         +----+ |  //
// | |    |          // | |    |          //         |    | |  //         |    | |  //
// v |    |          // X |    |          //         |    | Y  //         |    | v  //
// Y |    |          // ^ |    |          //         |    | ^  //         |    | X  //
// | |    |          // | |    |          //         |    | |  //         |    | |  //
// v +----+          // | +----+          //         +----+ |  //         +----+ v  //
// 320               // 0---> Y ---> 240  // 240 <--- X <---0  //               320 //
//                   //                   //                   //                   //

static uint8_t m_orientation = 1;

static struct device		*gpio_port = NULL;
static struct device		*spi_port = NULL;
static struct spi_config	spi_config;

static uint8_t m_tx_data[ILI9341_LCD_WIDTH * 2];
struct spi_buf m_tx_buff[ILI9341_LCD_HEIGHT];

/**@brief Function to wait microseconds.
 */
static void wait_us(uint16_t t)
{
	k_busy_wait(t);
}

/**@brief Function to wait milliseconds.
 */
static void wait_ms(uint16_t t)
{
	k_sleep(t);
}

/**@brief Function to pull down LCD \RESET pin.
 */
static void lcd_nreset_down(void)
{
	gpio_pin_write(gpio_port, N_RESET, 0);
}

/**@brief Function to pull up LCD \RESET pin.
 */
static void lcd_nreset_up(void)
{
	gpio_pin_write(gpio_port, N_RESET, 1);
}

/**@brief Function to pull down LCD CS pin.
 */
static void lcd_cs_down(void)
{
	gpio_pin_write(gpio_port, TFT_CS, 0);
}

/**@brief Function to pull up LCD CS pin.
 */
static void lcd_cs_up(void)
{
	gpio_pin_write(gpio_port, TFT_CS, 1);
}

/**@brief Function to pull down LCD D/C pin.
 */
static void lcd_dc_down(void)
{
	gpio_pin_write(gpio_port, TFT_DC, 0);
}

/**@brief Function to pull up LCD D/C pin.
 */
static void lcd_dc_up(void)
{
	gpio_pin_write(gpio_port, TFT_DC, 1);
}

/**@brief Function to send a LCD command byte by SPI.
 */
static void wr_cmd(uint8_t cmd)
{
	lcd_dc_down();
	lcd_cs_down();
	wait_us(1);

	struct spi_buf_set tx_bufs;
	struct spi_buf tx_buff;
	uint8_t buff = cmd;

	tx_buff.buf = &buff;
	tx_buff.len = 1;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;
	spi_write(spi_port, &spi_config, &tx_bufs);

	lcd_dc_up();
	wait_us(1);
}

/**@brief Function to send data to LCD by SPI.
 */
static void wr_dat(uint8_t *data, uint8_t len)
{
	struct spi_buf_set tx_bufs;
	struct spi_buf tx_buff;

	tx_buff.buf = data;
	tx_buff.len = len;
	tx_bufs.buffers = &tx_buff;
	tx_bufs.count = 1;
	spi_write(spi_port, &spi_config, &tx_bufs);
}

/**@brief Function to get LCD width.
 */
static uint16_t width()
{
	if (m_orientation == 0 || m_orientation == 2)
	{
		return ILI9341_LCD_HEIGHT;
	}
	else
	{
		return ILI9341_LCD_WIDTH;
	}
}

/**@brief Function to get LCD height.
 */
static uint16_t height()
{
	if (m_orientation == 0 || m_orientation == 2)
	{
		return ILI9341_LCD_WIDTH;
	}
	else
	{
		return ILI9341_LCD_HEIGHT;
	}
}

/**@brief Function to set LCD driver window.
 */
static void window (uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint8_t data[4];

	wr_cmd(0x2A);
	data[0] = x >> 8;
	data[1] = x;
	data[2] = (x+w-1) >> 8;
	data[3] = x+w-1;
	wr_dat(data, 4);
	lcd_cs_up();

	wr_cmd(0x2B);
	data[0] = y >> 8;
	data[1] = y;
	data[2] = (y+h-1) >> 8;
	data[3] = y+h-1;
	wr_dat(data, 4);
	lcd_cs_up();
}

/**@brief Function to set LCD driver window to entire area.
 */
static void WindowMax(void)
{
	window(0, 0, width(), height());
}

/**@brief Function to initiate LCD GPIO pins.
 */
static void gpio_init(void)
{
	int err;

	gpio_port = device_get_binding(GPIO_PORT);
	if (gpio_port == NULL) {
		return;
	}

	err = gpio_pin_configure(gpio_port, N_RESET, GPIO_DIR_OUT);
	err += gpio_pin_configure(gpio_port, TFT_CS, GPIO_DIR_OUT);
	err += gpio_pin_configure(gpio_port, TFT_DC, GPIO_DIR_OUT);
	if (!err) {
		err = gpio_port_write(gpio_port, 1 << N_RESET |
						1 << TFT_CS |
						1 << TFT_DC);
	}

	if (err) {
		gpio_port = NULL;
	}
}

/**@brief Function to initiate LCD.
 */
void ili9341_lcd_init(void)
{
	uint8_t spi_data[16];

	gpio_init();

	spi_port = device_get_binding(SPI_PORT);
	if (spi_port == NULL) {
		return;
	}

	spi_config.frequency = 8000000;
	spi_config.operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE;
	spi_config.slave = 0;		/* MOSI & CLK only; CS is not used. */
	spi_config.cs = NULL;

	wait_ms(5);
	lcd_nreset_down();
	wait_us(10);
	lcd_nreset_up();
	wait_ms(5);

	const uint8_t *p_init = Adafruit_ILI9341_initcmd;

	while (*p_init != ILI9341_NOP) {
		wr_cmd(*p_init++);

		uint8_t data_parm = *p_init++;
		uint8_t num_data = data_parm & 0x0F;
		if (num_data > 0) {
			memcpy(spi_data, p_init, num_data);
			p_init += num_data;
			wr_dat(spi_data, num_data);
			lcd_cs_up();
		}
		if (data_parm & 0x40) {
			wait_ms(10);
		}
		if (data_parm & 0x80) {
			wait_ms(120);
		}
	}
}

/**@brief Function to turn on LCD.
 */
void ili9341_lcd_on(void)
{
	wr_cmd(0x29);               // display on
	lcd_cs_up();

	wait_ms(100);
}

/**@brief Function to turn off LCD.
 */
void ili9341_lcd_off(void)
{
	wr_cmd(0x28);               // display off
	lcd_cs_up();
}

static uint16_t ili9341_lcd_get_color(int color8)
{
	uint16_t color = 0;
	
	if (color8 & LCD_COLOR_RED)
		color |= 0x001F;
	if (color8 & LCD_COLOR_GREEN)
		color |= 0x07E0;
	if (color8 & LCD_COLOR_BLUE)
		color |= 0xF800;

	return color;
}

/**@brief Function to fill the entire LCD.
 */
void ili9341_lcd_fill(int color8)
{
	WindowMax();
	
	uint16_t i, j;

	wr_cmd(0x2C);  // send pixel

	uint16_t color = ili9341_lcd_get_color(color8);

	for (i = 0; i < ILI9341_LCD_WIDTH * 2; i += 2)
	{
		m_tx_data[i + 0] = color >> 8;
		m_tx_data[i + 1] = color & 0xFF;
	}

	for (j = 0; j < ILI9341_LCD_HEIGHT; j++)
	{
		m_tx_buff[j].buf = m_tx_data;
		m_tx_buff[j].len = ILI9341_LCD_WIDTH * 2;
	}

	struct spi_buf_set tx_bufs;

	tx_bufs.buffers = m_tx_buff;
	tx_bufs.count = ILI9341_LCD_HEIGHT;
	spi_write(spi_port, &spi_config, &tx_bufs);
	lcd_cs_up();
}

/**@brief Function to put a big dot on LCD.
 */
void ili9341_lcd_put_dot(int x, int y, int color8)
{
	uint16_t lcd_x, lcd_y;
	uint16_t w, w_l, w_r;
	uint16_t h, h_l, h_u;

	if (x >= ILI9341_LCD_WIDTH)
		x = ILI9341_LCD_WIDTH - 1;
	else if (x < 0)
		x = 0;
	if (y >= ILI9341_LCD_HEIGHT)
		y = ILI9341_LCD_HEIGHT - 1;
	else if (y < 0)
		y = 0;

	lcd_x = x;
	lcd_y = y;

	w_l = (lcd_x                 >= LCD_DOT_W / 2)      ? (LCD_DOT_W / 2) : (lcd_x);
	w_r = (lcd_x + LCD_DOT_W / 2 <  ILI9341_LCD_WIDTH)  ? (LCD_DOT_W / 2) : (ILI9341_LCD_WIDTH - 1 - lcd_x);
	h_u = (lcd_y                 >= LCD_DOT_H / 2)      ? (LCD_DOT_H / 2) : (lcd_y);
	h_l = (lcd_y + LCD_DOT_H / 2 <  ILI9341_LCD_HEIGHT) ? (LCD_DOT_H / 2) : (ILI9341_LCD_HEIGHT - 1 - lcd_y);

	lcd_x -= w_l;
	lcd_y -= h_u;

	w = w_l + w_r + 1;
	h = h_l + h_u + 1;

	// set display window
	window(lcd_x, lcd_y, w, h);

	uint16_t i, j;

	wr_cmd(0x2C);  // send pixel

	uint16_t color = ili9341_lcd_get_color(color8);

	for (i = 0; i < w * 2; i += 2)
	{
		m_tx_data[i + 0] = color >> 8;
		m_tx_data[i + 1] = color & 0xFF;
	}

	for (j = 0; j < h; j++)
	{
		m_tx_buff[j].buf = m_tx_data;
		m_tx_buff[j].len = w * 2;
	}

	struct spi_buf_set tx_bufs;

	tx_bufs.buffers = m_tx_buff;
	tx_bufs.count = h;
	spi_write(spi_port, &spi_config, &tx_bufs);
	lcd_cs_up();
}

/**@brief Function to get LCD dot size.
 */
lcd_dot_size_t ili9341_lcd_get_dot_size(void)
{
	lcd_dot_size_t dot_size;

	dot_size.w = LCD_DOT_W;
	dot_size.h = LCD_DOT_H;

	return dot_size;
}
