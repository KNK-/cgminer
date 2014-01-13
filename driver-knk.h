/*
 * Copyright 2013 Kaloyan Kovachev
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#ifndef BF_KNK_H
#define BF_KNK_H


/* Mining Hardware definitions */
#define KNK_BOARD_CHIPS	14
#define KNK_BANK_BOARDS	2
#define KNK_BANKS		8

#define KNK_BANK_CHIPS	(KNK_BANK_BOARDS*KNK_BOARD_CHIPS)
#define KNK_MAXCHIPS	(KNK_BANKS*KNK_BANK_CHIPS)
#define KNK_MAXBUF		(KNK_MAXCHIPS * 512)

#define KNK_DETECT_RETRIES	5

#define KNK_CHIP_X_CORES	21
#define KNK_CHIP_Y_CORES	36

#define KNK_NOP ((uint8_t *)"\00")
#define KNK_BREAK ((uint8_t *)"\04")
#define KNK_ASYNC ((uint8_t *)"\05")
#define KNK_SYNC ((uint8_t *)"\06")

/* Set clock defaults */
#define KNK_CLK_NO_DIV2	0

#if KNK_CLK_NO_DIV2

#define KNK_BITS_MIN	30
#define KNK_BITS_MAX	48
#define KNK_BITS_DEF	42
#define KNK_BITS_INIT	38

#else

#define KNK_BITS_MIN	52
#define KNK_BITS_MAX	56
#define KNK_BITS_DEF	54
#define KNK_BITS_INIT	52

#endif	// CLK_NO_DIV2

#define KNK_OE_ACTIVE_LOW

#define KNK_SPI_MODE			0
#define KNK_SPI_WORD_BITS		8
#define KNK_SPI_MAX_SPEED		5000000								// Maximum speed the hardware is capable of
#define KNK_SPI_INIT_SPEED		200000
#define KNK_SPI_CHIP_DELAY		5000								// Per chip propagation delay in ns
#define KNK_SPI_CHIP_SPDDEC		(1000000000/(2*KNK_SPI_CHIP_DELAY))	// Per chip decrease of the SPI speed in HZ
#define KNK_SPI_CHIP_SPD(_n)	(KNK_SPI_MAX_SPEED - KNK_SPI_CHIP_SPDDEC * (_n))


/* Mining Host definitions */
// use MPSSE if not on Linux
#ifndef LINUX

#define KNK_MPSSE_SPI

#endif	// LINUX

//#define KNK_MPSSE_SPI	// force MPSSE use

#ifndef KNK_MPSSE_SPI

#define KNK_GPIO_ADDR(_n)			(*((knkinfo->gpio) + (_n)))

#define KNK_PIN_INPUT(_n)			KNK_GPIO_ADDR((_n) / 10) &= (~(7 << (((_n) % 10) * 3)))
#define KNK_PIN_OUTPUT(_n)			KNK_GPIO_ADDR((_n) / 10) |= (1 << (((_n) % 10) * 3))
#define KNK_PIN_ALTERNATIVE(_n, _v)	KNK_GPIO_ADDR((_n) / 10) |= (((_v) <= 3 ? (_v) + 4 : \
													((_v) == 4 ? 3 : 2)) << (((_n) % 10) * 3))

#define KNK_PIN_SET(_n)	KNK_GPIO_ADDR(7) = (1 << _n)
#define KNK_PIN_CLR(_n)	KNK_GPIO_ADDR(10) = (1 << _n)


#ifdef KNK_OE_ACTIVE_LOW

#define KNK_BANK_ON(_n)		KNK_PIN_CLR(_n)
#define KNK_BANK_OFF(_n)	KNK_PIN_SET(_n)

#else

#define KNK_BANK_ON(_n)		KNK_PIN_SET(_n)
#define KNK_BANK_OFF(_n)	KNK_PIN_CLR(_n)

#endif	// KNK_OE_ACTIVE_LOW

#define KNK_LED_ON	KNK_PIN_CLR(18)
#define KNK_LED_OFF	KNK_PIN_SET(18)

#define KNK_DEV_SPI		"/dev/spidev0.0"
#define KNK_DEV_MEM		"/dev/mem"
#define KNK_MEM_LEN		1024
#define KNK_MEM_ADDR	0x20200000

#else	// is KNK_MPSSE_SPI

#ifdef KNK_OE_ACTIVE_LOW

#define KNK_BANK_ON(_n)		PinLow(knkinfo->mpsse_spi, _n)
#define KNK_BANK_OFF(_n)	PinHigh(knkinfo->mpsse_spi, _n)

#else

#define KNK_BANK_ON(_n)		PinHigh(knkinfo->mpsse_spi,_n)
#define KNK_BANK_OFF(_n)	PinLow(knkinfo->mpsse_spi, _n)

#endif	// KNK_OE_ACTIVE_LOW

#define KNK_LED_ON		PinLow(knkinfo->mpsse_spi, GPIOH0)
#define KNK_LED_OFF		PinHigh(knkinfo->mpsse_spi, GPIOH0)

#endif	// KNK_MPSSE_SPI

#define KNK_SPIBUF		4096	// RPi default buffer is 4k. Use for MPSSE too


/* driver definitions */
struct device_drv knk_drv;

struct chips_data {
	uint8_t bank;
	uint8_t bank_pos;
	uint8_t board;
	uint8_t board_pos;

	uint8_t osc_bits;
	uint32_t spi_speed;

	bool reinit;
	bool present;
};

#define KNK_SPI_BUFFERS	2	// 1 for current txrx, 1 for the next and IF we need 1 for overflow
struct knk_info {
	struct thr_info spi_thr;
	struct thr_info res_thr;

	pthread_mutex_t spi_lock;
	pthread_mutex_t res_lock;
	cglock_t blist_lock;

#ifndef KNK_MPSSE_SPI
	volatile unsigned *gpio;
	int spi_fd;
#else
	struct mpsse_context *mpsse_spi;
	bool initialized;
#endif

	struct timeval toggle_led;

	int chips;
	struct chips_data chip_data[KNK_MAXCHIPS];
	struct chips_data *bank_chip_link[KNK_BANKS][KNK_BANK_CHIPS];
	struct chips_data *board_chip_link[KNK_BANKS][KNK_BANK_BOARDS][KNK_BOARD_CHIPS];

	int buffer;
	int buf_status[KNK_SPI_BUFFERS];
	uint8_t buf_write[KNK_SPI_BUFFERS][KNK_MAXBUF];
	uint8_t buf_read[KNK_SPI_BUFFERS][KNK_MAXBUF];
	uint32_t buf_used[KNK_SPI_BUFFERS];
};


/* API definitions */


#endif // BF_KNK_H