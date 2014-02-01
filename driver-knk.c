/*
 * Copyright 2013 Andrew Smith
 * Copyright 2013 bitfury
 * Copyright 2013 Anatoly Legkodymov
 * Copyright 2013 Andrew Goodney
 * Copyright 2013/2014 Kaloyan Kovachev
 *
 * BitFury GPIO code based on BAB driver and:
 *
 * bitfury spitest / spic1 code and info:
 *	https://bitcointalk.org/index.php?topic=228677.0
 *
 * BFSB chainminer:
 *	https://github.com/bfsb/chainminer
 * 
 * Anatoly Legkodymov patches:
 *	https://github.com/legkodymov/cgminer 
 * 
 * Andrew Goodney patches for MPSSE support:
 *	https://github.com/agoodney/cgminer
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"
#include "compat.h"
#include "miner.h"
#include "sha2.h"

#include "driver-knk.h"

#ifdef KNK_MPSSE_SPI

#include "usbutils.h"
// FIXME require V1 and set from configure, add libmpsse to compat and check for USE_KNK_MPSSE in usbutils.c instead
#define LIBFTDI1	1
#include <mpsse.h>

#else

#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#endif	// KNK_MPSSE_SPI

/*	forward definitions and static vars */
static void knk_init_chips(struct cgpu_info *knkcgpu, struct knk_info *knkinfo);
static void knk_init(struct cgpu_info *knkcgpu);

static const uint8_t reg_ena[4] = { 0xc1, 0x6a, 0x59, 0xe3 };
static const uint8_t reg_dis[4] = { 0x00, 0x00, 0x00, 0x00 };

#define KNK_BASEA 4
#define KNK_BASEB 61
static const uint8_t knk_counters[16] = {
	64,					64,
	KNK_BASEA,			KNK_BASEA+4,
	KNK_BASEA+2,		KNK_BASEA+2+16,
	KNK_BASEA,			KNK_BASEA+1,
	(KNK_BASEB)%65,		(KNK_BASEB+1)%65,
	(KNK_BASEB+3)%65,	(KNK_BASEB+3+16)%65,
	(KNK_BASEB+4)%65,	(KNK_BASEB+4+4)%65,
	(KNK_BASEB+3+3)%65,	(KNK_BASEB+3+1+3)%65
};

#define KNK_COUNT_ADDR 0x0100
#define KNK_W1A_ADDR 0x1000
#define KNK_W1B_ADDR 0x1400
#define KNK_W2_ADDR 0x1900
#define KNK_INP_ADDR 0x3000
#define KNK_OSC_ADDR 0x6000
#define KNK_REG_ADDR 0x7000

static const uint32_t knk_w1[16] = {
	0,			0,	0,	0xffffffff,
	0x80000000,	0,	0,	0,
	0,			0,	0,	0,
	0,			0,	0,	0x00000280
};

static const uint32_t knk_w2[8] = {
	0x80000000,	0,	0,	0,
	0,		0,	0,	0x00000100
};

static const uint32_t knk_test_work[19] = {
	0xb0e72d8e,	0x1dc5b862,	0xe9e7c4a6,	0x3050f1f5,
	0x8a1a6b7e,	0x7ec384e8,	0x42c1c3fc,	0x8ed158a1,
	0x8a1a6b7e,	0x6f484872,	0x4ff0bb9b,	0x12c97f07,
	0xb0e72d8e,	0x55d979bc,	0x39403296,	0x40f09e84,
	0x8a0bb7b7,	0x33af304f,	0x0b290c1a
	//	nonce	0xf0c4e61f = 0xf144e61f - 0x00800000
};

enum scan_strategy {
	SCAN_BANK,
	SCAN_BOARD,
	SCAN_CHIP,
};

enum scan_strategy scan_strategy = 	SCAN_BANK;
int scan_last_bank = 0;
int scan_last_board = 0;
int scan_last_chip = 0;


#ifdef KNK_MPSSE_SPI

//static const int knk_bank_pins[KNK_BANKS] = {};				// Define which pins are used for OE

//static const int knk_bank_pins[KNK_BANKS] = {GPIOL0,GPIOL1,GPIOL2,GPIOL3};															// 4
//static const int knk_bank_pins[KNK_BANKS] = {GPIOL0,GPIOL1,GPIOL2,GPIOL3,GPIOH7,GPIOH6};												// 6
static const int knk_bank_pins[KNK_BANKS] = {GPIOL0,GPIOL1,GPIOL2,GPIOL3,GPIOH7,GPIOH6,GPIOH5,GPIOH4};									// 8
//static const int knk_bank_pins[KNK_BANKS] = {GPIOL0,GPIOL1,GPIOL2,GPIOL3,GPIOH7,GPIOH6,GPIOH5,GPIOH4,GPIOH3,GPIOH2};					// 10
//			On the MPSSE (FTDI) based board by Goodney the GPIOH0 pin is used for LED and can't be used as OE
//static const int knk_bank_pins[KNK_BANKS] = {GPIOL0,GPIOL1,GPIOL2,GPIOL3,GPIOH7,GPIOH6,GPIOH5,GPIOH4,GPIOH3,GPIOH2,GPIOH1,GPIOH0};	// 12


static bool knk_init_mpsse(struct cgpu_info *knkcgpu, struct knk_info *knkinfo)
{
	struct mpsse_context *mpsse_spi;

	if (knkinfo->initialized) {
		Close(knkinfo->mpsse_spi);
		knkinfo->mpsse_spi = NULL;
	}

	if ( !((mpsse_spi = MPSSE(SPI0, KNK_SPI_MAX_SPEED, MSB)) != NULL && mpsse_spi->open) ) {
		applog(LOG_ERR, "Failed to initialize MPSSE: %s\n", ErrorString(mpsse_spi));
		Close(mpsse_spi);
		return false;
    }

	applog(LOG_WARNING, "%s initialized at %dHz (SPI mode 0)\n", GetDescription(mpsse_spi), GetClock(mpsse_spi));
	// On the MPSSE (FTDI) based board by Goodney the CS pin gates the clock for BitFury reset. This will set the pin low.
	Start(mpsse_spi);
	knkinfo->mpsse_spi = mpsse_spi;
	knkinfo->initialized = true;

	return true;
}

static struct cgpu_info *knk_detect_mpsse(struct libusb_device *dev, struct usb_find_devices *found)
{
	struct cgpu_info *knkcgpu = NULL;
	struct knk_info *knkinfo = NULL;

	knkcgpu = usb_alloc_cgpu(&knk_drv, 1);
	if (unlikely(!knkcgpu)) {
		quithere(1, "Failed to calloc knkcgpu");
	}
	if ( !usb_init(knkcgpu, dev, found) ) {
		knkcgpu = usb_free_cgpu(knkcgpu);
		return NULL;
	}

	knkinfo = calloc(1, sizeof(*knkinfo));
	if (unlikely(!knkinfo)) {
		quithere(1, "Failed to calloc knkinfo");
	}

	applog(LOG_INFO, "%s %d: Found at %s", knkcgpu->drv->name,
				knkcgpu->device_id, knkcgpu->device_path);

	knkinfo->initialized = false;
	knkcgpu->device_data = knkinfo;
	knk_init(knkcgpu);

	return knkcgpu;
}

static void spi_reset(struct knk_info *knkinfo, int chips)
{
	int chip;
	char rst_buf[8] = {0xFF,0x00,0xFF,0x00,0xFF,0x00};	// Toggle MOSI 3 times for each chip

	// Set the MPSSE CS pin high. This will gate the clock, forcing it high for the duration of the reset sequence
    Stop(knkinfo->mpsse_spi);
    for (chip=0; chip<chips; chip++){
        Write(knkinfo->mpsse_spi,rst_buf,6);
    }
    Start(knkinfo->mpsse_spi);	// Return the CS signal low, removing the gating on the clock.
}

static bool spi_txrx(struct knk_info *knkinfo, char *wrbuf, char *rdbuf, int bufsz, __maybe_unused int transfer_speed)
{
    char *MPSSE_readbuf = Transfer(knkinfo->mpsse_spi, wrbuf, bufsz);

    if (MPSSE_readbuf != NULL) {
        // Successful transfer
        memcpy(rdbuf, MPSSE_readbuf, bufsz);
		return true;
    }

	applog(LOG_ERR, "MPSSE Transfer error: %s\n", ErrorString(knkinfo->mpsse_spi));
	return false;
}

#else

#if KNK_BANKS > 1
//static const int knk_bank_pins[KNK_BANKS] = {};				// Define which gpio pins are used for OE

//static const int knk_bank_pins[KNK_BANKS] = {18,23,24,25};	// BFSB master boards V1-V3 ?

//static const int knk_bank_pins[KNK_BANKS] = {22,23,24,25};									// 4
//static const int knk_bank_pins[KNK_BANKS] = {22,23,24,25,8,7};								// 6
static const int knk_bank_pins[KNK_BANKS] = {22,23,24,25,8,7,4,27};								// 8
//static const int knk_bank_pins[KNK_BANKS] = {22,23,24,25,8,7,4,27,28,29,30,31};				// 12
//static const int knk_bank_pins[KNK_BANKS] = {22,23,24,25,8,7,4,27,28,29,30,31,14,15,17,18};	// 16

// All possible RPi P1 pins - GPIO without special functionality both revisions
//static const int knk_bank_pins[KNK_BANKS] = {4,17,18,22,23,24,25};

// All possible RPi P1 pins - GPIO without special functionality Revision 1 header pin 13 is 21
//static const int knk_bank_pins[KNK_BANKS] = {4,17,18,22,23,24,25,21};

// All possible RPi P1 pins - GPIO without special functionality Revision 2 header pin 13 is 27
//static const int knk_bank_pins[KNK_BANKS] = {4,17,18,22,23,24,25,27};

// All possible RPi P5 pins - Revision 2 only
//static const int knk_bank_pins[KNK_BANKS] = {28,29,30,31};
#endif	// KNK_BANKS > 1

static const char *knk_modules[] = {
	"i2c-dev",
	"i2c-bcm2708",
	"spidev",
	"spi-bcm2708",
	NULL
};

static struct {
	int request;
	int value;
} knk_spi_ioc[] = {
	{ SPI_IOC_RD_MODE, KNK_SPI_MODE },
	{ SPI_IOC_WR_MODE, KNK_SPI_MODE },
	{ SPI_IOC_RD_BITS_PER_WORD, KNK_SPI_WORD_BITS },
	{ SPI_IOC_WR_BITS_PER_WORD, KNK_SPI_WORD_BITS },
	{ SPI_IOC_RD_MAX_SPEED_HZ, KNK_SPI_MAX_SPEED },
	{ SPI_IOC_WR_MAX_SPEED_HZ, KNK_SPI_MAX_SPEED },
	{ -1, -1 }
};

static void spi_reset(struct knk_info *knkinfo, int chips)
{
	KNK_PIN_INPUT(9);
	KNK_PIN_INPUT(10);
	KNK_PIN_OUTPUT(10);
	KNK_PIN_INPUT(11);
	KNK_PIN_OUTPUT(11);

	chips *= 3;		// Toggle MOSI 3 times for each chip
	KNK_PIN_SET(11);
	do {
		KNK_PIN_SET(10);
		cgsleep_us(1);	// 1us = 1MHz
		KNK_PIN_CLR(10);
		cgsleep_us(1);
	} while (--chips);
	KNK_PIN_CLR(11);

	KNK_PIN_INPUT(11);
	KNK_PIN_INPUT(10);
	KNK_PIN_ALTERNATIVE(11, 0);
	KNK_PIN_ALTERNATIVE(10, 0);
	KNK_PIN_ALTERNATIVE(9, 0);
}

//	ToDo + toggle LED if it's time - should be done from the caller or via static int8
//	ToDo flash led on bank - api finction call to set bank and api.c to accept the param
static bool spi_txrx(struct knk_info *knkinfo, char *wrbuf, char *rdbuf, int bufsz, __maybe_unused int transfer_speed)
{
	int chunk = bufsz/KNK_SPIBUF;
	struct spi_ioc_transfer tr[chunk+1];

	memset(&tr,0,sizeof(tr));
	while (bufsz >= KNK_SPIBUF) {
		tr[chunk].tx_buf = (uintptr_t) wrbuf;
		tr[chunk].rx_buf = (uintptr_t) rdbuf;
		tr[chunk].len = KNK_SPIBUF;
		tr[chunk].delay_usecs = 1;
		tr[chunk].speed_hz = transfer_speed;
		tr[chunk].bits_per_word = KNK_SPI_WORD_BITS;
		bufsz -= KNK_SPIBUF;
		wrbuf += KNK_SPIBUF;
		rdbuf += KNK_SPIBUF;
		chunk--;
	}
	if (bufsz > 0) {	// The last chunk is 0
		tr[0].tx_buf = (uintptr_t) wrbuf;
		tr[0].rx_buf = (uintptr_t) rdbuf;
		tr[0].len = (unsigned)bufsz;
		tr[0].delay_usecs = 1;
		tr[0].speed_hz = transfer_speed;
		tr[0].bits_per_word = KNK_SPI_WORD_BITS;
	}

	for (chunk=bufsz/KNK_SPIBUF; chunk >= 0; chunk--) {
		if ( (int)ioctl(knkinfo->spi_fd, SPI_IOC_MESSAGE(1), (intptr_t)&tr[chunk]) < 0 ) {
			applog(LOG_ERR, "Unable to SPI_IOC_MESSAGE");
			return false; 
		}
	}

	return true;
}

static bool knk_init_gpio(struct cgpu_info *knkcgpu, struct knk_info *knkinfo)
{
	int i, err, memfd, data;
	char buf[64];

	for (i = 0; knk_modules[i]; i++) {
		snprintf(buf, sizeof(buf), "modprobe %s", knk_modules[i]);
		err = system(buf);
		if (err) {
			applog(LOG_ERR, "%s failed to modprobe %s (%d) - you need to be root?",
					knkcgpu->drv->dname,
					knk_modules[i], err);
			goto bad_out;
		}
	}

	memfd = open(KNK_DEV_MEM, O_RDWR | O_SYNC);
	if (memfd < 0) {
		applog(LOG_ERR, "%s failed open %s (%d)",
				knkcgpu->drv->dname,
				KNK_DEV_MEM, errno);
		goto bad_out;
	}

	knkinfo->gpio = (volatile unsigned *)mmap(NULL, KNK_MEM_LEN,
						  PROT_READ | PROT_WRITE,
						  MAP_SHARED, memfd, KNK_MEM_ADDR);

	close(memfd);

	if (knkinfo->gpio == MAP_FAILED) {
		applog(LOG_ERR, "%s failed mmap gpio (%d)",
				knkcgpu->drv->dname, errno);
		goto bad_out;
	}

	knkinfo->spi_fd = open(KNK_DEV_SPI, O_RDWR);
	if (knkinfo->spi_fd < 0) {
		applog(LOG_ERR, "%s failed to open spidev (%d)",
				knkcgpu->drv->dname, errno);
		goto map_out;
	}

	for (i = 0; knk_spi_ioc[i].value != -1; i++) {
		data = knk_spi_ioc[i].value;
		err = ioctl(knkinfo->spi_fd, knk_spi_ioc[i].request, (void *)&data);
		if (err < 0) {
			applog(LOG_ERR, "%s failed ioctl (%d) (%d)",
					knkcgpu->drv->dname, i, errno);
			goto close_out;
		}
	}

	knkinfo->initialized = true;
	return true;

close_out:
	close(knkinfo->spi_fd);
	knkinfo->spi_fd = 0;

map_out:
	munmap((void *)(knkinfo->gpio), KNK_MEM_LEN);
	knkinfo->gpio = NULL;

bad_out:
	return false;
}

static void knk_detect_spidev(void)
{
	struct cgpu_info *knkcgpu = NULL;
	struct knk_info *knkinfo = NULL;

	knkcgpu = calloc(1, sizeof(*knkcgpu));
	if (unlikely(!knkcgpu)) {
		quithere(1, "Failed to calloc knkcgpu");
	}

	knkinfo = calloc(1, sizeof(*knkinfo));
	if (unlikely(!knkinfo)) {
		quithere(1, "Failed to calloc knkinfo");
	}

	knkcgpu->drv = &knk_drv;
	knkcgpu->deven = DEV_ENABLED;
	knkcgpu->threads = 1;
	knkcgpu->device_data = (void *)knkinfo;
	knkinfo->initialized = false;

	knk_init(knkcgpu);
}

#endif	// KNK_MPSSE_SPI

static void knk_detect(__maybe_unused bool hotplug)
{

#ifdef KNK_MPSSE_SPI
	usb_detect(&knk_drv, knk_detect_mpsse);
#else
	if (hotplug)
#endif
		return;

	knk_detect_spidev();
}

static void knk_init(struct cgpu_info *knkcgpu)
{
	struct knk_info *knkinfo = knkcgpu->device_data;

	mutex_init(&knkinfo->spi_lock);
#ifdef KNK_MPSSE_SPI
	if ( !knk_init_mpsse(knkcgpu, knkinfo) ) {
#else
	if ( !knk_init_gpio(knkcgpu, knkinfo) ) {
#endif
		goto unalloc;
	}

	mutex_init(&knkinfo->res_lock);
	cglock_init(&knkinfo->blist_lock);

	cgtime(&knkinfo->toggle_led);
	knk_init_chips(knkcgpu, knkinfo);
	if (!knkinfo->chips || !add_cgpu(knkcgpu)) {
		goto cleanup;
	}

	return;

cleanup:
#ifndef KNK_MPSSE_SPI
	close(knkinfo->spi_fd);
	munmap((void *)(knkinfo->gpio), KNK_MEM_LEN);
#endif
	cglock_destroy(&knkinfo->blist_lock);
	mutex_destroy(&knkinfo->res_lock);

unalloc:
	mutex_destroy(&knkinfo->spi_lock);
	free(knkinfo);
#ifdef KNK_MPSSE_SPI
	usb_uninit(knkcgpu);
	knkcgpu = usb_free_cgpu(knkcgpu);
#else
	free(knkcgpu);
#endif
}

static uint32_t decnonce(uint32_t in)
{
	uint32_t out;

	/* First part load */
	out = (in & 0xFF) << 24;
	in >>= 8;

	/* Byte reversal */
	in = (((in & 0xaaaaaaaa) >> 1) | ((in & 0x55555555) << 1));
	in = (((in & 0xcccccccc) >> 2) | ((in & 0x33333333) << 2));
	in = (((in & 0xf0f0f0f0) >> 4) | ((in & 0x0f0f0f0f) << 4));

	out |= (in >> 2) & 0x3FFFFF;

	/* Extraction */
	if (in & 1) {
		out |= (1 << 23);
	}
	if (in & 2) {
		out |= (1 << 22);
	}

	out -= 0x800004;
	return out;
}

static void ms3steps(uint32_t *p)
{
	uint32_t a, b, c, d, e, f, g, h, new_e, new_a;
	int i;

	a = p[0];
	b = p[1];
	c = p[2];
	d = p[3];
	e = p[4];
	f = p[5];
	g = p[6];
	h = p[7];
	for (i = 0; i < 3; i++) {
		new_e = p[i+16] + sha256_k[i] + h + CH(e,f,g) + SHA256_F2(e) + d;
		new_a = p[i+16] + sha256_k[i] + h + CH(e,f,g) + SHA256_F2(e) +
			SHA256_F1(a) + MAJ(a,b,c);
		d = c;
		c = b;
		b = a;
		a = new_a;
		h = g;
		g = f;
		f = e;
		e = new_e;
	}
	p[15] = a;
	p[14] = b;
	p[13] = c;
	p[12] = d;
	p[11] = e;
	p[10] = f;
	p[9] = g;
	p[8] = h;
}

static void spi_add_buf_reverse(char *spibuf, int *spibufsz, const char *str, unsigned sz)
{
	unsigned i;

	if (*spibufsz + sz >= KNK_MAXBUF) {
		return;
	}

	for (i = 0; i < sz; i++) {	// Reverse bit order in each byte!
		unsigned char p = str[i];
		p = ((p & 0xaa)>>1) | ((p & 0x55) << 1);
		p = ((p & 0xcc)>>2) | ((p & 0x33) << 2);
		p = ((p & 0xf0)>>4) | ((p & 0x0f) << 4);
		spibuf[*spibufsz+i] = p;
	}
	*spibufsz += sz;
}

static void spi_add_buf(char *spibuf, int *spibufsz, const char *str, unsigned sz)
{
	if (*spibufsz + sz >= KNK_MAXBUF) {
		return;
	}

	memcpy(&spibuf[*spibufsz], str, sz);
	*spibufsz += sz;
}

static void spi_buffer_data(char *spibuf, int *spibufsz, unsigned addr, const char *buf, unsigned len)
{
	unsigned char otmp[3];

	if (len < 4 || len > 128) {
		return; /* This cannot be programmed in single frame! */
	}

	len /= 4; /* Strip */
	otmp[0] = (len - 1) | 0xE0;
	otmp[1] = (addr >> 8)&0xFF; otmp[2] = addr & 0xFF;

	spi_add_buf(spibuf, spibufsz, otmp, 3);
	spi_add_buf_reverse(spibuf, spibufsz, buf, len * 4);
}

/*
	Configuration registers - control oscillators and such stuff.
	PROGRAMMED when magic number is matches, UNPROGRAMMED (default) otherwise
*/
static void config_reg(char *spibuf, int *spibufsz, int cfgreg, int ena)
{
	spi_buffer_data(spibuf, spibufsz, KNK_REG_ADDR + cfgreg * 32, ((ena) ? (void*)reg_ena : (void*)reg_dis), 4);
}

static void set_freq(char *spibuf, int *spibufsz, uint8_t bits)
{
	uint64_t freq;
	unsigned char *osc;

	osc = (unsigned char *)&freq;
	freq = (1ULL << bits) - 1ULL;

	/* Program internal on-die slow oscillator frequency */
	spi_buffer_data(spibuf, spibufsz, KNK_OSC_ADDR, (void*)osc, 8);
}

static void send_conf(char *spibuf, int *spibufsz)
{
	int reg;

	for ( reg=7; reg<=11; reg++ ) {
		config_reg(spibuf, spibufsz, reg, 0);
	}

	config_reg(spibuf, spibufsz, 1, 0);					/* Use INCLK */
	config_reg(spibuf, spibufsz, 6, 0);					/* Disable OUTCLK */
//	config_reg(spibuf, spibufsz, 6, 1);					/* Enable OUTCLK */
	config_reg(spibuf, spibufsz, 3, KNK_CLK_NO_DIV2);	/* Divide or not by 2 the clock */
	config_reg(spibuf, spibufsz, 2, 0);					/* Disable fast oscillator */
	config_reg(spibuf, spibufsz, 4, 1);					/* Enable slow oscillator */
}

static void send_bufs_init(char *spibuf, int *spibufsz)
{
	/* Program counters correctly for rounds processing */
	spi_buffer_data(spibuf, spibufsz, KNK_COUNT_ADDR, knk_counters, sizeof(knk_counters));
	spi_buffer_data(spibuf, spibufsz, KNK_W1A_ADDR, (const char *)knk_w1, sizeof(knk_w1) * 4);
	spi_buffer_data(spibuf, spibufsz, KNK_W1B_ADDR, (const char *)knk_w1, sizeof(knk_w1) * 4 / 2);
	spi_buffer_data(spibuf, spibufsz, KNK_W2_ADDR, (const char *)knk_w2, sizeof(knk_w2) * 4);
}

static void send_full_init(char *spibuf, int *spibufsz)
{
	set_freq(spibuf, spibufsz, KNK_BITS_INIT);
	send_conf(spibuf, spibufsz);
	send_bufs_init(spibuf, spibufsz);
}

static void process_newbuf(uint32_t *oldbuf, uint32_t *newbuf, struct chips_data *chip_info)
{
	int8_t pos = chip_info->old_counter_pos;
	uint32_t counter = chip_info->old_counter_val;

	if ( !memcmp(oldbuf, newbuf, 16*4) || (newbuf[16] != 0xFFFFFFFF && newbuf[16] != 0x00000000) ) {
		return;
	}

	if ( pos < 0) {
		pos = 0;
	}

	for (; pos < 16; pos++) {
		if (oldbuf[pos] != newbuf[pos]) {
			counter = decnonce(newbuf[pos]);

			if (chip_info->old_counter_pos < 0 && newbuf[((!pos) ? 15 : pos)] != 0xF144E61F) {
				continue;
			}

			if ((counter & 0xFFC00000) == 0xDF800000) {
				chip_info->new_counter_val = counter - 0xDF800000;
				chip_info->new_counter_pos = pos;
				return;
			}

			applog(LOG_INFO, "new nonce %d = %x / %x / %x", pos, counter, counter - 0x00800000, counter - 0x00400000);
		}
	}

	for (pos=0; chip_info->old_counter_pos > 0 && pos < chip_info->old_counter_pos; pos++) {
		if (oldbuf[pos] != newbuf[pos]) {
			counter = decnonce(newbuf[pos]);

			if (chip_info->old_counter_pos < 0 && newbuf[((!pos) ? 15 : pos)] != 0xF144E61F) {
				continue;
			}

			if ((counter & 0xFFC00000) == 0xDF800000) {
				chip_info->new_counter_val = counter - 0xDF800000;
				chip_info->new_counter_pos = pos;
				return;
			}

			applog(LOG_INFO, "new nonce %d = %x / %x / %x", pos, counter, counter - 0x00800000, counter - 0x00400000);
		}
	}
}

static void detect_chips(struct knk_info *knkinfo, uint8_t bank) {
	uint8_t chip;
	int tries = KNK_DETECT_RETRIES;
	uint32_t newbuf[KNK_BANK_CHIPS][17], oldbuf[KNK_BANK_CHIPS][17];
	char wrbuf[KNK_SPIBUF], rdbuf[KNK_SPIBUF];
	int spibufsz = 0;
	int chips_found = 0;

	memset(newbuf, 0, sizeof(newbuf));
	memset(oldbuf, 0, sizeof(oldbuf));

	mutex_lock(&knkinfo->spi_lock);
	do {
		spi_reset(knkinfo, KNK_BANK_CHIPS+1);
		spi_add_buf(wrbuf, &spibufsz, KNK_BREAK,1);
		for (chip = 0; chip < KNK_BANK_CHIPS; chip++) {
			if ( !knkinfo->chip_data[knkinfo->chips].present ) {
				send_full_init(wrbuf, &spibufsz);
				spi_txrx(knkinfo, wrbuf, rdbuf, spibufsz, KNK_SPI_INIT_SPEED);
				spibufsz = 0;

//				spi_add_buf(wrbuf, &spibufsz, KNK_NOP,1);	// Padding to 4 bytes not necessary
				spi_buffer_data(wrbuf, &spibufsz, KNK_INP_ADDR, (void*)&knk_test_work, sizeof(knk_test_work));
				spi_txrx(knkinfo, wrbuf, rdbuf, spibufsz, KNK_SPI_INIT_SPEED);

				memcpy(oldbuf[chip], newbuf[chip], sizeof(newbuf[chip]));
				memcpy(newbuf[chip], rdbuf + spibufsz - sizeof(knk_test_work), sizeof(newbuf[chip]));
				spibufsz = 0;

				if (tries == KNK_DETECT_RETRIES) {
					memcpy(oldbuf[chip], newbuf[chip], sizeof(newbuf[chip]));
				} else if ( memcmp(oldbuf[chip], newbuf[chip], 16*4) &&		// ToDo just check for job switch for now
							(newbuf[chip][16] == 0xFFFFFFFF || 
							(newbuf[chip][16] == 0x00000000 && oldbuf[chip][16] != 0xFFFFFFFF)) ) {
					uint8_t board = chip / KNK_BOARD_CHIPS;
					uint8_t board_pos = chip % KNK_BOARD_CHIPS;

					knkinfo->chip_data[knkinfo->chips].bank_pos = chip;
					knkinfo->chip_data[knkinfo->chips].bank = bank;
					knkinfo->chip_data[knkinfo->chips].board = board;
					knkinfo->chip_data[knkinfo->chips].board_pos = board_pos;
					knkinfo->chip_data[knkinfo->chips].present = true;
					knkinfo->chip_data[knkinfo->chips].reinit = false;
					knkinfo->chip_data[knkinfo->chips].spi_speed = KNK_SPI_CHIP_SPD(chip);
					mutex_init(&knkinfo->chip_data[knkinfo->chips].buf_lock);

					knkinfo->bank_chip_link[bank][chip] = &knkinfo->chip_data[knkinfo->chips];
					knkinfo->board_chip_link[bank][board][board_pos] = &knkinfo->chip_data[knkinfo->chips];

					applog(LOG_WARNING, "Chip detected: slot %d, board %d, position %d - chip #%d (SPI speed %d)",
							bank, board, board_pos, knkinfo->chips, knkinfo->chip_data[knkinfo->chips].spi_speed);

					knkinfo->chips++;
					chips_found++;
				}
			}
			spi_add_buf(wrbuf, &spibufsz, KNK_ASYNC,1);
		}
		cgsleep_ms(1500/KNK_DETECT_RETRIES);
		tries--;
	} while (tries && chips_found != KNK_BANK_CHIPS);
	mutex_unlock(&knkinfo->spi_lock);
}

static void knk_init_chips(struct cgpu_info *knkcgpu, struct knk_info *knkinfo)
{
	uint8_t chip, bank = 0;

	applog(LOG_WARNING, "%s testing for %d chips ...", knkcgpu->drv->dname, KNK_MAXCHIPS);

	knkinfo->chips = 0;
	for (chip = 0; chip < KNK_MAXCHIPS; chip++) {
		memset(&knkinfo->chip_data[chip], 0, sizeof(struct chips_data));

		knkinfo->chip_data[chip].spi_speed = KNK_SPI_INIT_SPEED;
		knkinfo->chip_data[chip].osc_bits = KNK_BITS_INIT;
		knkinfo->chip_data[chip].reinit = true;
		knkinfo->chip_data[chip].present = false;
	}

#if KNK_BANKS > 1
	for (bank = 0; bank < KNK_BANKS; bank++) {	// First turn OFF all banks
#ifndef KNK_MPSSE_SPI
		KNK_PIN_INPUT(knk_bank_pins[bank]);
		KNK_PIN_OUTPUT(knk_bank_pins[bank]);
#endif	// KNK_MPSSE_SPI
		KNK_BANK_OFF(knk_bank_pins[bank]);
	}

	for (bank = 0; bank < KNK_BANKS; bank++) {
		KNK_BANK_ON(knk_bank_pins[bank]);
#endif	// KNK_BANKS > 1
		detect_chips(knkinfo, bank);
#if KNK_BANKS > 1
		KNK_BANK_OFF(knk_bank_pins[bank]);
	}
#endif

	applog(LOG_WARNING, "%s found %d chips", knkcgpu->drv->dname, knkinfo->chips);
}

static void knk_identify(__maybe_unused struct cgpu_info *knkcgpu)
{
	struct knk_info *knkinfo = knkcgpu->device_data;

	cgtime(&knkinfo->toggle_led);
	knkinfo->toggle_led.tv_sec += 5;	// Keep the LED ON for 5 sec
	KNK_LED_ON;
}

static struct api_data *knk_get_api_stats(struct cgpu_info *knkcgpu)
{
	struct knk_info *knkinfo = knkcgpu->device_data;
	struct api_data *root = NULL;
	int chip;

	root = api_add_int(root, "Chips", &(knkinfo->chips), true);

	for (chip = 0; chip < knkinfo->chips; chip++) {
		struct chips_data chip_info = knkinfo->chip_data[chip];
		char buf[32];

		if ( !chip_info.present ) {
			continue;
		}

		sprintf(buf, "clock_bits_%d_%d_%d", chip_info.bank, chip_info.board, chip_info.board_pos);
		root = api_add_uint8(root, buf, &(chip_info.osc_bits), false);

//	ToDo add others
//		root = api_add_temp(root, "Temp", &(knkinfo->temp), true);
	}

	return root;
}

static void fix_nonce(uint32_t got_nonce, struct chips_data chip_info, struct thr_info res_thr)
{
	uint32_t nonce = dec_nonce(got_nonce);
	uint32_t coor;
	int x, y;
	struct work *work;

	if ( (got_nonce & 0xFF) < 0x1C) {
		nonce -= 0x400000;
		coor = ( ((nonce >> 29) & 0x07) | ((nonce >> 19) & 0x3F8) );
		x = coor % 24;
		y = coor / 24;
		if ( y < 36 ) {
			// 3 out of 24 cases

// ToDo the nonce is OK - check if for the current or old work
//			if ( chip_info.current_nonce_high ^ chip_info.current_job_high && test_nonce(new_work, nonce) ) {
//				chip_info.current_nonce_high ^= 1;
//			}

			if ( submit_nonce(&res_thr, work, nonce) ) {
			// Store good or bad in chip info per core
				chip_info.ok_nonces++;
				chip_info.ok_core_nonces[x][y]++;
				return;
			} else {
				chip_info.hw_core_nonces[x][y]++;
			}
		}
	} else {
		coor = ( ((nonce >> 29) & 0x07) | ((nonce >> 19) & 0x3F8) );
		x = coor % 24;
		y = coor / 24;
		if ( x >= 17 && y < 36 ) {
			// 7 out of 24 cases

// ToDo the nonce is OK - check if for the current or old work

			if ( submit_nonce(&res_thr, work, nonce) ) {
			// Store good or bad in chip info per core
				chip_info.ok_nonces++;
				chip_info.ok_core_nonces[x][y]++;
				return;
			} else {
				chip_info.hw_core_nonces[x][y]++;
			}
		}

		nonce -= 0x800000;
		coor = ( ((nonce >> 29) & 0x07) | ((nonce >> 19) & 0x3F8) );
		x = coor % 24;
		y = coor / 24;
		if ( y < 36 && ((x >= 1 && x <= 4) || (x >= 9 && x <= 15)) ) {
			// 11 out of 24 cases

// ToDo the nonce is OK - check if for the current or old work

			if ( submit_nonce(&res_thr, work, nonce) ) {
			// Store good or bad in chip info per core
				chip_info.ok_nonces++;
				chip_info.ok_core_nonces[x][y]++;
				return;
			} else {
				chip_info.hw_core_nonces[x][y]++;
			}
		}
	}

	// Bad core or invalid nonce
	chip_info.hw_nonces++;
	inc_hw_errors(&res_thr);
	return;
}

// Thread to handle SPI communication with the chips
static void *knk_spi_thread(void *userdata)
{
	struct cgpu_info *knkcgpu = (struct cgpu_info *)userdata;
	struct knk_info *knkinfo = knkcgpu->device_data;
	struct timeval start_time, stop_time;

	// Wait until we're ready
	while (!knkcgpu->shutdown || !knkinfo->initialized) {
		cgsleep_ms(1000);
	}

	while (!knkcgpu->shutdown) {
		mutex_lock(&knkinfo->spi_lock);
		cgtime(&start_time);

	// ToDo (bank/board/chip) strategy based SPI communication
	// Use buf_lock for each chip

		cgtime(&stop_time);
		mutex_unlock(&knkinfo->spi_lock);
		if ( ms_tdiff(&start_time, &stop_time) < 1) {
			cgsleep_ms(1);
		}
	}
}

// Results checking thread
static void *knk_res_thread(void *userdata)
{
	struct cgpu_info *knkcgpu = (struct cgpu_info *)userdata;
	struct knk_info *knkinfo = knkcgpu->device_data;

	// ToDo walktrough the banks, boards, chips and process the buffers
	// Use buf_lock for each chip
}

static bool knk_thread_prepare(struct thr_info *thr)
{
	struct cgpu_info *knkcgpu = thr->cgpu;
	struct knk_info *knkinfo = knkcgpu->device_data;

	if (thr_info_create(&(knkinfo->spi_thr), NULL, knk_spi_thread, (void *)knkcgpu)) {
		applog(LOG_ERR, "%s%i: SPI thread create failed", knkcgpu->drv->name, knkcgpu->device_id);
		return false;
	}
	pthread_detach(knkinfo->spi_thr.pth);

	/*
	 * We require a separate results checking thread since there is a lot
	 * of work done checking the results multiple times - thus we don't
	 * want that delay affecting sending/receiving work to/from the device
	 */
	if (thr_info_create(&(knkinfo->res_thr), NULL, knk_res_thread, (void *)knkcgpu)) {
		applog(LOG_ERR, "%s%i: Results thread create failed", knkcgpu->drv->name, knkcgpu->device_id);
		return false;
	}
	pthread_detach(knkinfo->res_thr.pth);

	return true;
}

static void knk_shutdown(struct thr_info *thr)
{
	struct cgpu_info *knkcgpu = thr->cgpu;
	struct knk_info *knkinfo = knkcgpu->device_data;
	char wrbuf[KNK_SPIBUF], rdbuf[KNK_SPIBUF];
	int bank, chip, spibufsz = 0;

	applog(LOG_DEBUG, "%s%i: shutting down", knkcgpu->drv->name, knkcgpu->device_id);
	mutex_lock(&knkinfo->spi_lock);

#if KNK_BANKS > 1
	for (bank = 0; bank < KNK_BANKS; bank++) {
		KNK_BANK_ON(knk_bank_pins[bank]);
#endif	// KNK_BANKS > 1
		spi_reset(knkinfo, KNK_BANK_CHIPS+1);
		spi_add_buf(wrbuf, &spibufsz, KNK_BREAK,1);
		for (chip = 0; chip < KNK_BANK_CHIPS; chip++) {
			config_reg(wrbuf, &spibufsz, 4, 0);		/* Disable slow oscillator which stops the chip */
			spi_add_buf(wrbuf, &spibufsz, KNK_ASYNC,1);
		}
		spi_txrx(knkinfo, wrbuf, rdbuf, spibufsz, KNK_SPI_INIT_SPEED);
#if KNK_BANKS > 1
		KNK_BANK_OFF(knk_bank_pins[bank]);
	}
#endif

	for (chip = 0; chip < KNK_MAXCHIPS; chip++) {
		if (knkinfo->chip_data[chip].present) {
			mutex_destroy(&knkinfo->chip_data[chip].buf_lock);
			memset(&knkinfo->chip_data[chip], 0, sizeof(struct chips_data));
		}
	}

	knkcgpu->shutdown = true;
	knkinfo->initialized = false;
	mutex_unlock(&knkinfo->spi_lock);
}

#if ToDo
static BLIST *store_work(struct cgpu_info *knkcgpu, struct work *work)
{
	// ToDo
	return NULL;
}
#endif

static bool knk_queue_full(struct cgpu_info *knkcgpu)
{
	struct knk_info *knkinfo = (knkcgpu->device_data);
	struct work *work;

	if (knkinfo->work_queue_count >= knkinfo->chips) {
		return true;
	}

	if ( (work = get_queued(knkcgpu)) ) {
//		store_work(knkcgpu, work);	// ToDo
		return true;
	}

	// Avoid a hard loop when we can't get work fast enough
	cgsleep_ms(10);
	return false;
}

static int64_t knk_scanwork(__maybe_unused struct thr_info *thr)
{
	struct cgpu_info *knkcgpu = thr->cgpu;
	struct knk_info *knkinfo = knkcgpu->device_data;
	int64_t hashcount = 0;

	// ToDo we don't want to do SPI here, what should trigger a return value?
	// Do we just send back nonces from completed works?

	return hashcount;
}

static void knk_flush_work(struct cgpu_info *knkcgpu)
{
	struct knk_info *knkinfo = knkcgpu->device_data;

	// ToDo free all works not sent to chips yet and mark the rest as 'stale'
}

struct device_drv knk_drv = {
	.drv_id = DRIVER_knk,
#ifdef KNK_MPSSE_SPI
	.dname = "KNK_BitFury_MPSSE",
#else
	.dname = "KNK_BitFury_GPIO",
#endif
	.name = "KNK",
	.drv_detect = knk_detect,
	.reinit_device = knk_init,
	.identify_device = knk_identify,

	.thread_prepare = knk_thread_prepare,
	.thread_shutdown = knk_shutdown,

	.scanwork = knk_scanwork,
	.hash_work = hash_queued_work,
	.flush_work = knk_flush_work,
	.queue_full = knk_queue_full,

	.get_api_stats = knk_get_api_stats,
};
