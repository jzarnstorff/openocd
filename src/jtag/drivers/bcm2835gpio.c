// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013 by Paul Fertser, fercerpav@gmail.com               *
 *                                                                         *
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *   Based on at91rm9200.c (c) Anders Larsen                               *
 *   and RPi GPIO examples by Gert van Loo & Dom                           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

uint32_t bcm2835_peri_base = 0x20000000;
#define BCM2835_GPIO_BASE	(bcm2835_peri_base + 0x200000) /* GPIO controller */

#define BCM2835_PADS_GPIO_BASE          (bcm2835_peri_base + 0x100000)
#define BCM2835_PADS_GPIO_0_27_OFFSET   (0x2c / 4)
#define BCM2835_PADS_GPIO_28_45_OFFSET  (0x30 / 4)
#define BCM2835_PADS_GPIO_46_53_OFFSET  (0x34 / 4)

#define BCM2835_PADS_GPIO_PASSWORD          (0x5a << 24)
#define BCM2835_PADS_GPIO_SLEW_LIMITED      (0 << 4)
#define BCM2835_PADS_GPIO_SLEW_NOT_LIMITED  (1 << 4)
#define BCM2835_PADS_GPIO_HYST_DISABLED     (0 << 3)
#define BCM2835_PADS_GPIO_HYST_ENABLED      (1 << 3)

enum bcm2835_pads_gpio_drv {
	DRIVE_STRENGTH_2MA,
	DRIVE_STRENGTH_4MA,
	DRIVE_STRENGTH_6MA,
	DRIVE_STRENGTH_8MA,
	DRIVE_STRENGTH_10MA,
	DRIVE_STRENGTH_12MA,
	DRIVE_STRENGTH_14MA,
	DRIVE_STRENGTH_16MA,
};

/* See "GPIO Function Select Registers (GPFSELn)" in "Broadcom BCM2835 ARM Peripherals" datasheet. */
#define BCM2835_GPIO_MODE_INPUT 0
#define BCM2835_GPIO_MODE_OUTPUT 1

#define BCM2835_GPIO_REG_READ(offset) \
    (*(pio_base + (offset)))

#define BCM2835_GPIO_REG_WRITE(offset, value) \
    (*(pio_base + (offset)) = (value))

#define BCM2835_GPIO_SET_REG_BITS(offset, bit_mask) \
    (*(pio_base + (offset)) |= (bit_mask))

#define BCM2835_GPIO_CLEAR_REG_BITS(offset, bit_mask) \
    (*(pio_base + (offset)) &= ~(bit_mask))

/* GPIO setup macros */
#define MODE_GPIO(gpio_pin_num) /* read the function select bits of specified GPIO pin number */ \
    BCM2835_GPIO_REG_READ(((gpio_pin_num) / 10)) >> (((gpio_pin_num) % 10) * 3) & 7

#define INP_GPIO(gpio_pin_num) /* set GPIO pin number as input */ \
    BCM2835_GPIO_CLEAR_REG_BITS(((gpio_pin_num / 10)), (7 << (((gpio_pin_num) % 10) * 3)))

#define SET_MODE_GPIO(gpio_pin_num, bcm2835_gpio_mode) do { /* clear the mode bits first, then set as necessary */ \
    INP_GPIO(gpio_pin_num);						\
    BCM2835_GPIO_SET_REG_BITS(((gpio_pin_num) / 10), ((bcm2835_gpio_mode) << (((gpio_pin_num) % 10) * 3))); } while(0)

#define OUT_GPIO(gpio_pin_num) /* set GPIO pin number as output */ \
    SET_MODE_GPIO(gpio_pin_num, BCM2835_GPIO_MODE_OUTPUT)

#define GPIO_SET(offset, shift, value) /* GPSET[7,8] register sets bits which are 1, ignores bits which are 0 */ \
    BCM2835_GPIO_REG_WRITE((7 + (offset)), ((value) << (shift)))

#define GPIO_CLR(offset, shift, value) /* GPCLR[10,11] register clears bits which are 1, ignores bits which are 0 */ \
    BCM2835_GPIO_REG_WRITE((10 + (offset)), ((value) << (shift)))

#define GPIO_PIN_LEVEL(offset, shift) /* current level of the pin */ \
    (BCM2835_GPIO_REG_READ((13 + (offset))) >> (shift) & 1)

static int dev_mem_fd;
static volatile uint32_t *pio_base = MAP_FAILED;
static volatile uint32_t *pads_base = MAP_FAILED;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static const struct adapter_gpio_config *adapter_gpio_config;
static struct gpio_pin_config {
	unsigned int initial_mode;
	unsigned int initial_output_level;
	unsigned int register_offset;
	unsigned int shift_amount;
} gpio_pin_config[ADAPTER_GPIO_IDX_NUM];
static uint32_t initial_drive_strengths[3];

static inline void bcm2835_gpio_synchronize(void)
{
	/* Ensure that previous writes to GPIO registers are flushed out of
	 * the inner shareable domain to prevent pipelined writes to the
	 * same address being merged.
	 */
	__sync_synchronize();
}

static inline void bcm2835_delay(void)
{
	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");
}

static bool is_gpio_config_valid(enum adapter_gpio_config_index idx)
{
	/* Only chip 0 is supported, accept unset value (-1) too */
	return adapter_gpio_config[idx].chip_num >= -1
		&& adapter_gpio_config[idx].chip_num <= 0
		&& adapter_gpio_config[idx].gpio_num >= 0
		&& adapter_gpio_config[idx].gpio_num <= 53;
}

static void set_gpio_value(enum adapter_gpio_config_index idx, int value)
{
	const struct adapter_gpio_config gpio_config = adapter_gpio_config[idx];
	value = value ^ (gpio_config.active_low ? 1 : 0);
	switch (gpio_config.drive) {
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		if (value)
			GPIO_SET(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
		else
			GPIO_CLR(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
		/* For performance reasons assume the GPIO is already set as an output
		 * and therefore the call can be omitted here.
		 */
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		if (value) {
			INP_GPIO(gpio_config.gpio_num);
		} else {
			GPIO_CLR(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
			OUT_GPIO(gpio_config.gpio_num);
		}
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		if (value) {
			GPIO_SET(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
			OUT_GPIO(gpio_config.gpio_num);
		} else {
			INP_GPIO(gpio_config.gpio_num);
		}
		break;
	}
	bcm2835_gpio_synchronize();
}

static void restore_gpio(enum adapter_gpio_config_index idx)
{
	if (is_gpio_config_valid(idx)) {
		SET_MODE_GPIO(adapter_gpio_config[idx].gpio_num, gpio_pin_config[idx].initial_mode);
		if (gpio_pin_config[idx].initial_mode == BCM2835_GPIO_MODE_OUTPUT) {
			if (gpio_pin_config[idx].initial_output_level)
				GPIO_SET(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
			else
				GPIO_CLR(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount, 1);
		}
	}
	bcm2835_gpio_synchronize();
}

static void initialize_gpio(enum adapter_gpio_config_index idx)
{
	if (!is_gpio_config_valid(idx))
		return;

	gpio_pin_config[idx].register_offset = adapter_gpio_config[idx].gpio_num / 32;
	gpio_pin_config[idx].shift_amount = adapter_gpio_config[idx].gpio_num % 32;
	gpio_pin_config[idx].initial_mode = MODE_GPIO(adapter_gpio_config[idx].gpio_num);
	gpio_pin_config[idx].initial_output_level = GPIO_PIN_LEVEL(gpio_pin_config[idx].register_offset, gpio_pin_config[idx].shift_amount);
	LOG_DEBUG("saved GPIO mode for %s (GPIO %d %d): %d",
			adapter_gpio_get_name(idx), adapter_gpio_config[idx].chip_num, adapter_gpio_config[idx].gpio_num,
			gpio_pin_config[idx].initial_mode);

	if (adapter_gpio_config[idx].pull != ADAPTER_GPIO_PULL_NONE) {
		LOG_WARNING("BCM2835 GPIO does not support pull-up or pull-down settings (signal %s)",
			adapter_gpio_get_name(idx));
	}

	switch (adapter_gpio_config[idx].init_state) {
	case ADAPTER_GPIO_INIT_STATE_INACTIVE:
		set_gpio_value(idx, 0);
		break;
	case ADAPTER_GPIO_INIT_STATE_ACTIVE:
		set_gpio_value(idx, 1);
		break;
	case ADAPTER_GPIO_INIT_STATE_INPUT:
		INP_GPIO(adapter_gpio_config[idx].gpio_num);
		break;
	}

	/* Direction for non push-pull is already set by set_gpio_value() */
	if (adapter_gpio_config[idx].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL)
		OUT_GPIO(adapter_gpio_config[idx].gpio_num);
	bcm2835_gpio_synchronize();
}

static bb_value_t bcm2835gpio_read(void)
{
	uint32_t value = GPIO_PIN_LEVEL(gpio_pin_config[ADAPTER_GPIO_IDX_TDO].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TDO].shift_amount);
	return value ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].active_low ? BB_HIGH : BB_LOW);

}

static int bcm2835gpio_write(int tck, int tms, int tdi)
{
	GPIO_SET(gpio_pin_config[ADAPTER_GPIO_IDX_TCK].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TCK].shift_amount, tck);
	GPIO_SET(gpio_pin_config[ADAPTER_GPIO_IDX_TMS].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TMS].shift_amount, tms);
	GPIO_SET(gpio_pin_config[ADAPTER_GPIO_IDX_TDI].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TDI].shift_amount, tdi);

	GPIO_CLR(gpio_pin_config[ADAPTER_GPIO_IDX_TCK].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TCK].shift_amount, !tck);
	GPIO_CLR(gpio_pin_config[ADAPTER_GPIO_IDX_TMS].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TMS].shift_amount, !tms);
	GPIO_CLR(gpio_pin_config[ADAPTER_GPIO_IDX_TDI].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_TDI].shift_amount, !tdi);

	bcm2835_gpio_synchronize();

	bcm2835_delay();

	return ERROR_OK;
}

/* Requires push-pull drive mode for swclk and swdio */
static int bcm2835gpio_swd_write_fast(int swclk, int swdio)
{
	swclk = swclk ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].active_low ? 1 : 0);
	swdio = swdio ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].active_low ? 1 : 0);

	GPIO_SET(gpio_pin_config[ADAPTER_GPIO_IDX_SWCLK].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_SWCLK].shift_amount, swclk);
	GPIO_SET(gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].shift_amount, swdio);

	GPIO_CLR(gpio_pin_config[ADAPTER_GPIO_IDX_SWCLK].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_SWCLK].shift_amount, !swclk);
	GPIO_CLR(gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].shift_amount, !swdio);
	bcm2835_gpio_synchronize();

	bcm2835_delay();

	return ERROR_OK;
}

/* Generic mode that works for open-drain/open-source drive modes, but slower */
static int bcm2835gpio_swd_write_generic(int swclk, int swdio)
{
	set_gpio_value(ADAPTER_GPIO_IDX_SWDIO, swdio);
	set_gpio_value(ADAPTER_GPIO_IDX_SWCLK, swclk); /* Write clock last */

	bcm2835_delay();

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int bcm2835gpio_reset(int trst, int srst)
{
	/* As the "adapter reset_config" command keeps the srst and trst gpio drive
	 * mode settings in sync we can use our standard set_gpio_value() function
	 * that honours drive mode and active low.
	 */
	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SRST))
		set_gpio_value(ADAPTER_GPIO_IDX_SRST, srst);

	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_TRST))
		set_gpio_value(ADAPTER_GPIO_IDX_TRST, trst);

	LOG_DEBUG("BCM2835 GPIO: bcm2835gpio_reset(%d, %d), trst_gpio: %d %d, srst_gpio: %d %d",
		trst, srst,
		adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].chip_num, adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].gpio_num,
		adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].chip_num, adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].gpio_num);
	return ERROR_OK;
}

static void bcm2835_swdio_drive(bool is_output)
{
	if (is_output) {
		if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO_DIR))
			set_gpio_value(ADAPTER_GPIO_IDX_SWDIO_DIR, 1);
		OUT_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
	} else {
		INP_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
		if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO_DIR))
			set_gpio_value(ADAPTER_GPIO_IDX_SWDIO_DIR, 0);
	}
	bcm2835_gpio_synchronize();
}

static int bcm2835_swdio_read(void)
{
	uint32_t value = GPIO_PIN_LEVEL(gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].register_offset, gpio_pin_config[ADAPTER_GPIO_IDX_SWDIO].shift_amount);
	return value ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].active_low ? 1 : 0);
}

static int bcm2835gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("BCM2835 GPIO: RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = DIV_ROUND_UP(speed_coeff, khz) - speed_offset;
	LOG_DEBUG("jtag_delay %d", *jtag_speed);
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int bcm2835gpio_speed_div(int speed, int *khz)
{
	int divisor = speed + speed_offset;
	/* divide with roundig to the closest */
	*khz = (speed_coeff + divisor / 2) / divisor;
	return ERROR_OK;
}

static int bcm2835gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "BCM2835 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bcm2835_peri_base);

	command_print(CMD, "BCM2835 GPIO: peripheral_base = 0x%08x",
				  bcm2835_peri_base);
	return ERROR_OK;
}

static const struct command_registration bcm2835gpio_subcommand_handlers[] = {
	{
		.name = "speed_coeffs",
		.handler = &bcm2835gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "peripheral_base",
		.handler = &bcm2835gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration bcm2835gpio_command_handlers[] = {
	{
		.name = "bcm2835gpio",
		.mode = COMMAND_ANY,
		.help = "perform bcm2835gpio management",
		.chain = bcm2835gpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static bool bcm2835gpio_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TCK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TMS))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDI))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDO))
		return false;
	return true;
}

static bool bcm2835gpio_swd_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWCLK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO))
		return false;
	return true;
}

static void bcm2835gpio_munmap(void)
{
	if (pio_base != MAP_FAILED) {
		munmap((void *)pio_base, sysconf(_SC_PAGE_SIZE));
		pio_base = MAP_FAILED;
	}

	if (pads_base != MAP_FAILED) {
		munmap((void *)pads_base, sysconf(_SC_PAGE_SIZE));
		pads_base = MAP_FAILED;
	}
}

static int bcm2835gpio_blink(int on)
{
	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_LED))
		set_gpio_value(ADAPTER_GPIO_IDX_LED, on);

	return ERROR_OK;
}

static struct bitbang_interface bcm2835gpio_bitbang = {
	.read = bcm2835gpio_read,
	.write = bcm2835gpio_write,
	.swdio_read = bcm2835_swdio_read,
	.swdio_drive = bcm2835_swdio_drive,
	.swd_write = bcm2835gpio_swd_write_generic,
	.blink = bcm2835gpio_blink,
};

static int bcm2835gpio_init(void)
{
	LOG_INFO("BCM2835 GPIO JTAG/SWD bitbang driver");

	bitbang_interface = &bcm2835gpio_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	if (transport_is_jtag() && !bcm2835gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !bcm2835gpio_swd_mode_possible()) {
		LOG_ERROR("Require swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_DEBUG("Cannot open /dev/gpiomem, fallback to /dev/mem");
		dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	}
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_PADS_GPIO_BASE);

	if (pads_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		bcm2835gpio_munmap();
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	close(dev_mem_fd);

    for (unsigned int i = 0; i < sizeof(initial_drive_strengths) / sizeof(initial_drive_strengths[0]); i++) {
		initial_drive_strengths[i] = pads_base[BCM2835_PADS_GPIO_0_27_OFFSET + i] & 0x1f;
		pads_base[BCM2835_PADS_GPIO_0_27_OFFSET + i] = BCM2835_PADS_GPIO_PASSWORD | BCM2835_PADS_GPIO_SLEW_LIMITED | BCM2835_PADS_GPIO_HYST_ENABLED | DRIVE_STRENGTH_4MA;
	}

	/* Configure JTAG/SWD signals. Default directions and initial states are handled
	 * by adapter.c and "adapter gpio" command.
	 */
	if (transport_is_jtag()) {
		initialize_gpio(ADAPTER_GPIO_IDX_TDO);
		initialize_gpio(ADAPTER_GPIO_IDX_TDI);
		initialize_gpio(ADAPTER_GPIO_IDX_TMS);
		initialize_gpio(ADAPTER_GPIO_IDX_TCK);
		initialize_gpio(ADAPTER_GPIO_IDX_TRST);
	}

	if (transport_is_swd()) {
		/* swdio and its buffer should be initialized in the order that prevents
		 * two outputs from being connected together. This will occur if the
		 * swdio GPIO of the AM335x is configured as an output while its
		 * external buffer is configured to send the swdio signal from the
		 * target to the AM335x.
		 */
		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].init_state == ADAPTER_GPIO_INIT_STATE_INPUT) {
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO);
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
		} else {
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO);
		}

		initialize_gpio(ADAPTER_GPIO_IDX_SWCLK);

		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL &&
				adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL) {
			LOG_DEBUG("BCM2835 GPIO using fast mode for SWD write");
			bcm2835gpio_bitbang.swd_write = bcm2835gpio_swd_write_fast;
		} else {
			LOG_DEBUG("BCM2835 GPIO using generic mode for SWD write");
			bcm2835gpio_bitbang.swd_write = bcm2835gpio_swd_write_generic;
		}
	}

	initialize_gpio(ADAPTER_GPIO_IDX_SRST);
	initialize_gpio(ADAPTER_GPIO_IDX_LED);

	return ERROR_OK;
}

static int bcm2835gpio_quit(void)
{
	if (transport_is_jtag()) {
		restore_gpio(ADAPTER_GPIO_IDX_TDO);
		restore_gpio(ADAPTER_GPIO_IDX_TDI);
		restore_gpio(ADAPTER_GPIO_IDX_TCK);
		restore_gpio(ADAPTER_GPIO_IDX_TMS);
		restore_gpio(ADAPTER_GPIO_IDX_TRST);
	}

	if (transport_is_swd()) {
		/* Restore swdio/swdio_dir to their initial modes, even if that means
		 * connecting two outputs. Begin by making swdio an input so that the
		 * current and final states of swdio and swdio_dir do not have to be
		 * considered to calculate the safe restoration order.
		 */
		INP_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO);
		restore_gpio(ADAPTER_GPIO_IDX_SWCLK);
	}

	restore_gpio(ADAPTER_GPIO_IDX_SRST);
	restore_gpio(ADAPTER_GPIO_IDX_LED);

    for (unsigned int i = 0; i < sizeof(initial_drive_strengths) / sizeof(initial_drive_strengths[0]); i++)
		pads_base[BCM2835_PADS_GPIO_0_27_OFFSET + i] = BCM2835_PADS_GPIO_PASSWORD | initial_drive_strengths[i];
	bcm2835gpio_munmap();

	return ERROR_OK;
}


static const char * const bcm2835_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface bcm2835gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};
struct adapter_driver bcm2835gpio_adapter_driver = {
	.name = "bcm2835gpio",
	.transports = bcm2835_transports,
	.commands = bcm2835gpio_command_handlers,

	.init = bcm2835gpio_init,
	.quit = bcm2835gpio_quit,
	.reset = bcm2835gpio_reset,
	.speed = bcm2835gpio_speed,
	.khz = bcm2835gpio_khz,
	.speed_div = bcm2835gpio_speed_div,

	.jtag_ops = &bcm2835gpio_interface,
	.swd_ops = &bitbang_swd,
};
