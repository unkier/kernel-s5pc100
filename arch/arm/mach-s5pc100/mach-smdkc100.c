/* linux/arch/arm/mach-s5pc100/mach-smdkc100.c
 *
 * Copyright 2009 Samsung Electronics Co.
 * Author: Byungho Min <bhmin@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/pwm_backlight.h>

#include <linux/dm9000.h>
#include <linux/irq.h>

#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/map.h>
#include <mach/regs-gpio.h>

#include <video/platform_lcd.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>

#include <plat/nand.h>
//#include <plat/partition.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/fb.h>
#include <plat/iic.h>
#include <plat/ata.h>
#include <plat/adc.h>
#include <plat/keypad.h>
#include <plat/ts.h>
#include <plat/audio.h>
#include <plat/backlight.h>
#include <plat/regs-fb-v4.h>

#include "common.h"

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKC100_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKC100_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKC100_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S3C2440_UFCON_RXTRIG8 |	\
				 S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdkc100_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = SMDKC100_UCON_DEFAULT,
		.ulcon	     = SMDKC100_ULCON_DEFAULT,
		.ufcon	     = SMDKC100_UFCON_DEFAULT,
	},
};

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
	{I2C_BOARD_INFO("wm8580", 0x1b),},
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
};

/* LCD power controller */
static void smdkc100_lcd_power_set(struct plat_lcd_data *pd,
				   unsigned int power)
{
	if (power) {
		/* module reset */
		gpio_direction_output(S5PC100_GPH0(6), 1);
		mdelay(100);
		gpio_direction_output(S5PC100_GPH0(6), 0);
		mdelay(10);
		gpio_direction_output(S5PC100_GPH0(6), 1);
		mdelay(10);
	}
}

static struct plat_lcd_data smdkc100_lcd_power_data = {
	.set_power	= smdkc100_lcd_power_set,
};

static struct platform_device smdkc100_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkc100_lcd_power_data,
};

/* Frame Buffer */
static struct s3c_fb_pd_win smdkc100_fb_win0 = {
	.max_bpp	= 32,
	.default_bpp	= 16,
	.xres		= 800,
	.yres		= 480,
};

static struct fb_videomode smdkc100_lcd_timing = {
// 	.left_margin	= 8,
// 	.right_margin	= 13,
// 	.upper_margin	= 7,
// 	.lower_margin	= 5,
// 	.hsync_len	= 3,
// 	.vsync_len	= 1,
// 	.xres		= 800,
// 	.yres		= 480,  
// 	.refresh	= 80,
	.left_margin	= 50,
	.right_margin	= 2,
	.upper_margin	= 10,
	.lower_margin	= 2,
	.hsync_len	= 1,
	.vsync_len	= 2,
	.xres		= 800,
	.yres		= 480,  
	.refresh	= 60,
};

static struct s3c_fb_platdata smdkc100_lcd_pdata __initdata = {
	.win[0]		= &smdkc100_fb_win0,
	.vtiming	= &smdkc100_lcd_timing,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= s5pc100_fb_gpio_setup_24bpp,
};

static struct s3c_ide_platdata smdkc100_ide_pdata __initdata = {
	.setup_gpio	= s5pc100_ide_setup_gpio,
};

static uint32_t smdkc100_keymap[] __initdata = {
	/* KEY(row, col, keycode) */
	KEY(0, 3, KEY_1), KEY(0, 4, KEY_2), KEY(0, 5, KEY_3),
	KEY(0, 6, KEY_4), KEY(0, 7, KEY_5),
	KEY(1, 3, KEY_A), KEY(1, 4, KEY_B), KEY(1, 5, KEY_C),
	KEY(1, 6, KEY_D), KEY(1, 7, KEY_E)
};

static struct matrix_keymap_data smdkc100_keymap_data __initdata = {
	.keymap		= smdkc100_keymap,
	.keymap_size	= ARRAY_SIZE(smdkc100_keymap),
};

static struct samsung_keypad_platdata smdkc100_keypad_data __initdata = {
	.keymap_data	= &smdkc100_keymap_data,
	.rows		= 2,
	.cols		= 8,
};

/* Network DM9000 */
static struct resource dm9000_resources[] = {
        [0] = {
                .start  = 0x88000000,
                .end    = 0x88000003,
                .flags  = IORESOURCE_MEM,
                },
	[1] = {
                .start  = 0x88000004,
                .end    = 0x88000007,
                .flags  = IORESOURCE_MEM,
                },
        [2] = {
//                 .start = S5P_IRQ_VIC0(10),
//                 .end   = S5P_IRQ_VIC0(10),
		   .start = IRQ_EINT(10),
		   .end = IRQ_EINT(10),
                .flags  = IORESOURCE_IRQ | IRQ_TYPE_EDGE_RISING,
                },
};

static struct dm9000_plat_data s5pc100_dm9000_platdata = {
               .flags  = DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM,              
               .dev_addr[0] = 0xF0,
               .dev_addr[1] = 0xDE,
               .dev_addr[2] = 0xF1,                           
               .dev_addr[3] = 0x7E,
               .dev_addr[4] = 0xFF,
               .dev_addr[5] = 0xAA,
};

static struct platform_device s3c_device_dm9ks = { 
		.name   = "dm9000",
		.id     = -1,
		.num_resources  = ARRAY_SIZE(dm9000_resources),
		.resource       = dm9000_resources,
		.dev = {
		  .platform_data = &s5pc100_dm9000_platdata,
		 }
};

/* Bosch C_CAN */
static struct resource bosch_c_can_resources[] = {
	[0] = {
		.start	= 0xec700000,
		.end	= 0xec700000 + 256*2 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	      },
	[1] = {
		.start	= IRQ_IIC2,
		.end	= IRQ_IIC2,
		.flags	= IORESOURCE_IRQ,
	      },
};

static struct platform_device bosch_c_can = {
	.name	= "c_can_platform",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(bosch_c_can_resources),
	.resource	= bosch_c_can_resources,
};

static struct resource bosch_c_can_resources1[] = {
	[0] = {
		.start	= 0xec800000,
		.end	= 0xec800000 + 256*2 - 1,
		.flags	= IORESOURCE_MEM | IORESOURCE_MEM_32BIT,
	      },
	[1] = {
		.start	= IRQ_IIC3,
		.end	= IRQ_IIC3,
		.flags	= IORESOURCE_IRQ,
	      },
};

static struct platform_device bosch_c_can1 = {
	.name	= "c_can_platform",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(bosch_c_can_resources1),
	.resource	= bosch_c_can_resources1,
};

/* GPIO Keys */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#include <linux/gpio_keys.h>
#include <mach/gpio.h>
 
static struct gpio_keys_button smdkc100_gpio_keys_table[] = {
	{KEY_F1, S5PC100_GPH0(1), 1, "gpio-keys: F1", EV_KEY, 0, 20},
	{KEY_F2, S5PC100_GPH0(2), 1, "gpio-keys: F2", EV_KEY, 0, 20},
	{KEY_F3, S5PC100_GPH0(3), 1, "gpio-keys: F3", EV_KEY, 0, 20},
	{KEY_F4, S5PC100_GPH0(4), 1, "gpio-keys: F4", EV_KEY, 0, 20},
// 	{KEY_5, S5PC100_GPH0(5), 1, "gpio-keys: 5", EV_KEY, 0, 20},
// 	{KEY_6, S5PC100_GPH0(6), 1, "gpio-keys: 6", EV_KEY, 0, 20},
};
 
static struct gpio_keys_platform_data smdkc100_gpio_keys_data = {
	.buttons        = smdkc100_gpio_keys_table,
	.nbuttons       = ARRAY_SIZE(smdkc100_gpio_keys_table),
};
 
static struct platform_device smdkc100_device_gpiokeys = {
	.name      = "gpio-keys",
	.dev = {
		.platform_data = &smdkc100_gpio_keys_data,
	},
};
#endif


/* NAND Flash */

static struct mtd_partition smdkc100_default_nand_part[] __initdata = {
	[0] = {
		.name	= "u-boot",
		.size	= SZ_256K,
		.offset	= 0,
	},
	[1] = {
		.name	= "u-boot-env",
		.size	= SZ_128K,
		.offset	= SZ_256K,
	},
	[2] = {
		.name	= "kernel",
		/* 5 megabytes, for a kernel with no modules
		 * or a uImage with a ramdisk attached */
		.size	= 0x02800000,
		.offset	= SZ_256K + SZ_128K,
	},
	[3] = {
		.name	= "root",
		.offset	= 0x03000000,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct s3c_nand_mtd_info smdkc100_nand_sets[] __initdata = {
	[0] = {
		.chip_nr	= 1,
		.mtd_part_nr = ARRAY_SIZE(smdkc100_default_nand_part),
		.partition = smdkc100_default_nand_part,
	},
};

// static struct s3c2410_platform_nand smdkc100_nand_info __initdata = {
// 	.tacls		= 0,
// 	.twrph0		= 25,
// 	.twrph1		= 15,
// 	.nr_sets	= ARRAY_SIZE(smdkc100_nand_sets),
// 	.sets		= smdkc100_nand_sets,
// 	.ignore_unset_ecc = 1,
// };


static struct platform_device *smdkc100_devices[] __initdata = {
	&s3c_device_adc,
	&s3c_device_cfcon,
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_fb,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_ts,
	&s3c_device_wdt,
	&smdkc100_lcd_powerdev,
	&samsung_asoc_dma,
	&s5pc100_device_iis0,
	&samsung_device_keypad,
	&s5pc100_device_ac97,
	&s3c_device_rtc,
	&s3c_device_dm9ks,
	&bosch_c_can,
	&bosch_c_can1,
	&s3c_device_nand,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5pc100_device_spdif,
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
	&smdkc100_device_gpiokeys,
#endif
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdkc100_bl_gpio_info = {
	.no = S5PC100_GPD(0),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdkc100_bl_data = {
	.pwm_id = 0,
};

static void __init smdkc100_map_io(void)
{
	s3c_device_nand.name = "s5pc100-nand";
	s5pc100_init_io(NULL, 0);
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdkc100_uartcfgs, ARRAY_SIZE(smdkc100_uartcfgs));
}

static void __init smdkc100_machine_init(void)
{
	s3c24xx_ts_set_platdata(NULL);

	/* I2C */
	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_fb_set_platdata(&smdkc100_lcd_pdata);
	s3c_ide_set_platdata(&smdkc100_ide_pdata);

	samsung_keypad_set_platdata(&smdkc100_keypad_data);

	s5pc100_spdif_setup_gpio(S5PC100_SPDIF_GPD);

	/* LCD init */
	gpio_request(S5PC100_GPH0(6), "GPH0");
	smdkc100_lcd_power_set(&smdkc100_lcd_power_data, 0);

	samsung_bl_set(&smdkc100_bl_gpio_info, &smdkc100_bl_data);
	
	// SMC
	// request resources
	struct resource *_req;
	_req = request_mem_region(0xE7000000, 4, "S5PC100");
	u32 *_adr;
	_adr = ioremap(0xE7000000, 4);
	// set flags
	u32 _dat;
	_dat = readl(_adr);
	_dat = _dat | 0x30; // Memory Bank 1 width = 16, byte address
 	writel(_dat, _adr);
// 	dev_info(db->dev, "SMC Control register is: 0x%08x;\n", _dat);
	// release resources
	iounmap(_adr);
	release_resource(_req);
	kfree(_req);
	
	// NAND
	//s3c_nand_set_platdata(&smdkc100_nand_info);
	s3c_device_nand.dev.platform_data = &smdkc100_nand_sets;

	platform_add_devices(smdkc100_devices, ARRAY_SIZE(smdkc100_devices));
}

MACHINE_START(SMDKC100, "SMDKC100")
	/* Maintainer: Byungho Min <bhmin@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pc100_init_irq,
	.handle_irq	= vic_handle_irq,
	.map_io		= smdkc100_map_io,
	.init_machine	= smdkc100_machine_init,
	.timer		= &s3c24xx_timer,
	.restart	= s5pc100_restart,
MACHINE_END
