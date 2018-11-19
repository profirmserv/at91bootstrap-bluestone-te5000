/* ----------------------------------------------------------------------------
 *         Microchip Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2017, Microchip Corporation
 * Copyright (c) 2018, Innovative Electronics, LLC
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Microchip's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "ddramc.h"
#include "debug.h"
#include "gpio.h"
#include "hardware.h"
#include "l2cc.h"
#include "matrix.h"
#include "pmc.h"
#include "bluestone_te5000.h"
#include "string.h"
#include "timer.h"
#include "usart.h"
#include "watchdog.h"
#include "arch/at91_ddrsdrc.h"
#include "arch/at91_pio.h"
#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/at91_sfr.h"
#include "arch/tz_matrix.h"
#include "twi.h"

static void at91_dbgu_hw_init(void)
{
	const struct pio_desc dbgu_pins[] = {
		{"RXD1", CONFIG_SYS_DBGU_RXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"TXD1", CONFIG_SYS_DBGU_TXD_PIN, 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};

	pio_configure(dbgu_pins);
	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_DBGU_ID);
}

static void initialize_dbgu(void)
{
	unsigned int baudrate = 115200;

	at91_dbgu_hw_init();

	if (pmc_check_mck_h32mxdiv())
		usart_init(BAUDRATE(MASTER_CLOCK / 2, baudrate));
	else
		usart_init(BAUDRATE(MASTER_CLOCK, baudrate));
}

#if defined(CONFIG_MATRIX)
static int matrix_configure_slave(void)
{
	unsigned int ddr_port;
	unsigned int ssr_setting, sasplit_setting, srtop_setting;

	/*
	 * Matrix 0 (H64MX)
	 */

	/*
	 * 0: Bridge from H64MX to AXIMX
	 * (Internal ROM, Crypto Library, PKCC RAM): Always Secured
	 */

	/* 1: H64MX Peripheral Bridge */

	/* 2 ~ 9 DDR2 Port1 ~ 7: Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_128M);
	ssr_setting = MATRIX_LANSECH_NS(0) |
		      MATRIX_RDNSECH_NS(0) |
		      MATRIX_WRNSECH_NS(0);
	/* DDR port 0 not used from NWd */
	for (ddr_port = 1; ddr_port < 8; ddr_port++) {
		matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					(H64MX_SLAVE_DDR2_PORT_0 + ddr_port),
					srtop_setting,
					sasplit_setting,
					ssr_setting);
	}

	/*
	 * 10: Internal SRAM 128K
	 * TOP0 is set to 128K
	 * SPLIT0 is set to 64K
	 * LANSECH0 is set to 0, the low area of region 0 is the Securable one
	 * RDNSECH0 is set to 0, region 0 Securable area is secured for reads.
	 * WRNSECH0 is set to 0, region 0 Securable area is secured for writes
	 */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_128K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_64K);
	ssr_setting = MATRIX_LANSECH_S(0) |
		      MATRIX_RDNSECH_S(0) |
		      MATRIX_WRNSECH_S(0);
	matrix_configure_slave_security(AT91C_BASE_MATRIX64,
					H64MX_SLAVE_INTERNAL_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 11:  Internal SRAM 128K (Cache L2) */
	/* 12:  QSPI0 */
	/* 13:  QSPI1 */
	/* 14:  AESB */

	/*
	 * Matrix 1 (H32MX)
	 */

	/* 0: Bridge from H32MX to H64MX: Not Secured */

	/* 1: H32MX Peripheral Bridge 0: Not Secured */

	/* 2: H32MX Peripheral Bridge 1: Not Secured */

	/*
	 * 3: External Bus Interface
	 * EBI CS0 Memory(256M) ----> Slave Region 0, 1
	 * EBI CS1 Memory(256M) ----> Slave Region 2, 3
	 * EBI CS2 Memory(256M) ----> Slave Region 4, 5
	 * EBI CS3 Memory(128M) ----> Slave Region 6
	 * NFC Command Registers(128M) -->Slave Region 7
	 *
	 * NANDFlash(EBI CS3) --> Slave Region 6: Non-Secure
	 */
	srtop_setting =	MATRIX_SRTOP(6, MATRIX_SRTOP_VALUE_128M) |
			MATRIX_SRTOP(7, MATRIX_SRTOP_VALUE_128M);
	sasplit_setting = MATRIX_SASPLIT(6, MATRIX_SASPLIT_VALUE_128M) |
			  MATRIX_SASPLIT(7, MATRIX_SASPLIT_VALUE_128M);
	ssr_setting = MATRIX_LANSECH_NS(6) |
		      MATRIX_RDNSECH_NS(6) |
		      MATRIX_WRNSECH_NS(6) |
		      MATRIX_LANSECH_NS(7) |
		      MATRIX_RDNSECH_NS(7) |
		      MATRIX_WRNSECH_NS(7);
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_EXTERNAL_EBI,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 4: NFC SRAM (4K): Non-Secure */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_8K);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_8K);
	ssr_setting = MATRIX_LANSECH_NS(0) |
		      MATRIX_RDNSECH_NS(0) |
		      MATRIX_WRNSECH_NS(0);
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_NFC_SRAM,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	/* 5:
	 * USB Device High Speed Dual Port RAM (DPR): 1M
	 * USB Host OHCI registers: 1M
	 * USB Host EHCI registers: 1M
	 */
	srtop_setting = MATRIX_SRTOP(0, MATRIX_SRTOP_VALUE_1M) |
			MATRIX_SRTOP(1, MATRIX_SRTOP_VALUE_1M) |
			MATRIX_SRTOP(2, MATRIX_SRTOP_VALUE_1M);
	sasplit_setting = MATRIX_SASPLIT(0, MATRIX_SASPLIT_VALUE_1M) |
			  MATRIX_SASPLIT(1, MATRIX_SASPLIT_VALUE_1M) |
			  MATRIX_SASPLIT(2, MATRIX_SASPLIT_VALUE_1M);
	ssr_setting = MATRIX_LANSECH_NS(0) |
		      MATRIX_LANSECH_NS(1) |
		      MATRIX_LANSECH_NS(2) |
		      MATRIX_RDNSECH_NS(0) |
		      MATRIX_RDNSECH_NS(1) |
		      MATRIX_RDNSECH_NS(2) |
		      MATRIX_WRNSECH_NS(0) |
		      MATRIX_WRNSECH_NS(1) |
		      MATRIX_WRNSECH_NS(2);
	matrix_configure_slave_security(AT91C_BASE_MATRIX32,
					H32MX_USB,
					srtop_setting,
					sasplit_setting,
					ssr_setting);

	return 0;
}

static unsigned int security_ps_peri_id[] = {
	0,
};

static int matrix_config_periheral(void)
{
	unsigned int *peri_id = security_ps_peri_id;
	unsigned int array_size = sizeof(security_ps_peri_id) /
				  sizeof(unsigned int);
	int ret;

	ret = matrix_configure_peri_security(peri_id, array_size);
	if (ret)
		return -1;

	return 0;
}

static int matrix_init(void)
{
	int ret;

	matrix_write_protect_disable(AT91C_BASE_MATRIX64);
	matrix_write_protect_disable(AT91C_BASE_MATRIX32);

	ret = matrix_configure_slave();
	if (ret)
		return -1;

	ret = matrix_config_periheral();
	if (ret)
		return -1;

	return 0;
}
#endif

#ifdef CONFIG_DDR2
static void ddramc_reg_config(struct ddramc_register *ddramc_config)
{
	ddramc_config->mdr = AT91C_DDRC2_DBW_16_BITS |
			     AT91C_DDRC2_MD_DDR2_SDRAM;

	ddramc_config->cr = AT91C_DDRC2_NC_DDR10_SDR9 |
			    AT91C_DDRC2_NR_13 |
			    AT91C_DDRC2_CAS_3 |
			    AT91C_DDRC2_DISABLE_RESET_DLL |
			    AT91C_DDRC2_WEAK_STRENGTH_RZQ7 |
			    AT91C_DDRC2_ENABLE_DLL |
			    AT91C_DDRC2_NB_BANKS_8 |
			    AT91C_DDRC2_NDQS_ENABLED |
			    AT91C_DDRC2_DECOD_INTERLEAVED |
			    AT91C_DDRC2_UNAL_SUPPORTED;

	ddramc_config->rtr = 0x511;

	ddramc_config->t0pr = AT91C_DDRC2_TRAS_(7) |
			      AT91C_DDRC2_TRCD_(3) |
			      AT91C_DDRC2_TWR_(3) |
			      AT91C_DDRC2_TRC_(9) |
			      AT91C_DDRC2_TRP_(3) |
			      AT91C_DDRC2_TRRD_(2) |
			      AT91C_DDRC2_TWTR_(2) |
			      AT91C_DDRC2_TMRD_(2);

	ddramc_config->t1pr = AT91C_DDRC2_TRFC_(22) |
			      AT91C_DDRC2_TXSNR_(23) |
			      AT91C_DDRC2_TXSRD_(200) |
			      AT91C_DDRC2_TXP_(2);

	ddramc_config->t2pr = AT91C_DDRC2_TXARD_(2) |
			      AT91C_DDRC2_TXARDS_(8) |
			      AT91C_DDRC2_TRPA_(4) |
			      AT91C_DDRC2_TRTP_(2) |
			      AT91C_DDRC2_TFAW_(8);
}

static void ddr2_init(void)
{
	struct ddramc_register ddramc_reg;
	unsigned int reg;

	ddramc_reg_config(&ddramc_reg);

	pmc_enable_periph_clock(AT91C_ID_MPDDRC);
	pmc_enable_system_clock(AT91C_PMC_DDR);

	reg = AT91C_MPDDRC_RD_DATA_PATH_ONE_CYCLES;
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_RD_DATA_PATH));

	reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
	reg &= ~AT91C_MPDDRC_RDIV;
	reg &= ~AT91C_MPDDRC_TZQIO;
	reg |= AT91C_MPDDRC_RDIV_DDR2_RZQ_50;
	reg |= AT91C_MPDDRC_TZQIO_(101);
	writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

	ddram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);

	ddramc_dump_regs(AT91C_BASE_MPDDRC);
}
#endif

/**
 * The MSBs [bits 31:16] of the CAN Message RAM for CAN0 and CAN1
 * are configured in 0x210000, instead of the default configuration
 * 0x200000, to avoid conflict with SRAM map for PM.
 */
#define CAN_MESSAGE_RAM_MSB	0x21

void at91_init_can_message_ram(void)
{
	writel(AT91C_CAN0_MEM_ADDR_(CAN_MESSAGE_RAM_MSB) |
	       AT91C_CAN1_MEM_ADDR_(CAN_MESSAGE_RAM_MSB),
	       (AT91C_BASE_SFR + SFR_CAN));
}

static void at91_red_led_on(void)
{
	pio_set_gpio_output(AT91C_PIN_PA(10), 1);
	pio_set_gpio_output(AT91C_PIN_PA(31), 0);
	pio_set_gpio_output(AT91C_PIN_PB(01), 0);
}

static void te5000_beeper_off(void)
{
	pio_set_gpio_output(AT91C_PIN_PB(29), 0);
}

static void te5000_rf_setup(void)
{
	pio_set_gpio_output(AT91C_PIN_PB(14), 0);
	pio_set_gpio_output(AT91C_PIN_PB(15), 0);
	mdelay(1);
	pio_set_gpio_output(AT91C_PIN_PB(14), 1);
	mdelay(5);
	pio_set_gpio_output(AT91C_PIN_PB(15), 1);
}

#define LCD_ADDR 0x3c
#define LCD_COLS 20

static void lcd_reset(void)
{
	pio_set_gpio_output(AT91C_PIN_PB(17), 1);
	mdelay(10);
	pio_set_gpio_output(AT91C_PIN_PB(17), 0);
	mdelay(50);
	pio_set_gpio_output(AT91C_PIN_PB(17), 1);
	mdelay(100);
}

static void lcd_write(unsigned char *data, unsigned int bytes)
{
	twi_write(0, LCD_ADDR, 0, 0, data, bytes);
}

static void lcd_write_cmd(unsigned char cmd)
{
	unsigned char buf[] = {0x80, cmd};
	lcd_write(buf, sizeof(buf));
}

static void lcd_write_string(char *data)
{
	unsigned int len = strlen(data);
	unsigned char buf[LCD_COLS + 1];
	buf[0] = 0x40;
	memcpy(buf + 1, data, min(len, LCD_COLS));
	lcd_write(buf, len + 1);
}

static void lcd_init(void)
{
	/* Bootup sequence from WO2004.C, acquired from Winstar customer
	 * support */

	lcd_reset();

	lcd_write_cmd(0x01); /* Clear dispaly */

	mdelay(50);

	lcd_write_cmd(0x39); /* Function SET DL=1 N=1 DH=0 RE=0 IS=1 */
	lcd_write_cmd(0x06); /* Entry mode set I/D=1 S=0 */
	lcd_write_cmd(0x0c); /* Display on D=1 C=0 B=0 */
	lcd_write_cmd(0x1b); /* Internal OSC BS0=1 F[2:0]=011 */
	lcd_write_cmd(0x56); /* Power Icon  Contrast set ION=0 BON=1 C[5:4]=10 */
	lcd_write_cmd(0x6d); /* Follower control DON=1 Rab[2:0]==101=4.4 */
	lcd_write_cmd(0x75); /* Contrast set  VOP=C[3:0]=0011 73 7.52V; 78:7.69V;7B:7.77V;7D:7.83V;7F:7.89V */

	lcd_write_cmd(0x38); /* Function SET DL=1 N=1 DH=0 RE=0 IS=0 */
	lcd_write_cmd(0x3e); /* Function set DL=1 N=1 RE=1 */
	lcd_write_cmd(0x02); /* Power down mode PD=0 */
	lcd_write_cmd(0x05); /* Entry mode set BDC=0 BDS=1 */
	lcd_write_cmd(0x09); /* Extended function set FW=0 B/W=0 NW=1 */
	lcd_write_cmd(0x1e); /* Double hight/bisa/display-dot shift UD[2:1]=11 BS1=1 DH'=0 */
	lcd_write_cmd(0x80); /* Set scroll quantity SQ[5:0]=000000 */

	lcd_write_cmd(0x38); /* Reset RE and IS to 0 */
}

static void lcd_goto(unsigned char row, unsigned char col)
{
	static const unsigned char
		ddram_row_addrs[] = {0x00, 0x20, 0x40, 0x60};
	unsigned char ddram_addr = ddram_row_addrs[row] + col;
	lcd_write_cmd(0x80 | ddram_addr);
}

static void lcd_write_string_at(unsigned char row,
				unsigned char col,
				char *s)
{
	lcd_goto(row, col);
	lcd_write_string(s);
}

static void lcd_splash(void)
{
	lcd_init();
	lcd_write_string_at(0, 0, "   OnSite TE5000    ");
	lcd_write_string_at(1, 0, "   Starting Up...   ");
	lcd_write_string_at(3, 0, "    Please Wait     ");
}

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	at91_disable_wdt();

	at91_red_led_on();
	te5000_beeper_off();

	pmc_cfg_plla(PLLA_SETTINGS);

	/* Initialize PLLA charge pump */
	/* No need: we keep what is set in ROM code */
	//pmc_init_pll(0x3);
	pmc_cfg_mck(BOARD_PRESCALER_PLLA);

	writel(AT91C_RSTC_KEY_UNLOCK | AT91C_RSTC_URSTEN,
	       AT91C_BASE_RSTC + RSTC_RMR);

#ifdef CONFIG_MATRIX
	matrix_init();
#endif
	initialize_dbgu();

	timer_init();

#ifdef CONFIG_DDR2
	ddr2_init();
#endif
	l2cache_prepare();

	at91_init_can_message_ram();

        if (!twi_init_done)
                twi_init();

	te5000_rf_setup();
	lcd_splash();
}
#endif

#ifdef CONFIG_QSPI
void at91_qspi_hw_init(void)
{
#if defined(CONFIG_QSPI_BUS0)
#error "Board does not support booting from QSPI0"
#elif defined(CONFIG_QSPI_BUS1)

#if defined(CONFIG_QSPI1_IOSET_1)
#error "Board does not support booting from QSPI1_IOSET_1"
#elif defined(CONFIG_QSPI1_IOSET_2)
	const struct pio_desc qspi_pins[] = {
		{"QSPI1_SCK",	AT91C_PIN_PB(5),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_CS",	AT91C_PIN_PB(6),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO0",	AT91C_PIN_PB(7),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO1",	AT91C_PIN_PB(8),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO2",	AT91C_PIN_PB(9),  0, PIO_DEFAULT, PIO_PERIPH_D},
		{"QSPI1_IO3",	AT91C_PIN_PB(10), 0, PIO_DEFAULT, PIO_PERIPH_D},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#elif defined(CONFIG_QSPI1_IOSET_3)
#error "Board does not support booting from QSPI1_IOSET_3"
#else
#error "No QSPI1 IOSET defined"
#endif
#else
#error "No QSPI Bus defined"
#endif

	pio_configure(qspi_pins);
	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_QSPI);
}
#endif

#ifdef CONFIG_SDCARD
#ifdef CONFIG_OF_LIBFDT
void at91_board_set_dtb_name(char *of_name)
{
	strcpy(of_name, "at91-bluestone_te5000.dtb");
}
#endif

#define ATMEL_SDHC_GCKDIV_VALUE		1

void at91_sdhc_hw_init(void)
{
#ifdef CONFIG_SDHC0
	const struct pio_desc sdmmc_pins[] = {
		{"SDMMC0_CK",   AT91C_PIN_PA(0), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_CMD",  AT91C_PIN_PA(1), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT0", AT91C_PIN_PA(2), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT1", AT91C_PIN_PA(3), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT2", AT91C_PIN_PA(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_DAT3", AT91C_PIN_PA(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{"SDMMC0_CD",   AT91C_PIN_PA(13), 0, PIO_DEFAULT, PIO_PERIPH_A},
		{(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
	};
#endif

#ifdef CONFIG_SDHC1
#error "Board does not support booting from SDHC1"
#endif

	pio_configure(sdmmc_pins);

	pmc_sam9x5_enable_periph_clk(CONFIG_SYS_ID_SDHC);
	pmc_enable_periph_generated_clk(CONFIG_SYS_ID_SDHC,
					GCK_CSS_UPLL_CLK,
					ATMEL_SDHC_GCKDIV_VALUE);
}
#endif

#if defined(CONFIG_TWI0)
unsigned int at91_twi0_hw_init(void)
{
        unsigned int base_addr = AT91C_BASE_TWI0;

        const struct pio_desc twi_pins[] = {
                {"TWD0", AT91C_PIN_PD(21), 0, PIO_DEFAULT, PIO_PERIPH_B},
                {"TWCK0", AT91C_PIN_PD(22), 0, PIO_DEFAULT, PIO_PERIPH_B},
                {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
        };

        pio_configure(twi_pins);

        pmc_sam9x5_enable_periph_clk(AT91C_ID_TWI0);

        return base_addr;
}
#endif

#if defined(CONFIG_TWI1)
unsigned int at91_twi1_hw_init(void)
{
	return NULL;
}
#endif

#if defined(CONFIG_AUTOCONFIG_TWI_BUS)
void at91_board_config_twi_bus(void)
{
}
#endif
