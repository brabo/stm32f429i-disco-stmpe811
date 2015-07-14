/*
 * Copyright (c) 2015 brabo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __STMPE811_H
#define __STMPE811_H

/* Private defines */
/* I2C address */
#define STMPE811_ADDRESS			0x41

/* STMPE811 Chip ID on reset */
#define STMPE811_CHIP_ID_VALUE			0x0811	/* Chip ID */

/* Registers */
#define STMPE811_CHIP_ID	0x00	/* STMPE811 Device identification */
#define STMPE811_ID_VER		0x02	/* STMPE811 Revision number */
#define STMPE811_SYS_CTRL1	0x03	/* Reset control */
#define STMPE811_SYS_CTRL2	0x04	/* Clock control */
#define STMPE811_SPI_CFG	0x08	/* SPI interface configuration */
#define STMPE811_INT_CTRL	0x09	/* Interrupt control register */
#define STMPE811_INT_EN		0x0A	/* Interrupt enable register */
#define STMPE811_INT_STA	0x0B	/* Interrupt status register */
#define STMPE811_GPIO_EN	0x0C	/* GPIO interrupt enable register */
#define STMPE811_GPIO_INT_STA	0x0D	/* GPIO interrupt status register */
#define STMPE811_ADC_INT_EN	0x0E	/* ADC interrupt enable register */
#define STMPE811_ADC_INT_STA	0x0F	/* ADC interface status register */
#define STMPE811_GPIO_SET_PIN	0x10	/* GPIO set pin register */
#define STMPE811_GPIO_CLR_PIN	0x11	/* GPIO clear pin register */
#define STMPE811_MP_STA		0x12	/* GPIO monitor pin state register */
#define STMPE811_GPIO_DIR	0x13	/* GPIO direction register */
#define STMPE811_GPIO_ED	0x14	/* GPIO edge detect register */
#define STMPE811_GPIO_RE	0x15	/* GPIO rising edge register */
#define STMPE811_GPIO_FE	0x16	/* GPIO falling edge register */
#define STMPE811_GPIO_AF	0x17	/* Alternate function register */
#define STMPE811_ADC_CTRL1	0x20	/* ADC control */
#define STMPE811_ADC_CTRL2	0x21	/* ADC control */
#define STMPE811_ADC_CAPT	0x22	/* To initiate ADC data acquisition */
#define STMPE811_ADC_DATA_CHO	0x30	/* ADC channel 0 */
#define STMPE811_ADC_DATA_CH1	0x32	/* ADC channel 1 */
#define STMPE811_ADC_DATA_CH2	0x34	/* ADC channel 2 */
#define STMPE811_ADC_DATA_CH3	0x36	/* ADC channel 3 */
#define STMPE811_ADC_DATA_CH4	0x38	/* ADC channel 4 */
#define STMPE811_ADC_DATA_CH5	0x3A	/* ADC channel 5 */
#define STMPE811_ADC_DATA_CH6	0x3C	/* ADC channel 6 */
#define STMPE811_ADC_DATA_CH7	0x3E	/* ADC channel 7 */
#define STMPE811_TSC_CTRL	0x40	/* 4-wire tsc setup */
#define STMPE811_TSC_CFG	0x41	/* Tsc configuration */
#define STMPE811_WDW_TR_X	0x42	/* Window setup for top right X */
#define STMPE811_WDW_TR_Y	0x44	/* Window setup for top right Y */
#define STMPE811_WDW_BL_X	0x46	/* Window setup for bottom left X */
#define STMPE811_WDW_BL_Y	0x48	/* Window setup for bottom left Y */
#define STMPE811_FIFO_TH	0x4A	/* FIFO level to generate interrupt */
#define STMPE811_FIFO_STA	0x4B	/* Current status of FIFO */
#define STMPE811_FIFO_SIZE	0x4C	/* Current filled level of FIFO */
#define STMPE811_TSC_DATA_X	0x4D	/* Data port for tsc data access */
#define STMPE811_TSC_DATA_Y	0x4F	/* Data port for tsc data access */
#define STMPE811_TSC_DATA_Z	0x51	/* Data port for tsc data access */
#define STMPE811_TSC_DATA_XYZ	0x52	/* Data port for tsc data access */
#define STMPE811_TSC_FRACTION_Z	0x56	/* Touchscreen controller FRACTION_Z */
#define STMPE811_TSC_DATA	0x57	/* Data port for tsc data access */
#define STMPE811_TSC_I_DRIVE	0x58	/* Touchscreen controller drivel */
#define STMPE811_TSC_SHIELD	0x59	/* Touchscreen controller shield */
#define STMPE811_TEMP_CTRL	0x60	/* Temperature sensor setup */
#define STMPE811_TEMP_DATA	0x61	/* Temperature data access port */
#define STMPE811_TEMP_TH	0x62	/* Threshold for temp controlled int */

#define STMPE811_SYS_CTRL2_ADC_OFF	(1 << 0)
#define STMPE811_SYS_CTRL2_TSC_OFF	(1 << 1)
#define STMPE811_SYS_CTRL2_GPIO_OFF	(1 << 2)
#define STMPE811_SYS_CTRL2_TS_OFF	(1 << 3)

#define STMPE811_TEMP_CTRL_EN		(1 << 0)
#define STMPE811_TEMP_CTRL_ACQ		(1 << 1)
#define STMPE811_TEMP_DATA_MSB_MASK	0x03

#define STMPE811_TSC_CTRL_EN		(1 << 0)

#define STMPE811_INT_EN_TOUCH_DET	(1 << 0)
#define STMPE811_INT_EN_FIFO_TH		(1 << 1)
#define STMPE811_INT_EN_FIFO_OFLOW	(1 << 2)
#define STMPE811_INT_EN_FIFO_FULL	(1 << 3)
#define STMPE811_INT_EN_FIFO_EMPTY	(1 << 4)
#define STMPE811_INT_EN_TEMP_SENS	(1 << 5)
#define STMPE811_INT_EN_ADC		(1 << 6)
#define STMPE811_INT_EN_GPIO		(1 << 7)

#define STMPE811_SYS_CTRL1_SOFT_RESET	(1 << 1)

#define STMPE811_FIFO_STA_TOUCH_DET	(1 << 0)

#define STMPE811_INT_CTRL_GLOBAL_INT	(1 << 0)

#define I2C_CR2_FREQ_MASK	0x3ff

#define I2C_CCR_CCRMASK	0xfff

#define I2C_TRISE_MASK	0x3f

/**
 * @brief  Default I2C used, on STM32F429-Discovery board
 */
#define STMPE811_I2C					I2C3

/**
 * @brief  Default I2C clock for STMPE811
 */
#ifndef STMPE811_I2C_CLOCK
#define STMPE811_I2C_CLOCK				100000
#endif

/**
 * @brief  Orientation enum
 * @note   You may need experimenting to get proper orientation to match your LCD
 */
typedef enum {
	stmpe811_portrait_1,
	stmpe811_portrait_2,
	stmpe811_landscape_1,
	stmpe811_landscape_2,
} stmpe811_orientation_t;

/**
 * @brief  Touch state enum
 */
typedef enum {
	stmpe811_state_pressed,
	stmpe811_state_released,
	stmpe811_state_ok,
	stmpe811_state_error
} stmpe811_state_t;

/**
 * @brief  Main structure
 */
typedef struct {
	uint16_t x;
	uint16_t y;
	stmpe811_state_t pressed;
	stmpe811_state_t last_pressed;
	stmpe811_orientation_t orientation;
} stmpe811_t;

void stmpe811_i2c_init(void);
uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode);
uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t data);
uint32_t i2c_read(uint32_t i2c, uint8_t address, uint8_t reg);
void stmpe811_write(uint8_t reg, uint8_t data);
uint32_t stmpe811_read(uint8_t reg);
void stmpe811_reset(void);
void stmpe811_enable_ts(void);
void stmpe811_disable_ts(void);
void stmpe811_enable_gpio(void);
void stmpe811_disable_gpio(void);
void stmpe811_enable_tsc(void);
void stmpe811_disable_tsc(void);
void stmpe811_enable_adc(void);
void stmpe811_disable_adc(void);
void stmpe811_enable_fifo_of(void);
void stmpe811_enable_fifo_th(void);
void stmpe811_enable_fifo_touch_det(void);
void stmpe811_set_adc_sample(uint8_t sample);
void stmpe811_set_adc_resolution(uint8_t res);
void stmpe811_set_adc_freq(uint8_t freq);
void stmpe811_set_gpio_af(uint8_t af);
void stmpe811_set_tsc(uint8_t tsc);
void stmpe811_set_fifo_th(uint8_t th);
void stmpe811_reset_fifo(void);
void stmpe811_set_tsc_fraction_z(uint8_t z);
void stmpe811_set_tsc_i_drive(uint8_t limit);
void stmpe811_set_int_sta(uint8_t status);
void stmpe811_enable_interrupts(void);
void stmpe811_temp_start(void);
void stmpe811_get_temp(void);
stmpe811_state_t stmpe811_init(void);
uint16_t stmpe811_read_x(uint16_t x);
uint16_t stmpe811_read_y(uint16_t y);
stmpe811_state_t stmpe811_read_touch(stmpe811_t *stmpe811_data);

#endif
