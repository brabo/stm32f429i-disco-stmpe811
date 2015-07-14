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

#include <stdint.h>
#include <stdio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include "stmpe811.h"

void stmpe811_temp_start(void) {
	uint32_t data;



	data = stmpe811_read(STMPE811_TEMP_CTRL);
	//printf("temp control: 0x%08X\r\n", data);
	data |= STMPE811_TEMP_CTRL_ACQ;
	data |= STMPE811_TEMP_CTRL_EN;
	stmpe811_write(STMPE811_TEMP_CTRL, (uint8_t )data);
	//data = stmpe811_read(STMPE811_TEMP_CTRL);
	//printf("temp control: 0x%08X\r\n", data);

}

void stmpe811_get_temp(void) {
	uint32_t temp;
	stmpe811_temp_start();
	temp = stmpe811_read(STMPE811_TEMP_DATA);
	//printf("temp data1 = 0x%08X\r\n", temp);
	temp = ((temp & STMPE811_TEMP_DATA_MSB_MASK) << 8) | stmpe811_read(STMPE811_TEMP_DATA + 1);
	//printf("temp data2 = 0x%08X\r\n", temp);
	//data = data * 3.3;
	//data = (uint32_t)(data * 7.51);
	//data = data - 273;
	//data = (data - 32) * (5 / 9);
	temp = (uint32_t)(((3.3 * temp) / 7.51) - 273);
	//data = (uint32_t)((data + 5) /10);
	//data = (uint32_t)((data + 5) / 10);
	//data = (uint32_t)((data - 32 - 255) * 5 / 9);
	printf("temp in celsius: %d\r\n", temp);
}

void stmpe811_i2c_init()
{
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_I2C3);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);

	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);

	i2c_peripheral_disable(I2C3); /* disable i2c during setup */
	i2c_reset(I2C3);

	i2c_set_fast_mode(I2C3);
	i2c_set_clock_frequency(I2C3, I2C_CR2_FREQ_42MHZ);
	i2c_set_ccr(I2C3, 35);
	i2c_set_trise(I2C3, 43);

	i2c_peripheral_enable(I2C3); /* finally enable i2c */

	i2c_set_own_7bit_slave_address(I2C3, 0x00);
}

uint8_t i2c_start(uint32_t i2c, uint8_t address, uint8_t mode)
{
	i2c_send_start(i2c);

	/* Wait for master mode selected */
	while (!((I2C_SR1(i2c) & I2C_SR1_SB)
		& (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

	i2c_send_7bit_address(i2c, address, mode);

	/* Waiting for address is transferred. */
	while (!(I2C_SR1(i2c) & I2C_SR1_ADDR)) {

	}

	/* Cleaning ADDR condition sequence. */
	uint32_t reg32 = I2C_SR2(i2c);
	(void) reg32; /* unused */

	return 0;
}

uint8_t i2c_write(uint32_t i2c, uint8_t address, uint8_t reg, uint8_t data)
{
	i2c_start(i2c, address, I2C_WRITE);

	i2c_send_data(i2c, reg);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));
	i2c_send_data(i2c, data);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_send_stop(i2c);

	return 0;
}

uint32_t i2c_read(uint32_t i2c, uint8_t address, uint8_t reg)
{
	while ((I2C_SR2(i2c) & I2C_SR2_BUSY));

	i2c_start(i2c, address, I2C_WRITE);
	i2c_send_data(i2c, reg);

	while (!(I2C_SR1(i2c) & (I2C_SR1_BTF)));

	i2c_start(i2c, address, I2C_READ);

	i2c_send_stop(i2c);

	while (!(I2C_SR1(i2c) & I2C_SR1_RxNE));

	uint32_t result = i2c_get_data(i2c);

	I2C_SR1(i2c) &= ~I2C_SR1_AF;

	return result;
}

void stmpe811_write(uint8_t reg, uint8_t data)
{
	i2c_write(I2C3, STMPE811_ADDRESS, reg, data);
}

uint32_t stmpe811_read(uint8_t reg)
{
	uint32_t data;
	data = i2c_read(I2C3, STMPE811_ADDRESS, reg);
	return data;
}

void stmpe811_reset(void)
{
	uint8_t data;

	data = i2c_read(I2C3, STMPE811_ADDRESS, STMPE811_SYS_CTRL1);

	data |= 0x02;

	i2c_write(I2C3, STMPE811_ADDRESS, STMPE811_SYS_CTRL1, data);

	data &= ~(0x02);

	i2c_write(I2C3, STMPE811_ADDRESS, STMPE811_SYS_CTRL1, data);
	msleep(2);
}

void stmpe811_enable_ts(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~STMPE811_SYS_CTRL2_TS_OFF;
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
}

void stmpe811_disable_ts(void)
{
	stmpe811_write(STMPE811_SYS_CTRL2, STMPE811_SYS_CTRL2_TS_OFF);
}

void stmpe811_enable_gpio(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~STMPE811_SYS_CTRL2_GPIO_OFF;
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
}

void stmpe811_disable_gpio(void)
{
	stmpe811_write(STMPE811_SYS_CTRL2, STMPE811_SYS_CTRL2_GPIO_OFF);
}

void stmpe811_enable_tsc(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~STMPE811_SYS_CTRL2_TSC_OFF;
	stmpe811_write(STMPE811_SYS_CTRL2, mode);

	mode = stmpe811_read(STMPE811_TSC_CTRL);
	mode |= STMPE811_TSC_CTRL_EN;
	stmpe811_write(STMPE811_TSC_CTRL, mode);
}

void stmpe811_disable_tsc(void)
{
	stmpe811_write(STMPE811_SYS_CTRL2, STMPE811_SYS_CTRL2_TSC_OFF);
}

void stmpe811_enable_adc(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_SYS_CTRL2);
	mode &= ~STMPE811_SYS_CTRL2_ADC_OFF;
	stmpe811_write(STMPE811_SYS_CTRL2, mode);
}

void stmpe811_disable_adc(void)
{
	stmpe811_write(STMPE811_SYS_CTRL2, STMPE811_SYS_CTRL2_ADC_OFF);
}

void stmpe811_enable_fifo_of(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_INT_EN);
	mode |= 0x04;
	stmpe811_write(STMPE811_INT_EN, mode);
}

void stmpe811_enable_fifo_th(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_INT_EN);
	mode |= 0x02;
	stmpe811_write(STMPE811_INT_EN, mode);
}

void stmpe811_enable_fifo_touch_det(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_INT_EN);
	mode |= 0x01;
	stmpe811_write(STMPE811_INT_EN, mode);
}

void stmpe811_set_adc_sample(uint8_t sample)
{
	stmpe811_write(STMPE811_ADC_CTRL1, sample);
}

void stmpe811_set_adc_resolution(uint8_t res)
{
	stmpe811_write(STMPE811_ADC_CTRL1, res);
}

void stmpe811_set_adc_freq(uint8_t freq)
{
	stmpe811_write(STMPE811_ADC_CTRL2, freq);
}

void stmpe811_set_gpio_af(uint8_t af)
{
	stmpe811_write(STMPE811_GPIO_AF, af);
}

void stmpe811_set_tsc(uint8_t tsc)
{
	stmpe811_write(STMPE811_TSC_CFG, tsc);
}

void stmpe811_set_fifo_th(uint8_t th)
{
	stmpe811_write(STMPE811_FIFO_TH, th);
}

void stmpe811_reset_fifo(void)
{
	uint8_t sta;
	sta = stmpe811_read(STMPE811_FIFO_STA);
	sta |= 0x01;
	stmpe811_write(STMPE811_FIFO_STA, sta);
	sta &= ~(0x01);
	stmpe811_write(STMPE811_FIFO_STA, sta);
}

void stmpe811_set_tsc_fraction_z(uint8_t z)
{
	stmpe811_write(STMPE811_TSC_FRACTION_Z, z);
}

void stmpe811_set_tsc_i_drive(uint8_t limit)
{
	stmpe811_write(STMPE811_TSC_I_DRIVE, limit);
}

void stmpe811_set_int_sta(uint8_t status)
{
	stmpe811_write(STMPE811_INT_STA, status);
}

void stmpe811_enable_interrupts(void)
{
	uint8_t mode;
	mode = stmpe811_read(STMPE811_INT_CTRL);
	mode |= 0x01;
	stmpe811_write(STMPE811_INT_CTRL, mode);
}

stmpe811_state_t stmpe811_init(void)
{
	stmpe811_i2c_init();
	msleep(50);

	stmpe811_reset();

	/* start of the documented initialization sequence */
	stmpe811_disable_ts();
	stmpe811_disable_gpio();

	stmpe811_enable_fifo_of();
	stmpe811_enable_fifo_th();
	stmpe811_enable_fifo_touch_det();

	stmpe811_set_adc_sample(0x40);     /* set to 80 cycles */
	stmpe811_set_adc_resolution(0x08); /* set adc resolution to 12 bit */

	stmpe811_set_adc_freq(0x01);	/* set adc clock speed to 3.25 MHz */

	stmpe811_set_gpio_af(0x00);	/* set GPIO AF to function as ts/adc */

	/*
	 * tsc cfg set to avg control = 4 samples
	 * touch detection delay = 500 microseconds
	 * panel driver settling time = 500 microcseconds
	 */
	stmpe811_set_tsc(0x9A);

	stmpe811_set_fifo_th(0x01);        /* set fifo threshold */
	stmpe811_reset_fifo();             /* reset fifo */

	/* set fractional part to 7, whole part to 1 */
	stmpe811_set_tsc_fraction_z(0x07);

	/* set current limit value to 50 mA */
	stmpe811_set_tsc_i_drive(0x01);

	stmpe811_enable_tsc();             /* enable tsc (touchscreen clock) */

	stmpe811_set_int_sta(0xFF);        /* clear interrupt status */

	stmpe811_enable_interrupts();
	/* end of the documented initialization sequence */

	msleep(2);

	return stmpe811_state_ok;
}

uint16_t stmpe811_read_x(uint16_t x)
{
	uint8_t data[2];
	int16_t val, dx;
	data[1] = stmpe811_read(STMPE811_TSC_DATA_X);
	data[0] = stmpe811_read(STMPE811_TSC_DATA_X + 1);
	val = (data[1] << 8 | (data[0] & 0xFF));

	if (val <= 3000) {
		val = 3900 - val;
	} else {
		val = 3800 - val;
	}

	val /= 15;

	if (val > 239) {
		val = 239;
	} else if (val < 0) {
		val = 0;
	}

	dx = (val > x) ? (val - x) : (x - val);
	if (dx > 4) {
		return val;
	}
	return x;
}

uint16_t stmpe811_read_y(uint16_t y)
{
	uint8_t data[2];
	int16_t val, dy;
	data[1] = stmpe811_read(STMPE811_TSC_DATA_Y);
	data[0] = stmpe811_read(STMPE811_TSC_DATA_Y + 1);
	val = (data[1] << 8 | (data[0] & 0xFF));

	val -= 360;
	val = val / 11;

	if (val <= 0) {
		val = 0;
	} else if (val >= 320) {
		val = 319;
	}

	dy = (val > y) ? (val - y) : (y - val);
	if (dy > 4) {
		return val;
	}
	return y;
}

stmpe811_state_t stmpe811_read_touch(stmpe811_t *stmpe811_data)
{
	uint8_t val;

	stmpe811_data->last_pressed = stmpe811_data->pressed;

	val = stmpe811_read(STMPE811_TSC_CTRL);

	if ((val & 0x80) == 0) {
		/* No touch detected, reset fifo and return released state */
		stmpe811_data->pressed = stmpe811_state_released;

		stmpe811_reset_fifo();

		return stmpe811_state_released;
	}

	/* Touch detected, reset fifo and return pressed state */
	stmpe811_reset_fifo();

	stmpe811_data->pressed = stmpe811_state_pressed;
	return stmpe811_state_pressed;
}
