# stm32f429i-disco-stmpe811

This is a driver for the STMPE811 touchscreen driver on
the STM32F429i-Discovery board.
It uses the open source libopencm3 library.
(https://github.com/libopencm3/libopencm3)

Basically just include stmpe811.h and stmpe811.c in your project,
then use as follows:


if (stmpe811_init() != stmpe811_state_ok) {
	console_puts("STMPE811 Error!");
}

/* can select an orientation */
stmpe811_data.orientation = stmpe811_portrait_2;

console_puts("Press on touchscreen please!\n\n");

while (1) {
	if (stmpe811_read_touch(&stmpe811_data) == stmpe811_state_pressed) {
		do_pressed_action;
	} else {
		do_unpressed_action;
	}
}


On state pressed one can use stmpe811_read_x and stmpe811_read_y to get
the x-y coordinates. These will not be correct, one must calibrate
the touchscreen for meaningful results.

the console_puts is from Chuck McMannis' console
(https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f4/stm32f429i-discovery/lcd-serial/console.c)

