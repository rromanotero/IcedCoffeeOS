#include <atmel_start.h>

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/* Replace with your application code */
	while (true) {
		delay_ms(2000);
		gpio_toggle_pin_level(LED);
	}
}
