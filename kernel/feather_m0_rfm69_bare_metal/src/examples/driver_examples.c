/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/**
 * Example of using ADC_TEMPERATURE to generate waveform.
 */
void ADC_TEMPERATURE_example(void)
{
	uint8_t buffer[2];

	adc_sync_enable_channel(&ADC_TEMPERATURE, 0);

	while (1) {
		adc_sync_read_channel(&ADC_TEMPERATURE, 0, buffer, 2);
	}
}

static void button_on_PB09_pressed(void)
{
}

/**
 * Example of using EIC_WIFI
 */
void EIC_WIFI_example(void)
{

	ext_irq_register(PIN_PB09, button_on_PB09_pressed);
}

/**
 * Example of using UART_M90E26 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_UART_M90E26[12] = "Hello World!";

static void tx_cb_UART_M90E26(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void UART_M90E26_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&UART_M90E26, USART_ASYNC_TXC_CB, tx_cb_UART_M90E26);
	/*usart_async_register_callback(&UART_M90E26, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&UART_M90E26, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&UART_M90E26, &io);
	usart_async_enable(&UART_M90E26);

	io_write(io, example_UART_M90E26, 12);
}

void I2C_ECC508_example(void)
{
	struct io_descriptor *I2C_ECC508_io;

	i2c_m_sync_get_io_descriptor(&I2C_ECC508, &I2C_ECC508_io);
	i2c_m_sync_enable(&I2C_ECC508);
	i2c_m_sync_set_slaveaddr(&I2C_ECC508, 0x12, I2C_M_SEVEN);
	io_write(I2C_ECC508_io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using UART_DEBUG to write "Hello World" using the IO abstraction.
 */
void UART_DEBUG_example(void)
{
	struct io_descriptor *io;
	usart_sync_get_io_descriptor(&UART_DEBUG, &io);
	usart_sync_enable(&UART_DEBUG);

	io_write(io, (uint8_t *)"Hello World!", 12);
}

/**
 * Example of using SPI_WIFI to write "Hello World" using the IO abstraction.
 */
static uint8_t example_SPI_WIFI[12] = "Hello World!";

void SPI_WIFI_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPI_WIFI, &io);

	spi_m_sync_enable(&SPI_WIFI);
	io_write(io, example_SPI_WIFI, 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

/**
 * Example of using RTC_CALENDAR.
 */
static struct calendar_alarm alarm;

static void alarm_cb(struct calendar_descriptor *const descr)
{
	/* alarm expired */
}

void RTC_CALENDAR_example(void)
{
	struct calendar_date date;
	struct calendar_time time;

	calendar_enable(&RTC_CALENDAR);

	date.year  = 2000;
	date.month = 12;
	date.day   = 31;

	time.hour = 12;
	time.min  = 59;
	time.sec  = 59;

	calendar_set_date(&RTC_CALENDAR, &date);
	calendar_set_time(&RTC_CALENDAR, &time);

	alarm.cal_alarm.datetime.time.sec = 4;
	alarm.cal_alarm.option            = CALENDAR_ALARM_MATCH_SEC;
	alarm.cal_alarm.mode              = REPEAT;

	calendar_set_alarm(&RTC_CALENDAR, &alarm, alarm_cb);
}

static struct timer_task TIMER_0_task1, TIMER_0_task2;

/**
 * Example of using TIMER_0.
 */
static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}
