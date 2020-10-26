/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_sync.h>
#include <hal_ext_irq.h>

#include <hal_usart_async.h>

#include <hal_i2c_m_sync.h>

#include <hal_usart_sync.h>
#include <hal_spi_m_sync.h>

#include <hal_delay.h>

#include <hal_calendar.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>

extern struct adc_sync_descriptor    ADC_TEMPERATURE;
extern struct usart_async_descriptor UART_M90E26;

extern struct i2c_m_sync_desc I2C_ECC508;

extern struct usart_sync_descriptor UART_DEBUG;
extern struct spi_m_sync_descriptor SPI_WIFI;

extern struct calendar_descriptor RTC_CALENDAR;
extern struct timer_descriptor    TIMER_0;

void ADC_TEMPERATURE_PORT_init(void);
void ADC_TEMPERATURE_CLOCK_init(void);
void ADC_TEMPERATURE_init(void);

void UART_M90E26_PORT_init(void);
void UART_M90E26_CLOCK_init(void);
void UART_M90E26_init(void);

void I2C_ECC508_CLOCK_init(void);
void I2C_ECC508_init(void);
void I2C_ECC508_PORT_init(void);

void UART_DEBUG_PORT_init(void);
void UART_DEBUG_CLOCK_init(void);
void UART_DEBUG_init(void);

void SPI_WIFI_PORT_init(void);
void SPI_WIFI_CLOCK_init(void);
void SPI_WIFI_init(void);

void delay_driver_init(void);

void RTC_CALENDAR_CLOCK_init(void);
void RTC_CALENDAR_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_INCLUDED
