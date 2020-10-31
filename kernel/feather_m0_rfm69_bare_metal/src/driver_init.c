/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>

#include <hpl_adc_base.h>

/*! The buffer size for USART */
#define UART_M90E26_BUFFER_SIZE 16

struct usart_async_descriptor UART_M90E26;
struct spi_m_sync_descriptor  SPI_WIFI;
struct timer_descriptor       TIMER_0;

static uint8_t UART_M90E26_buffer[UART_M90E26_BUFFER_SIZE];

struct adc_sync_descriptor ADC_TEMPERATURE;

struct i2c_m_sync_desc I2C_ECC508;

struct usart_sync_descriptor UART_DEBUG;

struct calendar_descriptor RTC_CALENDAR;

void ADC_TEMPERATURE_PORT_init(void)
{
}

void ADC_TEMPERATURE_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);
}

void ADC_TEMPERATURE_init(void)
{
	ADC_TEMPERATURE_CLOCK_init();
	ADC_TEMPERATURE_PORT_init();
	adc_sync_init(&ADC_TEMPERATURE, ADC, (void *)NULL);
}

void EIC_WIFI_init(void)
{
	_gclk_enable_channel(EIC_GCLK_ID, CONF_GCLK_EIC_SRC);

	// Set pin direction to input
	gpio_set_pin_direction(CONF_WINC_EXT_INT_PIN, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(CONF_WINC_EXT_INT_PIN,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(CONF_WINC_EXT_INT_PIN, PINMUX_PB09A_EIC_EXTINT9);

	ext_irq_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void UART_M90E26_CLOCK_init()
{

	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void UART_M90E26_PORT_init()
{

	gpio_set_pin_function(USART_M90E26_TX, PINMUX_PA10C_SERCOM0_PAD2);

	gpio_set_pin_function(USART_M90E26_RX, PINMUX_PA11C_SERCOM0_PAD3);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void UART_M90E26_init(void)
{
	UART_M90E26_CLOCK_init();
	usart_async_init(&UART_M90E26, SERCOM0, UART_M90E26_buffer, UART_M90E26_BUFFER_SIZE, (void *)NULL);
	UART_M90E26_PORT_init();
}

void I2C_ECC508_PORT_init(void)
{

	gpio_set_pin_pull_mode(ECC508_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ECC508_SDA, PINMUX_PA08D_SERCOM2_PAD0);

	gpio_set_pin_pull_mode(ECC508_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(ECC508_SCL, PINMUX_PA09D_SERCOM2_PAD1);
}

void I2C_ECC508_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
	_gclk_enable_channel(SERCOM2_GCLK_ID_SLOW, CONF_GCLK_SERCOM2_SLOW_SRC);
}

void I2C_ECC508_init(void)
{
	I2C_ECC508_CLOCK_init();
	i2c_m_sync_init(&I2C_ECC508, SERCOM2);
	I2C_ECC508_PORT_init();
}

void UART_DEBUG_PORT_init(void)
{

	gpio_set_pin_function(USART_DEBUG_TX, PINMUX_PA24C_SERCOM3_PAD2);

	gpio_set_pin_function(USART_DEBUG_RX, PINMUX_PA25C_SERCOM3_PAD3);
}

void UART_DEBUG_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
}

void UART_DEBUG_init(void)
{
	UART_DEBUG_CLOCK_init();
	usart_sync_init(&UART_DEBUG, SERCOM3, (void *)NULL);
	UART_DEBUG_PORT_init();
}

void SPI_WIFI_PORT_init(void)
{

	gpio_set_pin_level(WINC_SPI_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_SPI_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_SPI_MOSI, PINMUX_PA12D_SERCOM4_PAD0);

	gpio_set_pin_level(WINC_SPI_SCK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_SPI_SCK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_SPI_SCK, PINMUX_PA13D_SERCOM4_PAD1);

	// Set pin direction to input
	gpio_set_pin_direction(WINC_SPI_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(WINC_SPI_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(WINC_SPI_MISO, PINMUX_PA15D_SERCOM4_PAD3);
}

void SPI_WIFI_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM4);
	_gclk_enable_channel(SERCOM4_GCLK_ID_CORE, CONF_GCLK_SERCOM4_CORE_SRC);
}

void SPI_WIFI_init(void)
{
	SPI_WIFI_CLOCK_init();
	spi_m_sync_init(&SPI_WIFI, SERCOM4);
	SPI_WIFI_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void RTC_CALENDAR_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
}

void RTC_CALENDAR_init(void)
{
	RTC_CALENDAR_CLOCK_init();
	calendar_init(&RTC_CALENDAR, RTC);
}

/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC3);
	_gclk_enable_channel(TC3_GCLK_ID, CONF_GCLK_TC3_SRC);

	timer_init(&TIMER_0, TC3, _tc_get_timer());
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA14

	gpio_set_pin_level(WINC_SPI_CS_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_SPI_CS_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_SPI_CS_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA17

	gpio_set_pin_level(LED0,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED0, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21

	gpio_set_pin_level(LED_WR_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_WR_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_WR_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA22

	gpio_set_pin_level(LED_WG_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_WG_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_WG_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	gpio_set_pin_level(LED_WY_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_WY_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_WY_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA27

	gpio_set_pin_level(WINC_PIN_RESET,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_PIN_RESET, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_PIN_RESET, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA28

	gpio_set_pin_level(WINC_PIN_CHIP_ENABLE,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_PIN_CHIP_ENABLE, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_PIN_CHIP_ENABLE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB02

	gpio_set_pin_level(LED_PR_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   true);

	// Set pin direction to output
	gpio_set_pin_direction(LED_PR_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_PR_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB03

	gpio_set_pin_level(LED_PG_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(LED_PG_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(LED_PG_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB08

	gpio_set_pin_level(WINC_WAKE_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(WINC_WAKE_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(WINC_WAKE_PIN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22

	gpio_set_pin_level(RELAY_PIN,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(RELAY_PIN, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(RELAY_PIN, GPIO_PIN_FUNCTION_OFF);

	ADC_TEMPERATURE_init();
	EIC_WIFI_init();

	UART_M90E26_init();

	I2C_ECC508_init();

	UART_DEBUG_init();

	SPI_WIFI_init();

	delay_driver_init();

	RTC_CALENDAR_init();

	TIMER_0_init();
}
