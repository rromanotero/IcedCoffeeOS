/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define ECC508_SDA GPIO(GPIO_PORTA, 8)
#define ECC508_SCL GPIO(GPIO_PORTA, 9)
#define USART_M90E26_TX GPIO(GPIO_PORTA, 10)
#define USART_M90E26_RX GPIO(GPIO_PORTA, 11)
#define WINC_SPI_MOSI GPIO(GPIO_PORTA, 12)
#define WINC_SPI_SCK GPIO(GPIO_PORTA, 13)
#define WINC_SPI_CS_PIN GPIO(GPIO_PORTA, 14)
#define WINC_SPI_MISO GPIO(GPIO_PORTA, 15)
#define LED0 GPIO(GPIO_PORTA, 17)
#define LED_WR_PIN GPIO(GPIO_PORTA, 21)
#define LED_WG_PIN GPIO(GPIO_PORTA, 22)
#define LED_WY_PIN GPIO(GPIO_PORTA, 23)
#define USART_DEBUG_TX GPIO(GPIO_PORTA, 24)
#define USART_DEBUG_RX GPIO(GPIO_PORTA, 25)
#define WINC_PIN_RESET GPIO(GPIO_PORTA, 27)
#define WINC_PIN_CHIP_ENABLE GPIO(GPIO_PORTA, 28)
#define LED_PR_PIN GPIO(GPIO_PORTB, 2)
#define LED_PG_PIN GPIO(GPIO_PORTB, 3)
#define WINC_WAKE_PIN GPIO(GPIO_PORTB, 8)
#define CONF_WINC_EXT_INT_PIN GPIO(GPIO_PORTB, 9)
#define RELAY_PIN GPIO(GPIO_PORTB, 22)

#endif // ATMEL_START_PINS_H_INCLUDED
