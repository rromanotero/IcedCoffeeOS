#include <stdbool.h>
#include <stdint.h>

#define ARDUINO_MAIN  loop
void setup() {} /* Don't need it*/

#define HAL_SUCCESS  0
#define HAL_FAILED  1

#define HAL_IO_SERIAL_CHAR_NOT_READY       0
#define HAL_IO_SERIAL_CHAR_READY           1

#define HAL_IO_TYPE_NOT_FOUND					100
#define HAL_IO_PIO_PORT_NOT_FOUND     101
#define HAL_IO_PIO_PIN_NOT_FOUND      102
#define HAL_IO_SERIAL_PORT_NOT_FOUND	103

typedef uint32_t tPwmType;		/**< PWM Pin Type */
typedef uint32_t tFaultOrigin;	/**< Fault Origin type */
typedef uint32_t tSensorId;		/**< Sensor ID type */
typedef enum tIoType			 { IoPoll = 0, IoInterrupt  };
typedef enum tPioPort			 { PioA = 0, PioB, PioC, PioD };
typedef enum tPioDir		   { PioOutput = 0, PioInput } ;
typedef enum tSerialId     { SerialA = 0, SerialB, SerialC, SerialD  };
typedef enum tTimerId      { TimerSysTick = 0, TimerMicroseconds = 1, };

typedef struct{
	uint32_t		pin_number;
	tPioPort		pio_port;
	uint32_t		internal_rep; /* How the pin is represented internally (this is hardware specific) */
}tPioPin;

typedef struct{
	tSerialId		id;
	tIoType			io_type;
	uint32_t		baudrate;
	uint32_t		internal_rep; /* How the pin is represented internally (this is hardware specific) */
}tSerialPort;

typedef struct{
	tPioPin* io_pin;
	uint32_t period;
	uint32_t duty_cycle;
}tPwmChannel;

typedef struct{
	tPioPin* io_pin;
	uint32_t adc_channel_number;
}tAdcChannel;

typedef struct{
	uint32_t seconds; /**< seconds */
	uint32_t minutes; /**< minutes */
	uint32_t hours;   /**< hours */
	uint32_t day;     /**< day */
	uint32_t month;   /**< month */
	uint32_t year;    /**< year */
}tTime;

typedef struct{
    uint8_t* name;
    uint32_t speed_mhz;
    uint32_t num_cores;
}CPUInfo;
