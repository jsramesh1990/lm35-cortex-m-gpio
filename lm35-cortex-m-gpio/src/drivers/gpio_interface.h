/**
 * GPIO Interface for Cortex-M Boards
 * Hardware abstraction layer for GPIO operations
 */

#ifndef __GPIO_INTERFACE_H
#define __GPIO_INTERFACE_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// GPIO Pin States
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET
} GPIO_PinState;

// GPIO Pull configuration
typedef enum {
    GPIO_NOPULL = 0,
    GPIO_PULLUP,
    GPIO_PULLDOWN
} GPIO_Pull;

// GPIO Speed
typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH
} GPIO_Speed;

// GPIO Modes
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_OUTPUT_OD,
    GPIO_MODE_AF_PP,
    GPIO_MODE_AF_OD,
    GPIO_MODE_ANALOG
} GPIO_Mode;

// GPIO Alternate Functions
typedef enum {
    GPIO_AF0 = 0,
    GPIO_AF1,
    GPIO_AF2,
    GPIO_AF3,
    GPIO_AF4,
    GPIO_AF5,
    GPIO_AF6,
    GPIO_AF7,
    GPIO_AF8,
    GPIO_AF9,
    GPIO_AF10,
    GPIO_AF11,
    GPIO_AF12,
    GPIO_AF13,
    GPIO_AF14,
    GPIO_AF15
} GPIO_AltFunc;

// Pin Definition Structure
typedef struct {
    uint16_t pin;
    GPIO_Mode mode;
    GPIO_Pull pull;
    GPIO_Speed speed;
    GPIO_AltFunc alternate;
} GPIO_PinConfig;

// ADC Result
typedef enum {
    ADC_SUCCESS = 0,
    ADC_ERROR_INIT,
    ADC_ERROR_CONVERSION,
    ADC_ERROR_TIMEOUT,
    ADC_ERROR_INVALID_CHANNEL
} ADC_Result;

// LED Colors for Status
typedef enum {
    LED_OFF = 0,
    LED_GREEN,
    LED_YELLOW,
    LED_RED,
    LED_BLUE
} LED_Color;

// GPIO Ports (for STM32)
typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H
} GPIO_Port;

// Function Prototypes

// GPIO Initialization
void GPIO_InitPin(GPIO_PinConfig* config);
void GPIO_InitAll(void);
void GPIO_DeInit(void);

// GPIO Operations
void GPIO_SetPin(uint16_t pin, GPIO_PinState state);
GPIO_PinState GPIO_GetPin(uint16_t pin);
void GPIO_TogglePin(uint16_t pin);
void GPIO_WritePort(uint32_t value);
uint32_t GPIO_ReadPort(void);

// ADC Functions
bool ADC_Init(uint32_t channel, uint8_t resolution);
ADC_Result ADC_Read(uint32_t* value);
ADC_Result ADC_ReadMultiple(uint32_t* buffer, uint32_t count);
bool ADC_Calibrate(void);
float ADC_ToVoltage(uint32_t adc_value);

// LED Control
void LED_Init(void);
void LED_Set(LED_Color color);
void LED_Blink(LED_Color color, uint32_t duration_ms);
void LED_Pattern(uint8_t pattern);

// UART Functions
void UART_Init(uint32_t baudrate);
void UART_SendString(const char* str);
void UART_SendBytes(const uint8_t* data, uint32_t length);
int UART_ReceiveByte(uint8_t* byte);
void UART_Printf(const char* format, ...);

// System Functions
void System_Delay(uint32_t milliseconds);
void System_DelayMicroseconds(uint32_t microseconds);
uint32_t System_GetTick(void);

// Error Handling
void Error_Handler(const char* message);
void System_Reset(void);

// Power Management
void Enter_LowPowerMode(void);
void Exit_LowPowerMode(void);

// Helper Functions
uint8_t GPIO_GetPortFromPin(uint16_t pin);
uint16_t GPIO_GetPinNumber(uint16_t pin);

#endif // __GPIO_INTERFACE_H
