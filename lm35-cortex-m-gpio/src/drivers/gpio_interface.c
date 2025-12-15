/**
 * GPIO Interface Implementation for Cortex-M Boards
 */

#include "gpio_interface.h"
#include <stdio.h>
#include <stdarg.h>

// Board-specific includes (these would be for specific HAL)
// For STM32:
// #include "stm32f4xx.h"
// #include "stm32f4xx_hal.h"

// Simulated hardware registers (for compilation without HAL)
static uint32_t simulated_gpio_registers[8] = {0};
static uint32_t simulated_adc_value = 2048; // Simulated ADC reading

// Private function prototypes
static void configure_pin(uint16_t pin, GPIO_Mode mode, GPIO_Pull pull, GPIO_Speed speed);
static uint8_t extract_port(uint16_t pin);
static uint16_t extract_pin(uint16_t pin);

// Initialize GPIO pin
void GPIO_InitPin(GPIO_PinConfig* config) {
    if (config == NULL) return;
    
    configure_pin(config->pin, config->mode, config->pull, config->speed);
    
    // In real implementation, configure alternate function if needed
    if (config->mode == GPIO_MODE_AF_PP || config->mode == GPIO_MODE_AF_OD) {
        // Configure alternate function
    }
}

// Initialize all GPIO pins according to configuration
void GPIO_InitAll(void) {
    // Initialize status LED pin
    GPIO_PinConfig led_config = {
        .pin = STATUS_LED_PIN,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_LOW,
        .alternate = GPIO_AF0
    };
    GPIO_InitPin(&led_config);
    
    // Initialize sensor power pin (if applicable)
    // Note: For LM35, power comes from 3.3V rail, not GPIO
    
    // Initialize UART pins if debug enabled
#if DEBUG_UART_ENABLED
    GPIO_PinConfig uart_tx_config = {
        .pin = DEBUG_UART_TX_PIN,
        .mode = GPIO_MODE_AF_PP,
        .pull = GPIO_PULLUP,
        .speed = GPIO_SPEED_HIGH,
        .alternate = GPIO_AF7  // USART2 for STM32F4
    };
    
    GPIO_PinConfig uart_rx_config = {
        .pin = DEBUG_UART_RX_PIN,
        .mode = GPIO_MODE_AF_PP,
        .pull = GPIO_PULLUP,
        .speed = GPIO_SPEED_HIGH,
        .alternate = GPIO_AF7
    };
    
    GPIO_InitPin(&uart_tx_config);
    GPIO_InitPin(&uart_rx_config);
#endif
}

// Deinitialize GPIO
void GPIO_DeInit(void) {
    // Reset all GPIO pins to default state
    // Implementation depends on specific MCU
}

// Set GPIO pin state
void GPIO_SetPin(uint16_t pin, GPIO_PinState state) {
    uint8_t port = extract_port(pin);
    uint16_t pin_num = extract_pin(pin);
    
    if (port < 8) { // Only A-H ports
        if (state == GPIO_PIN_SET) {
            simulated_gpio_registers[port] |= (1 << pin_num);
        } else {
            simulated_gpio_registers[port] &= ~(1 << pin_num);
        }
    }
}

// Get GPIO pin state
GPIO_PinState GPIO_GetPin(uint16_t pin) {
    uint8_t port = extract_port(pin);
    uint16_t pin_num = extract_pin(pin);
    
    if (port < 8 && (simulated_gpio_registers[port] & (1 << pin_num))) {
        return GPIO_PIN_SET;
    }
    
    return GPIO_PIN_RESET;
}

// Toggle GPIO pin
void GPIO_TogglePin(uint16_t pin) {
    GPIO_PinState current_state = GPIO_GetPin(pin);
    GPIO_SetPin(pin, (current_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// Write to GPIO port
void GPIO_WritePort(uint32_t value) {
    // This would write to a specific port in real implementation
    // For simulation, we'll use port A
    simulated_gpio_registers[GPIO_PORT_A] = value;
}

// Read from GPIO port
uint32_t GPIO_ReadPort(void) {
    // This would read from a specific port in real implementation
    // For simulation, we'll use port A
    return simulated_gpio_registers[GPIO_PORT_A];
}

// Initialize ADC
bool ADC_Init(uint32_t channel, uint8_t resolution) {
    // Validate parameters
    if (resolution != 8 && resolution != 10 && resolution != 12 && resolution != 16) {
        return false;
    }
    
    if (channel >= 16) { // Assuming max 16 channels
        return false;
    }
    
    // Initialize ADC peripheral
    // In real implementation:
    // 1. Enable ADC clock
    // 2. Configure ADC parameters
    // 3. Configure channel
    // 4. Calibrate ADC
    
    // For simulation, just return success
    return true;
}

// Read ADC value
ADC_Result ADC_Read(uint32_t* value) {
    if (value == NULL) {
        return ADC_ERROR_INVALID_CHANNEL;
    }
    
    // Simulate ADC read with some noise
    static uint32_t noise = 0;
    simulated_adc_value = 2048 + (noise % 100) - 50;
    noise++;
    
    *value = simulated_adc_value;
    
    // Simulate conversion time
    for (volatile int i = 0; i < 1000; i++);
    
    return ADC_SUCCESS;
}

// Read multiple ADC samples
ADC_Result ADC_ReadMultiple(uint32_t* buffer, uint32_t count) {
    if (buffer == NULL || count == 0) {
        return ADC_ERROR_INVALID_CHANNEL;
    }
    
    for (uint32_t i = 0; i < count; i++) {
        ADC_Result result = ADC_Read(&buffer[i]);
        if (result != ADC_SUCCESS) {
            return result;
        }
    }
    
    return ADC_SUCCESS;
}

// Calibrate ADC
bool ADC_Calibrate(void) {
    // Simulate calibration
    // In real implementation, this would run the MCU's ADC calibration routine
    
    // Simulate calibration delay
    for (volatile int i = 0; i < 10000; i++);
    
    return true;
}

// Convert ADC value to voltage
float ADC_ToVoltage(uint32_t adc_value) {
    return adc_value * ADC_VOLTAGE_PER_STEP;
}

// Initialize LED
void LED_Init(void) {
    // LED is already initialized in GPIO_InitAll()
}

// Set LED color
void LED_Set(LED_Color color) {
    // For single-color LED, just turn on/off based on color
    switch (color) {
        case LED_OFF:
            GPIO_SetPin(STATUS_LED_PIN, GPIO_PIN_RESET);
            break;
        case LED_GREEN:
        case LED_YELLOW:
        case LED_RED:
        case LED_BLUE:
            GPIO_SetPin(STATUS_LED_PIN, GPIO_PIN_SET);
            break;
    }
}

// Blink LED
void LED_Blink(LED_Color color, uint32_t duration_ms) {
    LED_Set(color);
    System_Delay(duration_ms);
    LED_Set(LED_OFF);
    System_Delay(duration_ms);
}

// LED pattern (for diagnostics)
void LED_Pattern(uint8_t pattern) {
    for (int i = 0; i < 8; i++) {
        if (pattern & (1 << i)) {
            LED_Set(LED_GREEN);
        } else {
            LED_Set(LED_OFF);
        }
        System_Delay(200);
    }
    LED_Set(LED_OFF);
}

// Initialize UART
void UART_Init(uint32_t baudrate) {
#if DEBUG_UART_ENABLED
    // Initialize UART peripheral
    // In real implementation:
    // 1. Enable USART clock
    // 2. Configure baud rate
    // 3. Configure word length, parity, stop bits
    // 4. Enable transmitter/receiver
#endif
}

// Send string via UART
void UART_SendString(const char* str) {
#if DEBUG_UART_ENABLED
    // In real implementation, send each character
    printf("%s", str); // For simulation
#endif
}

// Send bytes via UART
void UART_SendBytes(const uint8_t* data, uint32_t length) {
#if DEBUG_UART_ENABLED
    for (uint32_t i = 0; i < length; i++) {
        // Send each byte
        printf("%c", data[i]); // For simulation
    }
#endif
}

// Receive byte from UART
int UART_ReceiveByte(uint8_t* byte) {
#if DEBUG_UART_ENABLED
    // In real implementation, check UART status and read byte
    // For simulation, return -1 (no data)
    return -1;
#else
    return -1;
#endif
}

// Print formatted string via UART
void UART_Printf(const char* format, ...) {
#if DEBUG_UART_ENABLED
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
#endif
}

// System delay in milliseconds
void System_Delay(uint32_t milliseconds) {
    // Simple busy-wait delay (in real implementation, use systick or timer)
    for (volatile uint32_t i = 0; i < milliseconds * 1000; i++);
}

// System delay in microseconds
void System_DelayMicroseconds(uint32_t microseconds) {
    // Simple busy-wait delay
    for (volatile uint32_t i = 0; i < microseconds; i++);
}

// Get system tick (milliseconds since startup)
uint32_t System_GetTick(void) {
    static uint32_t tick = 0;
    // In real implementation, read from systick timer
    return tick++;
}

// Error handler
void Error_Handler(const char* message) {
    // Blink LED rapidly to indicate error
    while (1) {
        LED_Blink(LED_RED, 100);
        
#if DEBUG_UART_ENABLED
        UART_Printf("ERROR: %s\n", message);
#endif
    }
}

// System reset
void System_Reset(void) {
    // In real implementation, trigger software reset
    while (1); // Hang for simulation
}

// Enter low power mode
void Enter_LowPowerMode(void) {
    // Configure peripherals for low power
    // Enter sleep/stop/standby mode
}

// Exit low power mode
void Exit_LowPowerMode(void) {
    // Restore peripheral configurations
}

// Get port from pin definition
uint8_t GPIO_GetPortFromPin(uint16_t pin) {
    return extract_port(pin);
}

// Get pin number from pin definition
uint16_t GPIO_GetPinNumber(uint16_t pin) {
    return extract_pin(pin);
}

// Private helper functions

static void configure_pin(uint16_t pin, GPIO_Mode mode, GPIO_Pull pull, GPIO_Speed speed) {
    // In real implementation, configure the pin using MCU registers
    // This is a simulation placeholder
    
    uint8_t port = extract_port(pin);
    uint16_t pin_num = extract_pin(pin);
    
    // Record configuration (for simulation)
    // In real MCU, you would set MODER, PUPDR, OSPEEDR registers
}

static uint8_t extract_port(uint16_t pin) {
    // Extract port from pin definition like GPIO_PIN_5
    // For simulation, assume port A for pins 0-15, port B for 16-31, etc.
    return (pin >> 4) & 0x07;
}

static uint16_t extract_pin(uint16_t pin) {
    // Extract pin number from pin definition
    return pin & 0x0F;
}
