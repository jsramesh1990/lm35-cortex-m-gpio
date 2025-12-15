/**
 * Main Application for LM35 Temperature Sensor
 * Cortex-M GPIO Interface Project
 */

#include "config.h"
#include "lm35_driver.h"
#include "gpio_interface.h"
#include <stdio.h>
#include <string.h>

// Global variables
static LM35_Config lm35_config;
static volatile bool temperature_alert = false;
static volatile float last_temperature_c = 0.0f;

// Function prototypes
static void initialize_system(void);
static void configure_lm35_sensor(void);
static void temperature_monitor_loop(void);
static void handle_temperature_alert(float temperature_c);
static void print_system_info(void);
static void print_temperature_report(float temperature_c);

// Main application entry point
int main(void) {
    // Initialize system
    initialize_system();
    
    // Configure LM35 sensor
    configure_lm35_sensor();
    
    // Print system information
    print_system_info();
    
    // Main application loop
    while (1) {
        temperature_monitor_loop();
        
        // Check for temperature alerts
        if (temperature_alert) {
            handle_temperature_alert(last_temperature_c);
            temperature_alert = false;
        }
        
        // Small delay to prevent busy-waiting
        System_Delay(100);
    }
    
    return 0;
}

// Initialize system components
static void initialize_system(void) {
    // Initialize GPIO and peripherals
    GPIO_InitAll();
    
    // Initialize UART for debug output
#if DEBUG_UART_ENABLED
    UART_Init(DEBUG_UART_BAUDRATE);
    UART_SendString("\r\n=== LM35 Temperature Monitor ===\r\n");
#endif
    
    // Initialize LED for status indication
    LED_Init();
    
    // Blink LED to indicate startup
    LED_Blink(LED_GREEN, 200);
    LED_Blink(LED_GREEN, 200);
    LED_Blink(LED_GREEN, 200);
    
    // Print startup message
    UART_Printf("System initialized successfully.\r\n");
    UART_Printf("Board: %s\r\n", BOARD_TYPE);
    UART_Printf("CPU: %s @ %lu Hz\r\n", BOARD_CPU, BOARD_CLOCK_HZ);
    UART_Printf("Firmware: %s\r\n", FIRMWARE_VERSION);
}

// Configure LM35 sensor
static void configure_lm35_sensor(void) {
    // Configure LM35 sensor
    memset(&lm35_config, 0, sizeof(LM35_Config));
    
    lm35_config.adc_channel = LM35_ADC_CHANNEL;
    lm35_config.adc_resolution = ADC_RESOLUTION_BITS;
    lm35_config.reference_voltage = ADC_REF_VOLTAGE;
    lm35_config.calibration_offset = CALIBRATION_OFFSET;
    lm35_config.calibration_gain = CALIBRATION_GAIN;
    lm35_config.oversampling_factor = OVERSAMPLING_FACTOR;
    lm35_config.filter_type = FILTER_EXPONENTIAL;
    lm35_config.filter_alpha = FILTER_ALPHA;
    
    // Initialize LM35 driver
    LM35_Error error = LM35_Init(&lm35_config);
    if (error != LM35_OK) {
        UART_Printf("ERROR: Failed to initialize LM35 sensor (code: %d)\r\n", error);
        Error_Handler("LM35 init failed");
    }
    
    UART_Printf("LM35 sensor initialized successfully.\r\n");
    UART_Printf("ADC Channel: %lu\r\n", lm35_config.adc_channel);
    UART_Printf("ADC Resolution: %u bits\r\n", lm35_config.adc_resolution);
    UART_Printf("Oversampling: %u\r\n", lm35_config.oversampling_factor);
    
    // Perform self-test
    if (LM35_SelfTest()) {
        UART_Printf("LM35 self-test: PASSED\r\n");
        LED_Set(LED_GREEN);
    } else {
        UART_Printf("LM35 self-test: FAILED\r\n");
        LED_Set(LED_RED);
        Error_Handler("LM35 self-test failed");
    }
    
    // If auto-calibration is enabled, calibrate at room temperature
#if AUTO_CALIBRATION
    UART_Printf("Auto-calibration enabled, calibrating at %.1f°C...\r\n", 
                ROOM_TEMP_CALIBRATION);
    
    error = LM35_Calibrate(ROOM_TEMP_CALIBRATION);
    if (error == LM35_OK) {
        UART_Printf("Calibration successful.\r\n");
    } else {
        UART_Printf("Calibration failed (code: %d)\r\n", error);
    }
#endif
}

// Main temperature monitoring loop
static void temperature_monitor_loop(void) {
    static uint32_t last_update = 0;
    uint32_t current_time = System_GetTick();
    
    // Update temperature at configured interval
    if (current_time - last_update >= TEMP_UPDATE_RATE_MS) {
        // Read temperature
        float temperature_c = LM35_ReadTemperatureC();
        
        if (!isnan(temperature_c)) {
            last_temperature_c = temperature_c;
            
            // Print temperature report
            print_temperature_report(temperature_c);
            
            // Update LED based on temperature status
            TemperatureStatus status = LM35_GetTemperatureStatus(temperature_c);
            switch (status) {
                case TEMP_STATUS_NORMAL:
                    LED_Set(LED_GREEN);
                    break;
                case TEMP_STATUS_LOW:
                    LED_Set(LED_BLUE);
                    temperature_alert = true;
                    break;
                case TEMP_STATUS_HIGH:
                    LED_Set(LED_YELLOW);
                    temperature_alert = true;
                    break;
                case TEMP_STATUS_CRITICAL:
                    LED_Set(LED_RED);
                    temperature_alert = true;
                    break;
                case TEMP_STATUS_ERROR:
                    LED_Set(LED_OFF);
                    break;
            }
            
            // Get statistics
            float min_temp, max_temp, avg_temp;
            LM35_GetStatistics(&min_temp, &max_temp, &avg_temp);
            
            // Periodically print statistics
            static uint32_t stat_counter = 0;
            if (++stat_counter >= 10) { // Every 10 readings
                stat_counter = 0;
                UART_Printf("Stats: Min=%.2f°C, Max=%.2f°C, Avg=%.2f°C\r\n",
                           min_temp, max_temp, avg_temp);
            }
        } else {
            UART_Printf("ERROR: Failed to read temperature\r\n");
            LED_Blink(LED_RED, 500);
        }
        
        last_update = current_time;
    }
}

// Handle temperature alerts
static void handle_temperature_alert(float temperature_c) {
    TemperatureStatus status = LM35_GetTemperatureStatus(temperature_c);
    
    UART_Printf("\r\n!!! TEMPERATURE ALERT !!!\r\n");
    UART_Printf("Temperature: %.2f°C (%.2f°F)\r\n", 
                temperature_c, CELSIUS_TO_FAHRENHEIT(temperature_c));
    
    switch (status) {
        case TEMP_STATUS_LOW:
            UART_Printf("Status: LOW TEMPERATURE\r\n");
            break;
        case TEMP_STATUS_HIGH:
            UART_Printf("Status: HIGH TEMPERATURE\r\n");
            break;
        case TEMP_STATUS_CRITICAL:
            UART_Printf("Status: CRITICAL TEMPERATURE\r\n");
            // In a real system, you might trigger safety measures here
            break;
        default:
            break;
    }
    
    UART_Printf("Thresholds: Low=%.1f°C, High=%.1f°C, Critical=%.1f°C\r\n",
                TEMP_LOW_THRESHOLD, TEMP_HIGH_THRESHOLD, TEMP_CRITICAL_THRESHOLD);
    
    // Blink LED to indicate alert
    for (int i = 0; i < 5; i++) {
        LED_Set(LED_RED);
        System_Delay(200);
        LED_Set(LED_OFF);
        System_Delay(200);
    }
}

// Print system information
static void print_system_info(void) {
    UART_Printf("\r\n=== System Configuration ===\r\n");
    UART_Printf("Board: %s\r\n", BOARD_TYPE);
    UART_Printf("CPU: %s\r\n", BOARD_CPU);
    UART_Printf("Clock: %lu Hz\r\n", BOARD_CLOCK_HZ);
    UART_Printf("Firmware: %s\r\n", FIRMWARE_VERSION);
    UART_Printf("ADC Pin: %d\r\n", LM35_ADC_PIN);
    UART_Printf("ADC Channel: %d\r\n", LM35_ADC_CHANNEL);
    UART_Printf("ADC Resolution: %d bits\r\n", ADC_RESOLUTION_BITS);
    UART_Printf("Reference Voltage: %.2f V\r\n", ADC_REF_VOLTAGE);
    UART_Printf("Sample Rate: %d ms\r\n", TEMP_UPDATE_RATE_MS);
    UART_Printf("Temperature Thresholds:\r\n");
    UART_Printf("  Low: %.1f°C\r\n", TEMP_LOW_THRESHOLD);
    UART_Printf("  High: %.1f°C\r\n", TEMP_HIGH_THRESHOLD);
    UART_Printf("  Critical: %.1f°C\r\n", TEMP_CRITICAL_THRESHOLD);
    UART_Printf("=============================\r\n\r\n");
}

// Print temperature report
static void print_temperature_report(float temperature_c) {
    static uint32_t report_count = 0;
    float temperature_f = CELSIUS_TO_FAHRENHEIT(temperature_c);
    
    // Get raw ADC value for debugging
    uint32_t raw_adc = LM35_GetRawADC();
    float voltage_mv = ADC_TO_MILLIVOLTS(raw_adc);
    
    UART_Printf("[%04lu] Temp: %6.2f°C (%6.2f°F) | ", 
                report_count++, temperature_c, temperature_f);
    UART_Printf("ADC: %4lu (%.1f mV) | ", raw_adc, voltage_mv);
    
    // Print temperature status
    TemperatureStatus status = LM35_GetTemperatureStatus(temperature_c);
    switch (status) {
        case TEMP_STATUS_NORMAL:
            UART_Printf("Status: NORMAL");
            break;
        case TEMP_STATUS_LOW:
            UART_Printf("Status: LOW");
            break;
        case TEMP_STATUS_HIGH:
            UART_Printf("Status: HIGH");
            break;
        case TEMP_STATUS_CRITICAL:
            UART_Printf("Status: CRITICAL");
            break;
        case TEMP_STATUS_ERROR:
            UART_Printf("Status: ERROR");
            break;
    }
    
    UART_Printf("\r\n");
    
    // Update console cursor (simple progress indicator)
    static uint8_t spinner = 0;
    const char* spinner_chars = "|/-\\";
    UART_Printf("\r%c", spinner_chars[spinner++ & 3]);
}

// Weak function definitions for HAL callbacks (if using HAL)
__attribute__((weak)) void SysTick_Handler(void) {
    // Systick handler - can be overridden by user
}

__attribute__((weak)) void ADC_IRQHandler(void) {
    // ADC interrupt handler - can be overridden by user
}
