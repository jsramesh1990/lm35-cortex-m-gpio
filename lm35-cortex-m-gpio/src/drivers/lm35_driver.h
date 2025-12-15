/**
 * LM35 Temperature Sensor Driver
 * Interface for LM35 temperature sensor via ADC
 */

#ifndef __LM35_DRIVER_H
#define __LM35_DRIVER_H

#include "config.h"
#include <stdint.h>
#include <stdbool.h>

// Error codes
typedef enum {
    LM35_OK = 0,
    LM35_ERROR_ADC_INIT,
    LM35_ERROR_ADC_READ,
    LM35_ERROR_CALIBRATION,
    LM35_ERROR_INVALID_TEMP
} LM35_Error;

// Sensor configuration
typedef struct {
    uint32_t adc_channel;
    uint8_t adc_resolution;
    float reference_voltage;
    float calibration_offset;
    float calibration_gain;
    uint16_t oversampling_factor;
    uint8_t filter_type;
    float filter_alpha;
} LM35_Config;

// Sensor state
typedef struct {
    float current_temp_c;
    float current_temp_f;
    float min_temp_c;
    float max_temp_c;
    float avg_temp_c;
    uint32_t sample_count;
    TemperatureStatus status;
    bool is_calibrated;
    float temperature_history[TEMP_SAMPLE_COUNT];
    uint8_t history_index;
} LM35_State;

// Initialize LM35 sensor
LM35_Error LM35_Init(LM35_Config* config);

// Read temperature in Celsius
float LM35_ReadTemperatureC(void);

// Read temperature in Fahrenheit
float LM35_ReadTemperatureF(void);

// Get temperature status
TemperatureStatus LM35_GetTemperatureStatus(float temperature_c);

// Calibrate sensor with known temperature
LM35_Error LM35_Calibrate(float known_temperature_c);

// Update calibration parameters
void LM35_UpdateCalibration(float offset, float gain);

// Get sensor statistics
void LM35_GetStatistics(float* min, float* max, float* avg);

// Reset sensor statistics
void LM35_ResetStatistics(void);

// Enable/disable sensor
void LM35_Enable(void);
void LM35_Disable(void);

// Check if sensor is ready
bool LM35_IsReady(void);

// Set filter parameters
void LM35_SetFilter(uint8_t filter_type, float alpha);

// Set oversampling factor
void LM35_SetOversampling(uint16_t factor);

// Get raw ADC value
uint32_t LM35_GetRawADC(void);

// Convert ADC to temperature
float LM35_ADCToTemperature(uint32_t adc_value);

// Self-test function
bool LM35_SelfTest(void);

#endif // __LM35_DRIVER_H
