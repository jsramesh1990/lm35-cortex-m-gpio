/**
 * LM35 Temperature Sensor Driver Implementation
 */

#include "lm35_driver.h"
#include "gpio_interface.h"
#include <math.h>
#include <string.h>

// Static configuration and state
static LM35_Config lm35_config = {0};
static LM35_State lm35_state = {0};

// Private function prototypes
static float apply_filter(float new_sample);
static bool validate_temperature(float temperature_c);
static float exponential_filter(float new_sample, float old_sample);

// Initialize LM35 sensor
LM35_Error LM35_Init(LM35_Config* config) {
    if (config == NULL) {
        return LM35_ERROR_INVALID_TEMP;
    }
    
    // Copy configuration
    memcpy(&lm35_config, config, sizeof(LM35_Config));
    
    // Initialize ADC
    if (!ADC_Init(lm35_config.adc_channel, lm35_config.adc_resolution)) {
        return LM35_ERROR_ADC_INIT;
    }
    
    // Initialize state
    memset(&lm35_state, 0, sizeof(LM35_State));
    lm35_state.is_calibrated = false;
    lm35_state.status = TEMP_STATUS_NORMAL;
    
    // Initialize temperature history
    for (int i = 0; i < TEMP_SAMPLE_COUNT; i++) {
        lm35_state.temperature_history[i] = ROOM_TEMP_CALIBRATION;
    }
    
    // Enable sensor power
    GPIO_SetPin(GPIO_SENSOR_POWER, GPIO_PIN_SET);
    
    // Small delay for sensor stabilization
    for (volatile int i = 0; i < 10000; i++);
    
    // Run self-test
    if (!LM35_SelfTest()) {
        return LM35_ERROR_ADC_INIT;
    }
    
    return LM35_OK;
}

// Read temperature in Celsius
float LM35_ReadTemperatureC(void) {
    uint32_t raw_adc = 0;
    float temperature_c = 0.0f;
    
    // Read raw ADC value with oversampling
    for (uint16_t i = 0; i < lm35_config.oversampling_factor; i++) {
        uint32_t sample;
        if (!ADC_Read(&sample)) {
            return NAN;
        }
        raw_adc += sample;
    }
    
    raw_adc /= lm35_config.oversampling_factor;
    
    // Convert to temperature
    temperature_c = LM35_ADCToTemperature(raw_adc);
    
    // Validate temperature
    if (!validate_temperature(temperature_c)) {
        return NAN;
    }
    
    // Apply filter
    temperature_c = apply_filter(temperature_c);
    
    // Update state
    lm35_state.current_temp_c = temperature_c;
    lm35_state.current_temp_f = CELSIUS_TO_FAHRENHEIT(temperature_c);
    
    // Update statistics
    if (lm35_state.sample_count == 0) {
        lm35_state.min_temp_c = temperature_c;
        lm35_state.max_temp_c = temperature_c;
        lm35_state.avg_temp_c = temperature_c;
    } else {
        if (temperature_c < lm35_state.min_temp_c) {
            lm35_state.min_temp_c = temperature_c;
        }
        if (temperature_c > lm35_state.max_temp_c) {
            lm35_state.max_temp_c = temperature_c;
        }
        
        // Update moving average
        lm35_state.avg_temp_c = (lm35_state.avg_temp_c * lm35_state.sample_count + temperature_c) /
                               (lm35_state.sample_count + 1);
    }
    
    lm35_state.sample_count++;
    lm35_state.status = LM35_GetTemperatureStatus(temperature_c);
    
    return temperature_c;
}

// Read temperature in Fahrenheit
float LM35_ReadTemperatureF(void) {
    float temp_c = LM35_ReadTemperatureC();
    if (isnan(temp_c)) {
        return NAN;
    }
    return CELSIUS_TO_FAHRENHEIT(temp_c);
}

// Get temperature status
TemperatureStatus LM35_GetTemperatureStatus(float temperature_c) {
    if (isnan(temperature_c)) {
        return TEMP_STATUS_ERROR;
    }
    
    if (temperature_c < TEMP_LOW_THRESHOLD) {
        return TEMP_STATUS_LOW;
    } else if (temperature_c > TEMP_CRITICAL_THRESHOLD) {
        return TEMP_STATUS_CRITICAL;
    } else if (temperature_c > TEMP_HIGH_THRESHOLD) {
        return TEMP_STATUS_HIGH;
    }
    
    return TEMP_STATUS_NORMAL;
}

// Calibrate sensor with known temperature
LM35_Error LM35_Calibrate(float known_temperature_c) {
    if (known_temperature_c < -55.0f || known_temperature_c > 150.0f) {
        return LM35_ERROR_INVALID_TEMP;
    }
    
    // Read current temperature
    float current_temp = LM35_ReadTemperatureC();
    if (isnan(current_temp)) {
        return LM35_ERROR_ADC_READ;
    }
    
    // Calculate calibration parameters
    float error = known_temperature_c - current_temp;
    
    // Simple offset calibration
    lm35_config.calibration_offset += error;
    lm35_state.is_calibrated = true;
    
    // Update configuration
    update_calibration_parameters(lm35_config.calibration_offset, 
                                 lm35_config.calibration_gain);
    
    return LM35_OK;
}

// Update calibration parameters
void LM35_UpdateCalibration(float offset, float gain) {
    lm35_config.calibration_offset = offset;
    lm35_config.calibration_gain = gain;
    
    // In a real implementation, store in EEPROM
}

// Get sensor statistics
void LM35_GetStatistics(float* min, float* max, float* avg) {
    if (min) *min = lm35_state.min_temp_c;
    if (max) *max = lm35_state.max_temp_c;
    if (avg) *avg = lm35_state.avg_temp_c;
}

// Reset sensor statistics
void LM35_ResetStatistics(void) {
    lm35_state.min_temp_c = 0.0f;
    lm35_state.max_temp_c = 0.0f;
    lm35_state.avg_temp_c = 0.0f;
    lm35_state.sample_count = 0;
    
    // Reset history
    for (int i = 0; i < TEMP_SAMPLE_COUNT; i++) {
        lm35_state.temperature_history[i] = ROOM_TEMP_CALIBRATION;
    }
    lm35_state.history_index = 0;
}

// Enable sensor
void LM35_Enable(void) {
    GPIO_SetPin(GPIO_SENSOR_POWER, GPIO_PIN_SET);
}

// Disable sensor
void LM35_Disable(void) {
    GPIO_SetPin(GPIO_SENSOR_POWER, GPIO_PIN_RESET);
}

// Check if sensor is ready
bool LM35_IsReady(void) {
    return (lm35_state.sample_count > 0);
}

// Set filter parameters
void LM35_SetFilter(uint8_t filter_type, float alpha) {
    lm35_config.filter_type = filter_type;
    lm35_config.filter_alpha = alpha;
    
    // Clamp alpha to valid range
    if (alpha < 0.0f) lm35_config.filter_alpha = 0.0f;
    if (alpha > 1.0f) lm35_config.filter_alpha = 1.0f;
}

// Set oversampling factor
void LM35_SetOversampling(uint16_t factor) {
    if (factor > 0 && factor <= 256) {
        lm35_config.oversampling_factor = factor;
    }
}

// Get raw ADC value
uint32_t LM35_GetRawADC(void) {
    uint32_t adc_value = 0;
    ADC_Read(&adc_value);
    return adc_value;
}

// Convert ADC to temperature
float LM35_ADCToTemperature(uint32_t adc_value) {
    // Convert ADC to millivolts
    float voltage_mv = ADC_TO_MILLIVOLTS(adc_value);
    
    // Apply calibration
    float temperature_c = (voltage_mv / LM35_SENSITIVITY_MV_C) * 
                         lm35_config.calibration_gain + 
                         lm35_config.calibration_offset;
    
    return temperature_c;
}

// Self-test function
bool LM35_SelfTest(void) {
    // Read ADC to check if sensor is connected
    uint32_t adc_value;
    if (!ADC_Read(&adc_value)) {
        return false;
    }
    
    // Check for reasonable values (not stuck at min/max)
    if (adc_value == 0 || adc_value == ADC_MAX_VALUE) {
        return false;
    }
    
    // Read temperature and check range
    float temp = LM35_ReadTemperatureC();
    if (isnan(temp) || temp < -60.0f || temp > 160.0f) {
        return false;
    }
    
    return true;
}

// Private functions

static float apply_filter(float new_sample) {
    float filtered_sample = new_sample;
    
    switch (lm35_config.filter_type) {
        case FILTER_MOVING_AVERAGE:
            // Update circular buffer
            lm35_state.temperature_history[lm35_state.history_index] = new_sample;
            lm35_state.history_index = (lm35_state.history_index + 1) % TEMP_SAMPLE_COUNT;
            
            // Calculate moving average
            filtered_sample = 0.0f;
            for (int i = 0; i < TEMP_SAMPLE_COUNT; i++) {
                filtered_sample += lm35_state.temperature_history[i];
            }
            filtered_sample /= TEMP_SAMPLE_COUNT;
            break;
            
        case FILTER_EXPONENTIAL:
            filtered_sample = exponential_filter(new_sample, lm35_state.current_temp_c);
            break;
            
        case FILTER_MEDIAN:
            // Simple median filter with 3 samples
            // In real implementation, use proper median filter
            lm35_state.temperature_history[lm35_state.history_index] = new_sample;
            lm35_state.history_index = (lm35_state.history_index + 1) % 3;
            
            // Sort three samples and take middle
            float samples[3] = {
                lm35_state.temperature_history[0],
                lm35_state.temperature_history[1],
                lm35_state.temperature_history[2]
            };
            
            // Simple bubble sort for 3 elements
            if (samples[0] > samples[1]) {
                float temp = samples[0];
                samples[0] = samples[1];
                samples[1] = temp;
            }
            if (samples[1] > samples[2]) {
                float temp = samples[1];
                samples[1] = samples[2];
                samples[2] = temp;
            }
            if (samples[0] > samples[1]) {
                float temp = samples[0];
                samples[0] = samples[1];
                samples[1] = temp;
            }
            
            filtered_sample = samples[1];
            break;
            
        case FILTER_NONE:
        default:
            // No filtering
            break;
    }
    
    return filtered_sample;
}

static bool validate_temperature(float temperature_c) {
    // Check for NaN
    if (isnan(temperature_c)) {
        return false;
    }
    
    // Check for reasonable temperature range
    if (temperature_c < -60.0f || temperature_c > 160.0f) {
        return false;
    }
    
    // Check for sudden jumps (more than 10°C change)
    if (lm35_state.sample_count > 0) {
        float delta = fabsf(temperature_c - lm35_state.current_temp_c);
        if (delta > 10.0f) {
            return false;
        }
    }
    
    return true;
}

static float exponential_filter(float new_sample, float old_sample) {
    return lm35_config.filter_alpha * new_sample + 
           (1.0f - lm35_config.filter_alpha) * old_sample;
}
