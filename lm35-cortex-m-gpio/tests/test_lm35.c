/**
 * LM35 Driver Test Suite
 */

#include "config.h"
#include "lm35_driver.h"
#include "gpio_interface.h"
#include <stdio.h>
#include <math.h>

// Test definitions
typedef struct {
    const char* name;
    bool (*test_func)(void);
    bool required;
} TestCase;

// Test function prototypes
static bool test_lm35_init(void);
static bool test_lm35_read_temperature(void);
static bool test_lm35_calibration(void);
static bool test_lm35_filtering(void);
static bool test_lm35_statistics(void);
static bool test_lm35_error_handling(void);
static bool test_adc_conversion(void);
static bool test_temperature_thresholds(void);

// Test cases
static TestCase test_cases[] = {
    {"LM35 Initialization", test_lm35_init, true},
    {"ADC Conversion", test_adc_conversion, true},
    {"Temperature Reading", test_lm35_read_temperature, true},
    {"Temperature Thresholds", test_temperature_thresholds, true},
    {"Calibration", test_lm35_calibration, false},
    {"Filtering", test_lm35_filtering, false},
    {"Statistics", test_lm35_statistics, false},
    {"Error Handling", test_lm35_error_handling, false},
    {NULL, NULL, false}
};

// Global variables for test
static LM35_Config test_config;
static uint32_t test_count = 0;
static uint32_t pass_count = 0;
static uint32_t fail_count = 0;

// Test runner
void run_all_tests(void) {
    printf("\n=== LM35 Driver Test Suite ===\n");
    printf("Board: %s\n", BOARD_TYPE);
    printf("ADC Resolution: %d bits\n", ADC_RESOLUTION_BITS);
    printf("ADC Reference: %.2f V\n", ADC_REF_VOLTAGE);
    printf("\n");
    
    // Initialize GPIO and ADC for testing
    GPIO_InitAll();
    
    // Configure LM35 for testing
    memset(&test_config, 0, sizeof(LM35_Config));
    test_config.adc_channel = LM35_ADC_CHANNEL;
    test_config.adc_resolution = ADC_RESOLUTION_BITS;
    test_config.reference_voltage = ADC_REF_VOLTAGE;
    test_config.calibration_offset = CALIBRATION_OFFSET;
    test_config.calibration_gain = CALIBRATION_GAIN;
    test_config.oversampling_factor = OVERSAMPLING_FACTOR;
    test_config.filter_type = FILTER_NONE;
    test_config.filter_alpha = FILTER_ALPHA;
    
    // Run all tests
    for (int i = 0; test_cases[i].name != NULL; i++) {
        printf("Test %d: %-30s ", i + 1, test_cases[i].name);
        
        if (!test_cases[i].required) {
            printf("[SKIP - Optional]\n");
            continue;
        }
        
        test_count++;
        
        // Run test
        bool result = test_cases[i].test_func();
        
        if (result) {
            printf("[PASS]\n");
            pass_count++;
        } else {
            printf("[FAIL]\n");
            fail_count++;
        }
    }
    
    // Print summary
    printf("\n=== Test Summary ===\n");
    printf("Tests Run:    %d\n", test_count);
    printf("Tests Passed: %d\n", pass_count);
    printf("Tests Failed: %d\n", fail_count);
    
    if (fail_count == 0) {
        printf("\n✅ All tests passed!\n");
    } else {
        printf("\n❌ Some tests failed!\n");
    }
}

// Test: LM35 Initialization
static bool test_lm35_init(void) {
    LM35_Error error = LM35_Init(&test_config);
    
    if (error != LM35_OK) {
        printf("Init failed with error: %d\n", error);
        return false;
    }
    
    // Check if sensor is ready
    if (!LM35_IsReady()) {
        printf("Sensor not ready after init\n");
        return false;
    }
    
    return true;
}

// Test: ADC Conversion
static bool test_adc_conversion(void) {
    // Read raw ADC value
    uint32_t raw_adc = LM35_GetRawADC();
    
    // Check if ADC value is within reasonable range
    if (raw_adc > ADC_MAX_VALUE) {
        printf("ADC value out of range: %lu > %lu\n", raw_adc, ADC_MAX_VALUE);
        return false;
    }
    
    // Convert to voltage
    float voltage = ADC_TO_VOLTAGE(raw_adc);
    if (voltage < 0 || voltage > ADC_REF_VOLTAGE) {
        printf("Voltage out of range: %.3f V\n", voltage);
        return false;
    }
    
    // Convert to temperature
    float temp = LM35_ADCToTemperature(raw_adc);
    if (isnan(temp)) {
        printf("Temperature conversion returned NaN\n");
        return false;
    }
    
    printf("ADC: %lu (%.3f V) -> %.2f°C ", raw_adc, voltage, temp);
    return true;
}

// Test: Temperature Reading
static bool test_lm35_read_temperature(void) {
    // Read temperature multiple times
    const int num_readings = 5;
    float readings[num_readings];
    
    for (int i = 0; i < num_readings; i++) {
        readings[i] = LM35_ReadTemperatureC();
        
        if (isnan(readings[i])) {
            printf("Reading %d: NaN\n", i);
            return false;
        }
        
        // Check for reasonable temperature range
        if (readings[i] < -60.0f || readings[i] > 160.0f) {
            printf("Reading %d out of range: %.2f°C\n", i, readings[i]);
            return false;
        }
        
        System_Delay(100);
    }
    
    // Check consistency (readings should be similar)
    float avg = 0;
    for (int i = 0; i < num_readings; i++) {
        avg += readings[i];
    }
    avg /= num_readings;
    
    // Check that all readings are within 5°C of average
    for (int i = 0; i < num_readings; i++) {
        if (fabsf(readings[i] - avg) > 5.0f) {
            printf("Inconsistent reading %d: %.2f°C (avg: %.2f°C)\n", 
                   i, readings[i], avg);
            return false;
        }
    }
    
    printf("Avg: %.2f°C (range: %.2f-%.2f) ", 
           avg, readings[0], readings[num_readings-1]);
    return true;
}

// Test: Temperature Thresholds
static bool test_temperature_thresholds(void) {
    // Test with various temperatures
    struct {
        float temp;
        TemperatureStatus expected;
    } test_points[] = {
        {-10.0f, TEMP_STATUS_LOW},
        {10.0f, TEMP_STATUS_NORMAL},
        {60.0f, TEMP_STATUS_HIGH},
        {90.0f, TEMP_STATUS_CRITICAL},
        {NAN, TEMP_STATUS_ERROR}
    };
    
    for (int i = 0; i < sizeof(test_points)/sizeof(test_points[0]); i++) {
        TemperatureStatus status = LM35_GetTemperatureStatus(test_points[i].temp);
        
        if (status != test_points[i].expected) {
            printf("Threshold test %d failed: got %d, expected %d\n", 
                   i, status, test_points[i].expected);
            return false;
        }
    }
    
    return true;
}

// Test: Calibration
static bool test_lm35_calibration(void) {
    // Save original calibration
    float original_offset = test_config.calibration_offset;
    float original_gain = test_config.calibration_gain;
    
    // Test calibration with known offset
    float known_temp = 25.0f;
    LM35_Error error = LM35_Calibrate(known_temp);
    
    if (error != LM35_OK) {
        printf("Calibration failed with error: %d\n", error);
        return false;
    }
    
    // Read temperature after calibration
    float temp_after = LM35_ReadTemperatureC();
    
    // Check if calibration improved accuracy
    if (fabsf(temp_after - known_temp) > 2.0f) {
        printf("Calibration not effective: %.2f°C vs %.2f°C\n", 
               temp_after, known_temp);
        // Restore original calibration
        LM35_UpdateCalibration(original_offset, original_gain);
        return false;
    }
    
    printf("Calibration: %.2f°C -> %.2f°C ", known_temp, temp_after);
    
    // Restore original calibration
    LM35_UpdateCalibration(original_offset, original_gain);
    return true;
}

// Test: Filtering
static bool test_lm35_filtering(void) {
    // Test different filter types
    const int num_filters = 3;
    uint8_t filter_types[] = {FILTER_NONE, FILTER_MOVING_AVERAGE, FILTER_EXPONENTIAL};
    const char* filter_names[] = {"None", "Moving Avg", "Exponential"};
    
    for (int f = 0; f < num_filters; f++) {
        LM35_SetFilter(filter_types[f], 0.1f);
        
        // Take multiple readings
        const int num_readings = 10;
        float readings[num_readings];
        
        for (int i = 0; i < num_readings; i++) {
            readings[i] = LM35_ReadTemperatureC();
            System_Delay(50);
        }
        
        // Check that readings are valid
        for (int i = 0; i < num_readings; i++) {
            if (isnan(readings[i])) {
                printf("Filter %s: NaN at reading %d\n", filter_names[f], i);
                return false;
            }
        }
    }
    
    // Restore default filter
    LM35_SetFilter(FILTER_EXPONENTIAL, FILTER_ALPHA);
    return true;
}

// Test: Statistics
static bool test_lm35_statistics(void) {
    // Reset statistics
    LM35_ResetStatistics();
    
    // Take multiple readings
    const int num_readings = 10;
    float expected_min = 100.0f;
    float expected_max = -100.0f;
    float sum = 0;
    
    for (int i = 0; i < num_readings; i++) {
        float temp = LM35_ReadTemperatureC();
        
        if (temp < expected_min) expected_min = temp;
        if (temp > expected_max) expected_max = temp;
        sum += temp;
        
        System_Delay(50);
    }
    
    float expected_avg = sum / num_readings;
    
    // Get statistics from driver
    float actual_min, actual_max, actual_avg;
    LM35_GetStatistics(&actual_min, &actual_max, &actual_avg);
    
    // Check statistics
    float tolerance = 0.1f;
    if (fabsf(actual_min - expected_min) > tolerance ||
        fabsf(actual_max - expected_max) > tolerance ||
        fabsf(actual_avg - expected_avg) > tolerance) {
        printf("Statistics mismatch:\n");
        printf("  Expected: Min=%.2f, Max=%.2f, Avg=%.2f\n", 
               expected_min, expected_max, expected_avg);
        printf("  Actual:   Min=%.2f, Max=%.2f, Avg=%.2f\n", 
               actual_min, actual_max, actual_avg);
        return false;
    }
    
    printf("Stats: Min=%.2f, Max=%.2f, Avg=%.2f ", 
           actual_min, actual_max, actual_avg);
    return true;
}

// Test: Error Handling
static bool test_lm35_error_handling(void) {
    // Test with invalid temperature
    float invalid_temp = LM35_ADCToTemperature(ADC_MAX_VALUE + 100);
    if (!isnan(invalid_temp)) {
        printf("Invalid ADC value not handled\n");
        return false;
    }
    
    // Test sensor disable/enable
    LM35_Disable();
    System_Delay(100);
    
    float temp_disabled = LM35_ReadTemperatureC();
    if (!isnan(temp_disabled)) {
        printf("Should return NaN when disabled\n");
        LM35_Enable();
        return false;
    }
    
    LM35_Enable();
    System_Delay(100);
    
    float temp_enabled = LM35_ReadTemperatureC();
    if (isnan(temp_enabled)) {
        printf("Should read temperature when enabled\n");
        return false;
    }
    
    return true;
}

// Main test entry point
int main(void) {
    // Initialize system
    GPIO_InitAll();
    UART_Init(115200);
    
    printf("\r\nStarting LM35 Driver Tests...\r\n");
    
    // Run tests
    run_all_tests();
    
    // Blink LED based on test results
    if (fail_count == 0) {
        // Success pattern: 3 short green blinks
        for (int i = 0; i < 3; i++) {
            LED_Set(LED_GREEN);
            System_Delay(100);
            LED_Set(LED_OFF);
            System_Delay(100);
        }
    } else {
        // Failure pattern: 3 long red blinks
        for (int i = 0; i < 3; i++) {
            LED_Set(LED_RED);
            System_Delay(500);
            LED_Set(LED_OFF);
            System_Delay(500);
        }
    }
    
    printf("\r\nTests completed. ");
    if (fail_count == 0) {
        printf("All tests passed!\r\n");
    } else {
        printf("%d test(s) failed.\r\n", fail_count);
    }
    
    return fail_count;
}
