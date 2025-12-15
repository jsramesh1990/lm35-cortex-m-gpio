/**
 * GPIO Interface Test Suite
 */

#include "gpio_interface.h"
#include "config.h"
#include <stdio.h>

// Test definitions
typedef struct {
    const char* name;
    bool (*test_func)(void);
} GpioTestCase;

// Test function prototypes
static bool test_gpio_init(void);
static bool test_gpio_output(void);
static bool test_gpio_toggle(void);
static bool test_adc_init(void);
static bool test_adc_read(void);
static bool test_led_control(void);
static bool test_uart_communication(void);
static bool test_system_timing(void);

// Test cases
static GpioTestCase gpio_tests[] = {
    {"GPIO Initialization", test_gpio_init},
    {"GPIO Output", test_gpio_output},
    {"GPIO Toggle", test_gpio_toggle},
    {"LED Control", test_led_control},
    {"ADC Initialization", test_adc_init},
    {"ADC Read", test_adc_read},
    {"UART Communication", test_uart_communication},
    {"System Timing", test_system_timing},
    {NULL, NULL}
};

// Test runner
void run_gpio_tests(void) {
    printf("\n=== GPIO Interface Test Suite ===\n");
    printf("Board: %s\n", BOARD_TYPE);
    printf("\n");
    
    int total_tests = 0;
    int passed_tests = 0;
    
    for (int i = 0; gpio_tests[i].name != NULL; i++) {
        printf("Test %d: %-25s ", i + 1, gpio_tests[i].name);
        
        total_tests++;
        
        bool result = gpio_tests[i].test_func();
        
        if (result) {
            printf("[PASS]\n");
            passed_tests++;
        } else {
            printf("[FAIL]\n");
        }
    }
    
    printf("\n=== Test Summary ===\n");
    printf("Total Tests:  %d\n", total_tests);
    printf("Passed:       %d\n", passed_tests);
    printf("Failed:       %d\n", total_tests - passed_tests);
    
    if (passed_tests == total_tests) {
        printf("\n✅ All GPIO tests passed!\n");
    } else {
        printf("\n❌ Some GPIO tests failed!\n");
    }
}

// Test: GPIO Initialization
static bool test_gpio_init(void) {
    GPIO_InitAll();
    
    // Check that LED pin is configured as output
    GPIO_PinState state = GPIO_GetPin(STATUS_LED_PIN);
    
    // After init, pin should be in reset state (0)
    if (state != GPIO_PIN_RESET) {
        printf("LED pin not in reset state after init\n");
        return false;
    }
    
    return true;
}

// Test: GPIO Output
static bool test_gpio_output(void) {
    // Test setting pin high
    GPIO_SetPin(STATUS_LED_PIN, GPIO_PIN_SET);
    GPIO_PinState high_state = GPIO_GetPin(STATUS_LED_PIN);
    
    if (high_state != GPIO_PIN_SET) {
        printf("Failed to set pin high\n");
        return false;
    }
    
    // Test setting pin low
    GPIO_SetPin(STATUS_LED_PIN, GPIO_PIN_RESET);
    GPIO_PinState low_state = GPIO_GetPin(STATUS_LED_PIN);
    
    if (low_state != GPIO_PIN_RESET) {
        printf("Failed to set pin low\n");
        return false;
    }
    
    return true;
}

// Test: GPIO Toggle
static bool test_gpio_toggle(void) {
    // Start with known state
    GPIO_SetPin(STATUS_LED_PIN, GPIO_PIN_RESET);
    
    // Toggle multiple times and verify
    for (int i = 0; i < 5; i++) {
        GPIO_PinState before = GPIO_GetPin(STATUS_LED_PIN);
        GPIO_TogglePin(STATUS_LED_PIN);
        GPIO_PinState after = GPIO_GetPin(STATUS_LED_PIN);
        
        if (after == before) {
            printf("Toggle failed on iteration %d\n", i);
            return false;
        }
        
        System_Delay(10);
    }
    
    return true;
}

// Test: LED Control
static bool test_led_control(void) {
    LED_Init();
    
    // Test all LED colors
    LED_Color colors[] = {LED_OFF, LED_GREEN, LED_YELLOW, LED_RED, LED_BLUE};
    const char* color_names[] = {"OFF", "GREEN", "YELLOW", "RED", "BLUE"};
    
    for (int i = 0; i < sizeof(colors)/sizeof(colors[0]); i++) {
        LED_Set(colors[i]);
        System_Delay(100);
        
        // Verify LED state
        GPIO_PinState state = GPIO_GetPin(STATUS_LED_PIN);
        
        // For single-color LED, only OFF should be 0, others should be 1
        if (colors[i] == LED_OFF && state != GPIO_PIN_RESET) {
            printf("LED not off when set to %s\n", color_names[i]);
            return false;
        }
        
        if (colors[i] != LED_OFF && state != GPIO_PIN_SET) {
            printf("LED not on when set to %s\n", color_names[i]);
            return false;
        }
    }
    
    // Test blinking
    LED_Blink(LED_GREEN, 100);
    LED_Blink(LED_RED, 100);
    
    return true;
}

// Test: ADC Initialization
static bool test_adc_init(void) {
    bool result = ADC_Init(LM35_ADC_CHANNEL, ADC_RESOLUTION_BITS);
    
    if (!result) {
        printf("ADC initialization failed\n");
        return false;
    }
    
    // Try to calibrate ADC
    result = ADC_Calibrate();
    
    if (!result) {
        printf("ADC calibration failed\n");
        return false;
    }
    
    return true;
}

// Test: ADC Read
static bool test_adc_read(void) {
    uint32_t adc_value;
    ADC_Result result = ADC_Read(&adc_value);
    
    if (result != ADC_SUCCESS) {
        printf("ADC read failed with error: %d\n", result);
        return false;
    }
    
    // Check if value is within reasonable range
    if (adc_value > ADC_MAX_VALUE) {
        printf("ADC value out of range: %lu > %lu\n", adc_value, ADC_MAX_VALUE);
        return false;
    }
    
    printf("ADC: %lu (%.3f V) ", adc_value, ADC_ToVoltage(adc_value));
    return true;
}

// Test: UART Communication
static bool test_uart_communication(void) {
#if DEBUG_UART_ENABLED
    // Initialize UART
    UART_Init(DEBUG_UART_BAUDRATE);
    
    // Test string transmission
    const char* test_string = "UART Test\n";
    UART_SendString(test_string);
    
    // Test formatted output
    UART_Printf("Formatted test: %d, %.2f, %s\n", 42, 3.14159, "OK");
    
    // Test byte transmission
    uint8_t test_bytes[] = {0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x0A}; // "Hello\n"
    UART_SendBytes(test_bytes, sizeof(test_bytes));
    
    printf("UART tests sent (check serial monitor) ");
#endif
    
    return true;
}

// Test: System Timing
static bool test_system_timing(void) {
    // Test delay functions
    
    // Test millisecond delay
    uint32_t start_ms = System_GetTick();
    System_Delay(100); // 100ms delay
    uint32_t end_ms = System_GetTick();
    
    uint32_t elapsed_ms = end_ms - start_ms;
    
    // Allow some tolerance for timing
    if (elapsed_ms < 90 || elapsed_ms > 110) {
        printf("Millisecond delay inaccurate: %lu ms\n", elapsed_ms);
        return false;
    }
    
    // Test microsecond delay (approximate)
    start_ms = System_GetTick();
    System_DelayMicroseconds(1000); // 1ms in microseconds
    end_ms = System_GetTick();
    
    elapsed_ms = end_ms - start_ms;
    
    if (elapsed_ms < 1 || elapsed_ms > 3) {
        printf("Microsecond delay inaccurate: %lu ms\n", elapsed_ms);
        return false;
    }
    
    printf("Timing: %lu ms delay ", elapsed_ms);
    return true;
}

// Main test entry point
int main(void) {
    // Initialize system
    GPIO_InitAll();
    
#if DEBUG_UART_ENABLED
    UART_Init(115200);
#endif
    
    printf("\r\nStarting GPIO Interface Tests...\r\n");
    
    // Run tests
    run_gpio_tests();
    
    // Indicate test completion with LED pattern
    for (int i = 0; i < 3; i++) {
        LED_Set(LED_GREEN);
        System_Delay(200);
        LED_Set(LED_OFF);
        System_Delay(200);
    }
    
    printf("\r\nGPIO tests completed.\r\n");
    
    return 0;
}
