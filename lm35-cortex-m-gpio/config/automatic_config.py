
## **config/automatic_config.py**
```python
#!/usr/bin/env python3
"""
Automatic Configuration Generator for LM35 + Cortex-M GPIO Project
Detects and configures GPIO pins, ADC settings, and sensor parameters
"""

import json
import os
import argparse
import sys
from pathlib import Path
import subprocess
from datetime import datetime

class AutoConfigGenerator:
    # Board definitions with pin mappings
    BOARD_DEFINITIONS = {
        "STM32F4": {
            "cpu": "Cortex-M4",
            "clock_speed": 168000000,
            "adc_pins": ["PA0", "PA1", "PA2", "PA3", "PC0", "PC1", "PC2", "PC3"],
            "gpio_pins": ["PA5", "PB0", "PB1", "PB2", "PC4", "PC5", "PC6", "PD12", "PD13", "PD14", "PD15"],
            "power_pins": ["3V3", "5V", "GND"],
            "uart_pins": {"TX": "PA2", "RX": "PA3"},
            "led_pins": ["PD12", "PD13", "PD14", "PD15"]
        },
        "STM32F7": {
            "cpu": "Cortex-M7",
            "clock_speed": 216000000,
            "adc_pins": ["PA0", "PA1", "PA2", "PA3", "PC0", "PC1", "PC2", "PC3", "PF10", "PF3"],
            "gpio_pins": ["PB0", "PB1", "PB2", "PE1", "PF2", "PG6", "PG7"],
            "power_pins": ["3V3", "5V", "GND"],
            "uart_pins": {"TX": "PA9", "RX": "PA10"},
            "led_pins": ["PB0", "PB1", "PB2"]
        },
        "STM32H7": {
            "cpu": "Cortex-M7",
            "clock_speed": 400000000,
            "adc_pins": ["PA0", "PA1", "PA2", "PA3", "PC0", "PC1", "PC2", "PC3", "PF10", "PF3", "PF4"],
            "gpio_pins": ["PA5", "PB0", "PB1", "PE1", "PF2", "PG6", "PG7", "PH6"],
            "power_pins": ["3V3", "5V", "GND"],
            "uart_pins": {"TX": "PA9", "RX": "PA10"},
            "led_pins": ["PA5", "PB0", "PB1"]
        },
        "NUCLEO-F401RE": {
            "cpu": "Cortex-M4",
            "clock_speed": 84000000,
            "adc_pins": ["PA0", "PA1", "PA4", "PB0", "PC1", "PC0"],
            "gpio_pins": ["PA5", "PB13", "PB14", "PC7", "PC8", "PC9"],
            "power_pins": ["3V3", "5V", "GND"],
            "uart_pins": {"TX": "PA2", "RX": "PA3"},
            "led_pins": ["PA5", "PB13"]
        },
        "NUCLEO-F746ZG": {
            "cpu": "Cortex-M7",
            "clock_speed": 216000000,
            "adc_pins": ["PA0", "PA1", "PA4", "PB0", "PC1", "PC0", "PC2", "PC3"],
            "gpio_pins": ["PB0", "PB1", "PB14", "PC7", "PC8", "PG14"],
            "power_pins": ["3V3", "5V", "GND"],
            "uart_pins": {"TX": "PA9", "RX": "PA10"},
            "led_pins": ["PB0", "PB1", "PB14"]
        }
    }
    
    def __init__(self, board_type="STM32F4", adc_resolution=12):
        self.board_type = board_type
        self.adc_resolution = adc_resolution
        
        if board_type not in self.BOARD_DEFINITIONS:
            print(f"Warning: Board {board_type} not defined, using STM32F4 as default")
            board_type = "STM32F4"
        
        board_info = self.BOARD_DEFINITIONS[board_type]
        
        self.config = {
            "project": {
                "name": "LM35_CortexM_GPIO",
                "version": "1.0.0",
                "description": "GPIO-based LM35 Temperature Sensor Interface",
                "generated": datetime.now().isoformat(),
                "board": board_type
            },
            "hardware": {
                "sensor": {
                    "type": "LM35",
                    "supply_voltage": 3.3,
                    "temperature_range": [-55, 150],
                    "sensitivity": 10.0,  # mV/°C
                    "accuracy": "±0.5°C at 25°C"
                },
                "board": {
                    "type": board_type,
                    "cpu": board_info["cpu"],
                    "clock_speed": board_info["clock_speed"],
                    "vendor": "STMicroelectronics",
                    "available_pins": {
                        "adc": board_info["adc_pins"],
                        "gpio": board_info["gpio_pins"],
                        "power": board_info["power_pins"],
                        "led": board_info["led_pins"]
                    }
                }
            },
            "gpio_config": {},
            "adc_config": {},
            "uart_config": {},
            "calibration": {},
            "sampling": {}
        }
    
    def generate_adc_config(self, pin="PA0"):
        """Generate ADC configuration"""
        board_info = self.BOARD_DEFINITIONS[self.board_type]
        
        # Validate pin
        if pin not in board_info["adc_pins"]:
            print(f"Warning: Pin {pin} not in available ADC pins for {self.board_type}")
            print(f"Available ADC pins: {', '.join(board_info['adc_pins'])}")
            pin = board_info["adc_pins"][0]
            print(f"Using default pin: {pin}")
        
        # Calculate ADC parameters
        max_adc_value = 2**self.adc_resolution - 1
        voltage_per_step = 3.3 / max_adc_value
        mv_per_step = voltage_per_step * 1000
        
        self.config["adc_config"] = {
            "pin": pin,
            "channel": self.get_adc_channel(pin),
            "resolution": self.adc_resolution,
            "sampling_time": 480 if self.board_type.startswith("STM32F4") else 810,
            "reference_voltage": 3.3,
            "conversion_mode": "single_ended",
            "oversampling": 16,
            "dma_enabled": True,
            "interrupt_enabled": True,
            "max_value": max_adc_value,
            "voltage_per_step": round(voltage_per_step, 6),
            "mv_per_step": round(mv_per_step, 4),
            "sampling_rate_hz": 1000
        }
    
    def get_adc_channel(self, pin):
        """Get ADC channel from pin name"""
        # Simplified mapping - should be expanded based on board
        channel_map = {
            "PA0": 0, "PA1": 1, "PA2": 2, "PA3": 3,
            "PC0": 10, "PC1": 11, "PC2": 12, "PC3": 13,
            "PB0": 8, "PB1": 9
        }
        return channel_map.get(pin, 0)
    
    def generate_gpio_config(self, led_pin=None):
        """Generate GPIO configuration"""
        board_info = self.BOARD_DEFINITIONS[self.board_type]
        
        # Use provided LED pin or first available
        if not led_pin or led_pin not in board_info["led_pins"]:
            led_pin = board_info["led_pins"][0]
        
        self.config["gpio_config"] = {
            "sensor_power": {
                "pin": "3V3",
                "mode": "output",
                "pull": "none",
                "initial_state": "high",
                "description": "LM35 VCC supply"
            },
            "sensor_ground": {
                "pin": "GND",
                "mode": "output",
                "pull": "none",
                "initial_state": "low",
                "description": "LM35 ground"
            },
            "status_led": {
                "pin": led_pin,
                "mode": "output",
                "pull": "none",
                "initial_state": "low",
                "description": "System status indicator"
            }
        }
    
    def generate_uart_config(self):
        """Generate UART configuration for debug output"""
        board_info = self.BOARD_DEFINITIONS[self.board_type]
        
        self.config["uart_config"] = {
            "enabled": True,
            "port": "USART1" if "F7" in self.board_type else "USART2",
            "baudrate": 115200,
            "tx_pin": board_info["uart_pins"]["TX"],
            "rx_pin": board_info["uart_pins"]["RX"],
            "data_bits": 8,
            "stop_bits": 1,
            "parity": "none",
            "flow_control": "none"
        }
    
    def generate_calibration(self):
        """Generate calibration parameters"""
        mv_per_celsius = 10.0  # LM35 sensitivity: 10mV/°C
        
        # Calculate temperature from ADC value
        adc_step_mv = self.config["adc_config"]["mv_per_step"]
        celsius_per_adc_step = adc_step_mv / mv_per_celsius
        
        self.config["calibration"] = {
            "mv_per_celsius": mv_per_celsius,
            "celsius_per_adc_step": round(celsius_per_adc_step, 4),
            "offset_voltage": 0.0,
            "gain_correction": 1.0,
            "room_temperature_calibration": 25.0,
            "calibration_points": [
                {"adc_value": 0, "temperature_c": -55, "voltage_mv": 0},
                {"adc_value": 1024, "temperature_c": 25, "voltage_mv": 250},
                {"adc_value": 3072, "temperature_c": 150, "voltage_mv": 1500}
            ],
            "auto_calibration": True,
            "calibration_interval_hours": 24
        }
    
    def generate_sampling_config(self):
        """Generate sampling configuration"""
        self.config["sampling"] = {
            "sample_count": 16,
            "update_rate_ms": 1000,
            "filter_alpha": 0.1,
            "filter_type": "exponential_moving_average",
            "oversampling_factor": 4,
            "noise_rejection": "median_filter",
            "thresholds": {
                "low": 0.0,
                "high": 50.0,
                "critical": 80.0
            }
        }
    
    def generate_config_file(self, output_path="config/project_config.json"):
        """Generate complete configuration file"""
        # Generate all configurations
        self.generate_adc_config()
        self.generate_gpio_config()
        self.generate_uart_config()
        self.generate_calibration()
        self.generate_sampling_config()
        
        # Save to file
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            json.dump(self.config, f, indent=2, sort_keys=True)
        
        print(f"✓ Configuration generated: {output_path}")
        
        return self.config
    
    def generate_c_header(self, output_path="inc/config.h"):
        """Generate C header file from configuration"""
        config = self.config
        
        # Get pin definitions
        adc_pin = config['adc_config']['pin']
        led_pin = config['gpio_config']['status_led']['pin']
        tx_pin = config['uart_config']['tx_pin']
        rx_pin = config['uart_config']['rx_pin']
        
        # Create pin macros
        pin_macros = f"""// Pin Definitions
#define LM35_ADC_PIN            {self._pin_to_macro(adc_pin)}
#define LM35_ADC_CHANNEL        {config['adc_config']['channel']}
#define STATUS_LED_PIN          {self._pin_to_macro(led_pin)}
#define DEBUG_UART_TX_PIN       {self._pin_to_macro(tx_pin)}
#define DEBUG_UART_RX_PIN       {self._pin_to_macro(rx_pin)}
"""
        
        header_content = f"""/**
 * AUTOGENERATED CONFIGURATION FILE
 * LM35 Temperature Sensor + Cortex-M GPIO Interface
 * Board: {config['hardware']['board']['type']}
 * CPU: {config['hardware']['board']['cpu']}
 * Generated: {config['project']['generated']}
 * 
 * DO NOT EDIT THIS FILE DIRECTLY
 * Edit config/project_config.json and regenerate
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <stdint.h>
#include <stdbool.h>

{pin_macros}
// ADC Configuration
#define ADC_RESOLUTION_BITS     {config['adc_config']['resolution']}
#define ADC_MAX_VALUE           {config['adc_config']['max_value']}
#define ADC_REF_VOLTAGE         {config['adc_config']['reference_voltage']}f
#define ADC_SAMPLING_TIME       {config['adc_config']['sampling_time']}
#define ADC_OVERSAMPLING        {config['adc_config']['oversampling']}
#define ADC_VOLTAGE_PER_STEP    {config['adc_config']['voltage_per_step']}f
#define ADC_MV_PER_STEP         {config['adc_config']['mv_per_step']}f
#define ADC_SAMPLING_RATE_HZ    {config['adc_config']['sampling_rate_hz']}

// UART Configuration
#define DEBUG_UART_ENABLED      {1 if config['uart_config']['enabled'] else 0}
#define DEBUG_UART_BAUDRATE     {config['uart_config']['baudrate']}
#define DEBUG_UART_PORT         {config['uart_config']['port']}

// Sensor Calibration
#define LM35_SENSITIVITY_MV_C   {config['calibration']['mv_per_celsius']}f
#define C_PER_ADC_STEP          {config['calibration']['celsius_per_adc_step']}f
#define CALIBRATION_OFFSET      {config['calibration']['offset_voltage']}f
#define CALIBRATION_GAIN        {config['calibration']['gain_correction']}f
#define ROOM_TEMP_CALIBRATION   {config['calibration']['room_temperature_calibration']}f
#define AUTO_CALIBRATION        {1 if config['calibration']['auto_calibration'] else 0}

// Sampling Configuration
#define TEMP_SAMPLE_COUNT       {config['sampling']['sample_count']}
#define TEMP_UPDATE_RATE_MS     {config['sampling']['update_rate_ms']}
#define FILTER_ALPHA            {config['sampling']['filter_alpha']}f
#define OVERSAMPLING_FACTOR     {config['sampling']['oversampling_factor']}

// Temperature Thresholds (Celsius)
#define TEMP_LOW_THRESHOLD      {config['sampling']['thresholds']['low']}f
#define TEMP_HIGH_THRESHOLD     {config['sampling']['thresholds']['high']}f
#define TEMP_CRITICAL_THRESHOLD {config['sampling']['thresholds']['critical']}f

// Conversion Macros
#define ADC_TO_VOLTAGE(adc_val)     ((adc_val) * ADC_VOLTAGE_PER_STEP)
#define ADC_TO_MILLIVOLTS(adc_val)  ((adc_val) * ADC_MV_PER_STEP)
#define ADC_TO_CELSIUS(adc_val)     (((adc_val) * C_PER_ADC_STEP * CALIBRATION_GAIN) + CALIBRATION_OFFSET)
#define MILLIVOLTS_TO_CELSIUS(mv)   (((mv) / LM35_SENSITIVITY_MV_C) * CALIBRATION_GAIN + CALIBRATION_OFFSET)
#define CELSIUS_TO_FAHRENHEIT(c)    ((c) * 9.0f / 5.0f + 32.0f)
#define FAHRENHEIT_TO_CELSIUS(f)    (((f) - 32.0f) * 5.0f / 9.0f)

// Status Codes
typedef enum {{
    TEMP_READ_SUCCESS = 0,
    TEMP_READ_ERROR_ADC,
    TEMP_READ_ERROR_OVERRANGE,
    TEMP_READ_ERROR_UNDERRANGE,
    TEMP_READ_ERROR_TIMEOUT
}} TempReadStatus;

// Temperature Status
typedef enum {{
    TEMP_STATUS_NORMAL = 0,
    TEMP_STATUS_LOW,
    TEMP_STATUS_HIGH,
    TEMP_STATUS_CRITICAL,
    TEMP_STATUS_ERROR
}} TemperatureStatus;

// Filter Types
typedef enum {{
    FILTER_NONE = 0,
    FILTER_MOVING_AVERAGE,
    FILTER_EXPONENTIAL,
    FILTER_MEDIAN
}} FilterType;

// Board Information
#define BOARD_TYPE              "{config['hardware']['board']['type']}"
#define BOARD_CPU               "{config['hardware']['board']['cpu']}"
#define BOARD_CLOCK_HZ          {config['hardware']['board']['clock_speed']}UL
#define FIRMWARE_VERSION        "{config['project']['version']}"

// Function Prototypes
float read_temperature_celsius(void);
float read_temperature_fahrenheit(void);
TemperatureStatus get_temperature_status(float temperature_c);
void calibrate_temperature_sensor(float reference_temp_c);
void update_calibration_parameters(float offset, float gain);

// Debug Macros
#ifdef DEBUG_ENABLED
    #define DEBUG_PRINT(fmt, ...)   debug_printf(fmt, ##__VA_ARGS__)
    #define DEBUG_HEX(data, len)    debug_print_hex(data, len)
#else
    #define DEBUG_PRINT(fmt, ...)   ((void)0)
    #define DEBUG_HEX(data, len)    ((void)0)
#endif

#endif // __CONFIG_H
"""
        
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w') as f:
            f.write(header_content)
        
        print(f"✓ C header generated: {output_path}")
        return header_content
    
    def _pin_to_macro(self, pin):
        """Convert pin name like 'PA0' to macro like 'GPIO_PIN_0'"""
        if pin.startswith(('PA', 'PB', 'PC', 'PD', 'PE', 'PF', 'PG', 'PH')):
            port = pin[:2]
            pin_num = pin[2:]
            return f"GPIO_PIN_{pin_num}"
        return pin
    
    def generate_makefile_config(self, output_path="Makefile.config"):
        """Generate Makefile configuration"""
        config = self.config
        
        makefile_content = f"""# Auto-generated Makefile configuration
# Board: {config['hardware']['board']['type']}
# Generated: {config['project']['generated']}

# Board settings
BOARD_TYPE = {config['hardware']['board']['type']}
CPU = {config['hardware']['board']['cpu']}
CLOCK_HZ = {config['hardware']['board']['clock_speed']}

# Toolchain
CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AS = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Compiler flags
MCU_FLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS = $(MCU_FLAGS) \\
    -O2 -Wall -fdata-sections -ffunction-sections \\
    -D{config['hardware']['board']['type']} \\
    -DADC_RESOLUTION={config['adc_config']['resolution']} \\
    -DADC_PIN={config['adc_config']['pin']} \\
    -DSTATUS_LED_PIN={config['gpio_config']['status_led']['pin']}

# Linker flags
LDFLAGS = $(MCU_FLAGS) \\
    -T"src/startup/startup_stm32f4xx.ld" \\
    -Wl,--gc-sections \\
    -specs=nosys.specs \\
    -specs=nano.specs

# Directories
SRC_DIR = src
INC_DIR = inc
BUILD_DIR = build
BIN_DIR = bin

# Source files
SRCS = \\
    $(SRC_DIR)/main.c \\
    $(SRC_DIR)/system_init.c \\
    $(SRC_DIR)/drivers/lm35_driver.c \\
    $(SRC_DIR)/drivers/gpio_interface.c

# Include directories
INCLUDES = -I$(INC_DIR) -I$(SRC_DIR)/drivers

# Targets
TARGET = $(BIN_DIR)/lm35_cortex_m

.PHONY: all clean flash debug

all: $(TARGET).elf $(TARGET).bin $(TARGET).hex

$(TARGET).elf: $(SRCS)
\tmkdir -p $(BUILD_DIR) $(BIN_DIR)
\t$(CC) $(CFLAGS) $(INCLUDES) $(SRCS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
\t$(OBJCOPY) -O binary $< $@

$(TARGET).hex: $(TARGET).elf
\t$(OBJCOPY) -O ihex $< $@

clean:
\trm -rf $(BUILD_DIR) $(BIN_DIR)

flash: $(TARGET).bin
\tst-flash write $(TARGET).bin 0x8000000

size: $(TARGET).elf
\t$(SIZE) $<

debug:
\topenocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg
"""
        
        with open(output_path, 'w') as f:
            f.write(makefile_content)
        
        print(f"✓ Makefile config generated: {output_path}")

def validate_configuration(config):
    """Validate the generated configuration"""
    errors = []
    warnings = []
    
    # Check ADC resolution
    if config['adc_config']['resolution'] not in [8, 10, 12, 16]:
        errors.append(f"Invalid ADC resolution: {config['adc_config']['resolution']}")
    
    # Check temperature thresholds
    thresholds = config['sampling']['thresholds']
    if thresholds['low'] >= thresholds['high']:
        errors.append("Low threshold must be less than high threshold")
    
    if thresholds['high'] >= thresholds['critical']:
        errors.append("High threshold must be less than critical threshold")
    
    # Check sampling rate
    if config['sampling']['update_rate_ms'] < 10:
        warnings.append("Update rate very fast (<10ms), may affect performance")
    
    # Check filter alpha
    alpha = config['sampling']['filter_alpha']
    if alpha <= 0 or alpha > 1:
        errors.append(f"Filter alpha must be between 0 and 1, got {alpha}")
    
    return errors, warnings

def main():
    parser = argparse.ArgumentParser(
        description='Generate LM35 + Cortex-M GPIO Configuration',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --board STM32F4 --adc-res 12
  %(prog)s --board STM32F7 --adc-pin PC0 --led-pin PB0
  %(prog)s --board NUCLEO-F401RE --adc-res 10
        """
    )
    
    parser.add_argument('--board', type=str, default='STM32F4', 
                       choices=['STM32F4', 'STM32F7', 'STM32H7', 'NUCLEO-F401RE', 'NUCLEO-F746ZG'],
                       help='Board type')
    parser.add_argument('--adc-res', type=int, default=12, choices=[8, 10, 12, 16],
                       help='ADC resolution in bits')
    parser.add_argument('--adc-pin', type=str,
                       help='ADC pin to use (e.g., PA0, PC1)')
    parser.add_argument('--led-pin', type=str,
                       help='Status LED pin')
    parser.add_argument('--output-dir', type=str, default='config',
                       help='Output directory for configuration files')
    parser.add_argument('--check-only', action='store_true',
                       help='Only check configuration without generating files')
    
    args = parser.parse_args()
    
    print("="*60)
    print("LM35 + Cortex-M GPIO Configuration Generator")
    print("="*60)
    
    # Create output directories
    os.makedirs(args.output_dir, exist_ok=True)
    os.makedirs('inc', exist_ok=True)
    os.makedirs('scripts', exist_ok=True)
    
    # Generate configuration
    generator = AutoConfigGenerator(args.board, args.adc_res)
    
    # Set custom pins if provided
    if args.adc_pin:
        generator.config['adc_config']['pin'] = args.adc_pin
    
    if args.led_pin:
        generator.config['gpio_config']['status_led']['pin'] = args.led_pin
    
    if args.check_only:
        print("\nConfiguration Check:")
        print("-" * 40)
        config = generator.config
        errors, warnings = validate_configuration(config)
        
        if errors:
            print("❌ ERRORS:")
            for error in errors:
                print(f"  - {error}")
        else:
            print("✅ No errors found")
        
        if warnings:
            print("\n⚠️  WARNINGS:")
            for warning in warnings:
                print(f"  - {warning}")
        
        print("\nConfiguration Summary:")
        print("-" * 40)
        print(f"Board: {config['hardware']['board']['type']}")
        print(f"ADC Pin: {config['adc_config']['pin']}")
        print(f"ADC Resolution: {config['adc_config']['resolution']} bits")
        print(f"LED Pin: {config['gpio_config']['status_led']['pin']}")
        return
    
    # Generate files
    config = generator.generate_config_file(
        os.path.join(args.output_dir, 'project_config.json')
    )
    
    generator.generate_c_header('inc/config.h')
    generator.generate_makefile_config('Makefile.config')
    
    # Validate configuration
    errors, warnings = validate_configuration(config)
    
    if errors:
        print("\n❌ Configuration errors found:")
        for error in errors:
            print(f"  - {error}")
    
    if warnings:
        print("\n⚠️  Configuration warnings:")
        for warning in warnings:
            print(f"  - {warning}")
    
    # Print summary
    print("\n" + "="*60)
    print("CONFIGURATION SUMMARY")
    print("="*60)
    print(f"Board: {config['hardware']['board']['type']}")
    print(f"CPU: {config['hardware']['board']['cpu']}")
    print(f"Clock: {config['hardware']['board']['clock_speed']:,} Hz")
    print(f"ADC Pin: {config['adc_config']['pin']} (Channel {config['adc_config']['channel']})")
    print(f"ADC Resolution: {config['adc_config']['resolution']} bits")
    print(f"Max ADC Value: {config['adc_config']['max_value']}")
    print(f"Voltage per step: {config['adc_config']['voltage_per_step']:.6f} V")
    print(f"Temperature per ADC step: {config['calibration']['celsius_per_adc_step']:.4f} °C")
    print(f"Status LED: {config['gpio_config']['status_led']['pin']}")
    print(f"UART Debug: {config['uart_config']['tx_pin']}/{config['uart_config']['rx_pin']}")
    print("="*60)
    
    print("\n✅ Configuration complete!")
    print("\nNext steps:")
    print("1. Review config/project_config.json")
    print("2. Build the project: make")
    print("3. Connect LM35 to specified pins")
    print("4. Flash to board: make flash")
    print("5. Monitor output: screen /dev/ttyACM0 115200")

if __name__ == "__main__":
    main()
