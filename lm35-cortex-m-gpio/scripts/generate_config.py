#!/usr/bin/env python3
"""
Configuration Generation Script Wrapper
Simplifies calling the automatic config generator
"""

import os
import sys
import subprocess
import json
from pathlib import Path

def main():
    # Default configuration
    config = {
        "board": "STM32F4",
        "adc_res": 12,
        "adc_pin": "PA0",
        "led_pin": "PD13"
    }
    
    # Check for command line arguments
    if len(sys.argv) > 1:
        for arg in sys.argv[1:]:
            if arg.startswith("--board="):
                config["board"] = arg.split("=")[1]
            elif arg.startswith("--adc-res="):
                config["adc_res"] = int(arg.split("=")[1])
            elif arg.startswith("--adc-pin="):
                config["adc_pin"] = arg.split("=")[1]
            elif arg.startswith("--led-pin="):
                config["led_pin"] = arg.split("=")[1]
            elif arg in ["-h", "--help"]:
                print_help()
                return
            elif arg == "--list-boards":
                list_boards()
                return
    
    # Generate configuration
    cmd = [
        "python3", "config/automatic_config.py",
        "--board", config["board"],
        "--adc-res", str(config["adc_res"]),
        "--adc-pin", config["adc_pin"],
        "--led-pin", config["led_pin"]
    ]
    
    print(f"Generating configuration for {config['board']}...")
    print(f"ADC Pin: {config['adc_pin']}, Resolution: {config['adc_res']} bits")
    print(f"LED Pin: {config['led_pin']}")
    
    # Run the config generator
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error generating configuration: {e}")
        sys.exit(1)
    
    # Verify generated files
    verify_configuration()

def print_help():
    print("""
Configuration Generator for LM35 Cortex-M GPIO Project

Usage:
  python3 scripts/generate_config.py [options]

Options:
  --board=<board>      Board type (STM32F4, STM32F7, STM32H7, NUCLEO-F401RE, NUCLEO-F746ZG)
  --adc-res=<bits>     ADC resolution (8, 10, 12, 16)
  --adc-pin=<pin>      ADC pin (e.g., PA0, PC1)
  --led-pin=<pin>      Status LED pin
  --list-boards        List available board configurations
  -h, --help           Show this help message

Examples:
  python3 scripts/generate_config.py --board=STM32F4 --adc-pin=PA1
  python3 scripts/generate_config.py --board=NUCLEO-F401RE --adc-res=10
  python3 scripts/generate_config.py --list-boards
""")

def list_boards():
    print("Available board configurations:")
    print("-" * 40)
    print("STM32F4       - STM32F4 Discovery (168 MHz)")
    print("STM32F7       - STM32F7 Discovery (216 MHz)")
    print("STM32H7       - STM32H7 Discovery (400 MHz)")
    print("NUCLEO-F401RE - STM32 Nucleo-F401RE (84 MHz)")
    print("NUCLEO-F746ZG - STM32 Nucleo-F746ZG (216 MHz)")
    print("\nDefault ADC pins:")
    print("STM32F4/NUCLEO: PA0, PA1, PA2, PA3, PC0, PC1")
    print("STM32F7/H7:     PA0, PA1, PA2, PA3, PC0, PC1, PF10")

def verify_configuration():
    """Verify that configuration files were generated correctly"""
    required_files = [
        "config/project_config.json",
        "inc/config.h",
        "Makefile.config"
    ]
    
    print("\nVerifying generated files...")
    all_ok = True
    
    for file in required_files:
        if Path(file).exists():
            print(f"✓ {file}")
            
            # Validate JSON config
            if file.endswith(".json"):
                try:
                    with open(file, 'r') as f:
                        json.load(f)
                    print(f"  ✓ Valid JSON")
                except json.JSONDecodeError as e:
                    print(f"  ✗ Invalid JSON: {e}")
                    all_ok = False
        else:
            print(f"✗ {file} - Missing!")
            all_ok = False
    
    if all_ok:
        print("\n✅ Configuration verified successfully!")
        print("\nNext steps:")
        print("1. Review the generated configuration")
        print("2. Build the project: make")
        print("3. Connect your LM35 sensor")
        print("4. Flash the firmware: make flash")
    else:
        print("\n❌ Configuration verification failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()
