#!/bin/bash
# Build script for LM35 Cortex-M GPIO Project

set -e  # Exit on error

# Default values
BUILD_TYPE="release"
BOARD="stm32f4"
CLEAN_BUILD=false
FLASH=false
DEBUG=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print usage
usage() {
    echo "Build script for LM35 Cortex-M GPIO Project"
    echo ""
    echo "Usage: $0 [options]"
    echo ""
    echo "Options:"
    echo "  -b, --board <board>    Target board (stm32f4, stm32f7, nucleo-f401re)"
    echo "  -t, --type <type>      Build type (debug, release)"
    echo "  -c, --clean            Clean build directory before building"
    echo "  -f, --flash            Flash to board after building"
    echo "  -d, --debug            Build with debug symbols"
    echo "  -h, --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 -b stm32f4 -t release"
    echo "  $0 --board nucleo-f401re --clean --flash"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--board)
            BOARD="$2"
            shift 2
            ;;
        -t|--type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -f|--flash)
            FLASH=true
            shift
            ;;
        -d|--debug)
            DEBUG=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Validate board
case $BOARD in
    stm32f4|stm32f7|stm32h7|nucleo-f401re|nucleo-f746zg)
        # Valid board
        ;;
    *)
        echo -e "${RED}Error: Invalid board '$BOARD'${NC}"
        echo "Valid boards: stm32f4, stm32f7, stm32h7, nucleo-f401re, nucleo-f746zg"
        exit 1
        ;;
esac

# Validate build type
case $BUILD_TYPE in
    debug|release)
        # Valid build type
        ;;
    *)
        echo -e "${RED}Error: Invalid build type '$BUILD_TYPE'${NC}"
        echo "Valid build types: debug, release"
        exit 1
        ;;
esac

# Print build configuration
echo -e "${GREEN}=== LM35 Cortex-M GPIO Build ===${NC}"
echo "Board:        $BOARD"
echo "Build Type:   $BUILD_TYPE"
echo "Clean Build:  $CLEAN_BUILD"
echo "Flash:        $FLASH"
echo "Debug:        $DEBUG"
echo ""

# Check for required tools
check_tools() {
    local tools=("arm-none-eabi-gcc" "make" "cmake")
    local missing=()
    
    for tool in "${tools[@]}"; do
        if ! command -v $tool &> /dev/null; then
            missing+=($tool)
        fi
    done
    
    if [ ${#missing[@]} -ne 0 ]; then
        echo -e "${RED}Error: Missing required tools:${NC}"
        for tool in "${missing[@]}"; do
            echo "  - $tool"
        done
        echo ""
        echo "Install with:"
        echo "  sudo apt-get install gcc-arm-none-eabi make cmake"
        exit 1
    fi
}

check_tools

# Create build directory
BUILD_DIR="build/$BOARD/$BUILD_TYPE"
echo -e "${YELLOW}Creating build directory: $BUILD_DIR${NC}"
mkdir -p $BUILD_DIR

# Clean build if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf $BUILD_DIR/*
fi

# Configure build
echo -e "${YELLOW}Configuring build...${NC}"
cd $BUILD_DIR

# Generate build configuration based on board
case $BOARD in
    stm32f4)
        CMAKE_FLAGS="-DBOARD=STM32F4 -DCPU=cortex-m4 -DFPU=hard"
        ;;
    stm32f7)
        CMAKE_FLAGS="-DBOARD=STM32F7 -DCPU=cortex-m7 -DFPU=hard"
        ;;
    stm32h7)
        CMAKE_FLAGS="-DBOARD=STM32H7 -DCPU=cortex-m7 -DFPU=hard"
        ;;
    nucleo-f401re)
        CMAKE_FLAGS="-DBOARD=NUCLEO_F401RE -DCPU=cortex-m4 -DFPU=soft"
        ;;
    nucleo-f746zg)
        CMAKE_FLAGS="-DBOARD=NUCLEO_F746ZG -DCPU=cortex-m7 -DFPU=hard"
        ;;
esac

# Add build type flags
if [ "$BUILD_TYPE" = "debug" ]; then
    CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_BUILD_TYPE=Debug -DDEBUG=1"
else
    CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_BUILD_TYPE=Release"
fi

# Add debug symbols if requested
if [ "$DEBUG" = true ]; then
    CMAKE_FLAGS="$CMAKE_FLAGS -DDEBUG_SYMBOLS=1"
fi

# Run CMake
cmake ../../.. $CMAKE_FLAGS
if [ $? -ne 0 ]; then
    echo -e "${RED}CMake configuration failed${NC}"
    exit 1
fi

# Build the project
echo -e "${YELLOW}Building project...${NC}"
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed${NC}"
    exit 1
fi

# Show build size
echo -e "${GREEN}Build completed successfully!${NC}"
echo ""
echo -e "${YELLOW}Binary sizes:${NC}"
arm-none-eabi-size lm35_cortex_m.elf

# Copy binaries to root
echo -e "${YELLOW}Copying binaries...${NC}"
cp lm35_cortex_m.elf ../../../bin/
cp lm35_cortex_m.bin ../../../bin/
cp lm35_cortex_m.hex ../../../bin/

# Flash if requested
if [ "$FLASH" = true ]; then
    echo -e "${YELLOW}Flashing to board...${NC}"
    
    # Check for programming tools
    if command -v st-flash &> /dev/null; then
        st-flash write lm35_cortex_m.bin 0x8000000
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Flash successful!${NC}"
        else
            echo -e "${RED}Flash failed${NC}"
            exit 1
        fi
    elif command -v openocd &> /dev/null; then
        openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg \
                -c "program lm35_cortex_m.elf verify reset exit"
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}Flash successful!${NC}"
        else
            echo -e "${RED}Flash failed${NC}"
            exit 1
        fi
    else
        echo -e "${RED}No flashing tool found (st-flash or openocd)${NC}"
        echo "Install stlink-tools or openocd to flash the board"
        exit 1
    fi
fi

echo -e "${GREEN}=== Build Complete ===${NC}"
echo "Binaries available in: bin/"
if [ "$FLASH" = false ]; then
    echo "To flash manually:"
    echo "  st-flash write bin/lm35_cortex_m.bin 0x8000000"
fi
