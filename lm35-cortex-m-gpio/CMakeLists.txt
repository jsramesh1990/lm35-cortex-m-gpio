cmake_minimum_required(VERSION 3.16)
project(lm35_cortex_m_gpio C CXX ASM)

# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project version
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

# Detect target platform
if(NOT DEFINED BOARD)
    set(BOARD "STM32F4")
endif()

if(NOT DEFINED CPU)
    set(CPU "cortex-m4")
endif()

if(NOT DEFINED FPU)
    set(FPU "fpv4-sp-d16")
endif()

if(NOT DEFINED FLOAT_ABI)
    set(FLOAT_ABI "hard")
endif()

# Toolchain configuration
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Toolchain paths (adjust these to match your installation)
set(TOOLCHAIN_PREFIX arm-none-eabi-)
find_program(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
find_program(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
find_program(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# Compiler flags
set(COMMON_FLAGS "-mcpu=${CPU} -mthumb -mfpu=${FPU} -mfloat-abi=${FLOAT_ABI}")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -Wall -Werror -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -Wall -Werror -ffunction-sections -fdata-sections")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp")

# Debug/Release configuration
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -DDEBUG")
set(CMAKE_ASM_FLAGS_DEBUG "-g3")

set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_FLAGS} -Wl,--gc-sections -specs=nosys.specs -specs=nano.specs")

# Linker script (board-specific)
if(BOARD STREQUAL "STM32F4")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/startup/STM32F407VGTx_FLASH.ld)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")
    add_definitions(-DSTM32F4 -DARM_MATH_CM4)
elseif(BOARD STREQUAL "STM32F7")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/startup/STM32F746NGHx_FLASH.ld)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")
    add_definitions(-DSTM32F7 -DARM_MATH_CM7)
elseif(BOARD STREQUAL "NUCLEO_F401RE")
    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/src/startup/STM32F401RETx_FLASH.ld)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")
    add_definitions(-DSTM32F4 -DARM_MATH_CM4)
endif()

# Include directories
include_directories(
    inc
    src/drivers
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Source files
file(GLOB_RECURSE SOURCES 
    "src/*.c"
    "src/drivers/*.c"
)

# Assembly startup files
set(ASSEMBLY_SOURCES
    src/startup/startup_stm32f4xx.s
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${ASSEMBLY_SOURCES})

# Post-build steps to generate binary files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMENT "Generating binary and hex files"
)

# Size target to show memory usage
add_custom_target(size
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    DEPENDS ${PROJECT_NAME}
    COMMENT "Calculating memory usage"
)

# Test targets
enable_testing()

# Add LM35 driver tests
add_executable(test_lm35 tests/test_lm35.c ${SOURCES} ${ASSEMBLY_SOURCES})
target_compile_definitions(test_lm35 PRIVATE TEST_MODE=1)
add_test(NAME test_lm35 COMMAND test_lm35)

# Add GPIO interface tests
add_executable(test_gpio tests/test_gpio.c ${SOURCES} ${ASSEMBLY_SOURCES})
target_compile_definitions(test_gpio PRIVATE GPIO_TEST=1)
add_test(NAME test_gpio COMMAND test_gpio)

# Install target (optional)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Output configuration summary
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Board: ${BOARD}")
message(STATUS "CPU: ${CPU}")
message(STATUS "FPU: ${FPU}")
message(STATUS "Float ABI: ${FLOAT_ABI}")
message(STATUS "Compiler: ${CMAKE_C_COMPILER}")
message(STATUS "Linker script: ${LINKER_SCRIPT}")
