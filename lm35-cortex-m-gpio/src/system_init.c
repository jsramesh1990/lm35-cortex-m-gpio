/**
 * System Initialization
 * Cortex-M specific initialization routines
 */

#include "config.h"
#include "gpio_interface.h"
#include <stdint.h>

// System Clock Configuration
typedef struct {
    uint32_t clock_source;
    uint32_t pll_m;
    uint32_t pll_n;
    uint32_t pll_p;
    uint32_t pll_q;
    uint32_t ahb_prescaler;
    uint32_t apb1_prescaler;
    uint32_t apb2_prescaler;
    uint32_t flash_latency;
} SystemClockConfig;

// System Configuration
static SystemClockConfig sys_clock_config;

// Function prototypes
static void SystemClock_Config(void);
static void CPU_Cache_Enable(void);
static void MPU_Config(void);
static void NVIC_Config(void);
static void PeriphClock_Enable(void);
static void FPU_Enable(void);

// System initialization
void System_Init(void) {
    // Reset of all peripherals, Initializes the Flash interface
    // In real implementation, call HAL_Init()
    
    // Configure system clock
    SystemClock_Config();
    
    // Enable FPU for Cortex-M4/M7
    FPU_Enable();
    
    // Configure CPU cache (for Cortex-M7)
    CPU_Cache_Enable();
    
    // Configure MPU (Memory Protection Unit)
    MPU_Config();
    
    // Configure NVIC (Nested Vectored Interrupt Controller)
    NVIC_Config();
    
    // Enable peripheral clocks
    PeriphClock_Enable();
    
    // Initialize systick timer
    SysTick_Config(SystemCoreClock / 1000); // 1ms tick
}

// Configure system clock
static void SystemClock_Config(void) {
    // Default configuration for STM32F4 @ 168MHz
    sys_clock_config.clock_source = 1;    // HSE
    sys_clock_config.pll_m = 8;
    sys_clock_config.pll_n = 336;
    sys_clock_config.pll_p = 2;
    sys_clock_config.pll_q = 7;
    sys_clock_config.ahb_prescaler = 1;   // AHB = 168MHz
    sys_clock_config.apb1_prescaler = 4;  // APB1 = 42MHz
    sys_clock_config.apb2_prescaler = 2;  // APB2 = 84MHz
    sys_clock_config.flash_latency = 5;
    
#if defined(STM32F7)
    // Configuration for STM32F7 @ 216MHz
    sys_clock_config.pll_n = 432;
    sys_clock_config.ahb_prescaler = 1;   // AHB = 216MHz
    sys_clock_config.apb1_prescaler = 4;  // APB1 = 54MHz
    sys_clock_config.apb2_prescaler = 2;  // APB2 = 108MHz
    sys_clock_config.flash_latency = 7;
#elif defined(STM32H7)
    // Configuration for STM32H7 @ 400MHz
    sys_clock_config.pll_n = 400;
    sys_clock_config.ahb_prescaler = 1;   // AHB = 400MHz
    sys_clock_config.apb1_prescaler = 2;  // APB1 = 200MHz
    sys_clock_config.apb2_prescaler = 2;  // APB2 = 200MHz
    sys_clock_config.flash_latency = 4;
#endif
    
    // In real implementation, configure RCC registers
    // This is a simplified placeholder
}

// Enable FPU
static void FPU_Enable(void) {
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    // Enable CP10 and CP11 coprocessors
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
#endif
}

// Configure CPU cache (Cortex-M7)
static void CPU_Cache_Enable(void) {
#ifdef __DCACHE_PRESENT
    // Enable D-Cache
    SCB_EnableDCache();
#endif
    
#ifdef __ICACHE_PRESENT
    // Enable I-Cache
    SCB_EnableICache();
#endif
}

// Configure MPU
static void MPU_Config(void) {
    // MPU configuration for memory protection
    // This is optional but recommended for safety-critical applications
    
    // In real implementation:
    // 1. Disable MPU
    // 2. Configure regions (Flash, SRAM, Peripherals)
    // 3. Enable MPU
}

// Configure NVIC
static void NVIC_Config(void) {
    // Set priority grouping
    NVIC_SetPriorityGrouping(4); // 4 bits for preemption, 0 bits for subpriority
    
    // Configure ADC interrupt
    NVIC_SetPriority(ADC_IRQn, 5);
    NVIC_EnableIRQ(ADC_IRQn);
    
    // Configure systick interrupt
    NVIC_SetPriority(SysTick_IRQn, 0);
    
    // Configure USART interrupt if debug enabled
#if DEBUG_UART_ENABLED
    NVIC_SetPriority(USART2_IRQn, 6);
    NVIC_EnableIRQ(USART2_IRQn);
#endif
}

// Enable peripheral clocks
static void PeriphClock_Enable(void) {
    // Enable GPIO clocks
    // In STM32: RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | ...);
    
    // Enable ADC clock
    // RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Enable USART clock if debug enabled
#if DEBUG_UART_ENABLED
    // RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
#endif
    
    // Enable DMA clock if using DMA
    // RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
}

// Get system core clock
uint32_t SystemCoreClock = BOARD_CLOCK_HZ;

// Default systick handler
void SysTick_Handler(void) {
    // Increment system tick counter
    // In real implementation, update a global tick variable
}

// Default error handler
void Default_Handler(void) {
    while (1) {
        // Trap error - blink LED rapidly
        LED_Blink(LED_RED, 100);
    }
}

// Weak interrupt handlers (can be overridden)
__attribute__((weak)) void ADC_IRQHandler(void) {
    Default_Handler();
}

__attribute__((weak)) void USART2_IRQHandler(void) {
    Default_Handler();
}

__attribute__((weak)) void DMA2_Stream0_IRQHandler(void) {
    Default_Handler();
}

// Vector table (simplified)
__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    // Core exceptions
    (void (*)(void))((uint32_t)0x20010000), // Initial stack pointer
    System_Init,                            // Reset handler
    Default_Handler,                        // NMI handler
    Default_Handler,                        // Hard fault handler
    Default_Handler,                        // MPU fault handler
    Default_Handler,                        // Bus fault handler
    Default_Handler,                        // Usage fault handler
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    0,                                      // Reserved
    Default_Handler,                        // SVCall handler
    Default_Handler,                        // Debug monitor handler
    0,                                      // Reserved
    Default_Handler,                        // PendSV handler
    SysTick_Handler,                        // SysTick handler
    
    // External interrupts
    Default_Handler,                        // Window Watchdog
    Default_Handler,                        // PVD
    Default_Handler,                        // TAMP_STAMP
    Default_Handler,                        // RTC_WKUP
    Default_Handler,                        // Flash
    Default_Handler,                        // RCC
    Default_Handler,                        // EXTI0
    Default_Handler,                        // EXTI1
    Default_Handler,                        // EXTI2
    Default_Handler,                        // EXTI3
    Default_Handler,                        // EXTI4
    Default_Handler,                        // DMA1_Stream0
    Default_Handler,                        // DMA1_Stream1
    Default_Handler,                        // DMA1_Stream2
    Default_Handler,                        // DMA1_Stream3
    Default_Handler,                        // DMA1_Stream4
    Default_Handler,                        // DMA1_Stream5
    Default_Handler,                        // DMA1_Stream6
    Default_Handler,                        // ADC
    Default_Handler,                        // CAN1_TX
    Default_Handler,                        // CAN1_RX0
    Default_Handler,                        // CAN1_RX1
    Default_Handler,                        // CAN1_SCE
    Default_Handler,                        // EXTI9_5
    Default_Handler,                        // TIM1_BRK_TIM9
    Default_Handler,                        // TIM1_UP_TIM10
    Default_Handler,                        // TIM1_TRG_COM_TIM11
    Default_Handler,                        // TIM1_CC
    Default_Handler,                        // TIM2
    Default_Handler,                        // TIM3
    Default_Handler,                        // TIM4
    Default_Handler,                        // I2C1_EV
    Default_Handler,                        // I2C1_ER
    Default_Handler,                        // I2C2_EV
    Default_Handler,                        // I2C2_ER
    Default_Handler,                        // SPI1
    Default_Handler,                        // SPI2
    Default_Handler,                        // USART1
    USART2_IRQHandler,                      // USART2
    Default_Handler,                        // USART3
    Default_Handler,                        // EXTI15_10
    Default_Handler,                        // RTC_Alarm
    Default_Handler,                        // OTG_FS_WKUP
    Default_Handler,                        // TIM8_BRK_TIM12
    Default_Handler,                        // TIM8_UP_TIM13
    Default_Handler,                        // TIM8_TRG_COM_TIM14
    Default_Handler,                        // TIM8_CC
    Default_Handler,                        // DMA1_Stream7
    Default_Handler,                        // FSMC
    Default_Handler,                        // SDIO
    Default_Handler,                        // TIM5
    Default_Handler,                        // SPI3
    Default_Handler,                        // UART4
    Default_Handler,                        // UART5
    Default_Handler,                        // TIM6_DAC
    Default_Handler,                        // TIM7
    Default_Handler,                        // DMA2_Stream0
    DMA2_Stream0_IRQHandler,                // DMA2_Stream1
    Default_Handler,                        // DMA2_Stream2
    Default_Handler,                        // DMA2_Stream3
    Default_Handler,                        // DMA2_Stream4
    Default_Handler,                        // CAN2_TX
    Default_Handler,                        // CAN2_RX0
    Default_Handler,                        // CAN2_RX1
    Default_Handler,                        // CAN2_SCE
    Default_Handler,                        // OTG_FS
    Default_Handler,                        // DMA2_Stream5
    Default_Handler,                        // DMA2_Stream6
    Default_Handler,                        // DMA2_Stream7
    Default_Handler,                        // USART6
    Default_Handler,                        // I2C3_EV
    Default_Handler,                        // I2C3_ER
    Default_Handler,                        // OTG_HS_EP1_OUT
    Default_Handler,                        // OTG_HS_EP1_IN
    Default_Handler,                        // OTG_HS_WKUP
    Default_Handler,                        // OTG_HS
    Default_Handler,                        // DCMI
    Default_Handler,                        // CRYP
    Default_Handler,                        // HASH_RNG
    Default_Handler,                        // FPU
};
