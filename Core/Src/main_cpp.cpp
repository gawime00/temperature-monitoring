#include "main.h"
#include "TemperatureMonitor.hpp"
#include <cstdio>

// Global peripheral handle declarations (as provided by HAL)
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;

// Global pointer for the TemperatureMonitor instance used in the ADC conversion callback.
TemperatureMonitor* monitor = nullptr;

// HAL ADC conversion complete callback
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// Making sure the the pointer exists already
    if (monitor) {
        monitor->onADCConversionComplete();
    }
}

int main() {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();

    // Create TemperatureMonitor object and initialize the system. Its static memory allocation to stay alive, which is important for the callback
    static TemperatureMonitor mon(hi2c1, hadc1, htim3);
    // Assigning the address of mon to the globally defined monitor, which is used in the callback
    monitor = &mon;
    // Starting the initialization
    monitor->init();

    // Main loop
    while (true) {
    }
}
