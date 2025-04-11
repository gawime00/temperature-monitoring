#include "TemperatureMonitor.hpp"
#include "main.h"
#include <cstring>

// Constructor: Initializes member variables and buffers using initialization List
TemperatureMonitor::TemperatureMonitor(I2C_HandleTypeDef& i2c, ADC_HandleTypeDef& adc, TIM_HandleTypeDef& timer)
    : i2c(i2c), adc(adc), timer(timer), rawADC(0), hwRevision(0), temperatureFactor(1)
{
    memset(hwSerialNo, 0, sizeof(hwSerialNo));
    memset(tempBuffer, 0, sizeof(tempBuffer));
}

// Initialization routine: Read EEPROM data, compute the temperature factor, and start peripherals.
void TemperatureMonitor::init() {
    readEEPROM();
    computeTemperatureFactor();
    startADC();
    startTimer();
}

// ADC conversion callback handler: Uses ADC value to control LEDs based on temperature thresholds.
void TemperatureMonitor::onADCConversionComplete() {
    if ((rawADC >= (105 * temperatureFactor)) || (rawADC < (5 * temperatureFactor))) {
        // Turn on Red LED, turn off Green and Yellow LEDs.
        setLEDs(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
    } else if (rawADC >= (85 * temperatureFactor)) {
        // Turn on Yellow LED, turn off Red and Green LEDs.
        setLEDs(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
    } else {
        // Turn on Green LED, turn off Red and Yellow LEDs.
        setLEDs(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
}

// Reads hardware revision and serial number from the EEPROM.
// The hardware revision is stored in the first 4 bytes, and the serial number in the second page.
void TemperatureMonitor::readEEPROM() {
    // Read 4 bytes from EEPROM at address 0x0000 for hardware revision.
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, 0x0000,I2C_MEMADD_SIZE_16BIT, (uint8_t*) &hwRevision, 4, HAL_I2C_ERROR_TIMEOUT);

    // Read 20 bytes (serial number) from the second EEPROM page.
    HAL_I2C_Mem_Read(&i2c, EEPROM_ADDR, PAGE_SIZE, I2C_MEMADD_SIZE_16BIT,
                     (uint8_t*)hwSerialNo, 20, HAL_I2C_ERROR_TIMEOUT);
}

// Computes the temperature factor based on the hardware revision.
void TemperatureMonitor::computeTemperatureFactor() {
    switch (hwRevision) {
        case 0:
            temperatureFactor = 1;
            break;
        case 1:
            temperatureFactor = 10;
            break;
        default:
            temperatureFactor = 1;
            break;
    }
}

// Starts the ADC with DMA transfer.
void TemperatureMonitor::startADC() {
    HAL_ADC_Start_DMA(&adc, &rawADC, 1);
}

// Starts the timer that triggers the ADC conversion (PWM mode).
void TemperatureMonitor::startTimer() {
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
}

// Helper function to update LED states.
void TemperatureMonitor::setLEDs(GPIO_PinState green, GPIO_PinState yellow, GPIO_PinState red) {
    HAL_GPIO_WritePin(GPIOA, LED_Green_Pin, green);
    HAL_GPIO_WritePin(GPIOA, LED_Yellow_Pin, yellow);
    HAL_GPIO_WritePin(GPIOA, LED_Red_Pin, red);
}
