#ifndef TEMPERATURE_MONITOR_HPP
#define TEMPERATURE_MONITOR_HPP

#include "main.h"
#include <cstring>


class TemperatureMonitor {
public:
    TemperatureMonitor(I2C_HandleTypeDef& i2c, ADC_HandleTypeDef& adc, TIM_HandleTypeDef& timer);
    void init();
    void onADCConversionComplete();

private:
    I2C_HandleTypeDef& i2c;
    ADC_HandleTypeDef& adc;
    TIM_HandleTypeDef& timer;
    uint32_t rawADC;
    uint32_t hwRevision;
    uint32_t temperatureFactor;
    char hwSerialNo[20];
    char tempBuffer[PAGE_SIZE];

    void readEEPROM();
    void computeTemperatureFactor();
    void startADC();
    void startTimer();
    void setLEDs(GPIO_PinState green, GPIO_PinState yellow, GPIO_PinState red);
};

#endif
