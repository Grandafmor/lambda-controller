#ifndef REGULATOR_H
#define REGULATOR_H

#include <Arduino.h>
#include "cj125.h"

#define I_MAX   250
#define I_MIN   -250
#define P_GAIN  120
#define I_GAIN  0.8
#define D_GAIN  10

#define HEATER_PWM_PIN        33  

typedef struct _regulator_
{
    int16_t lastPosition;
    int16_t integratorState;
    int16_t maxIntegratorState;
    int16_t minIntegratorState;
    float proportionalGain;
    float integralGain;
    float derivativeGain;
}PID_REGULATOR;

int16_t calculateHeaterOutput(uint16_t inputValue);

uint16_t adjustHeaterOutputPWM(ADC_READ data);

#endif