#include "regulator.h"

PID_REGULATOR *heaterRegulator = new PID_REGULATOR
{
    0,
    0,
    I_MAX,
    I_MIN,
    P_GAIN,
    I_GAIN,
    D_GAIN
};

extern ADC_READ optimalCjConfig;

#define HEATER_PWM_PIN 33

void validateRegulatorIntegralState(PID_REGULATOR* regulator){
    if(regulator->integratorState > regulator->maxIntegratorState){
        regulator->integratorState = regulator->maxIntegratorState;
    }

    if(regulator->integratorState < regulator->minIntegratorState){
        regulator->integratorState = regulator->minIntegratorState;
    }
}

#define MAX_VALUE 255
#define MIN_VALUE 0

void validateRegulatorOutput(int16_t* output){
    if((*output) > MAX_VALUE) (*output) = MAX_VALUE;
    if((*output) < MIN_VALUE) (*output) = MIN_VALUE;
}

int16_t calculateHeaterOutput(uint16_t inputValue)
{
    int16_t error, position, result;
    float proportionalTerm, integralTerm, derivativeTerm;

    error = optimalCjConfig.UR - inputValue;
    position = inputValue;
    proportionalTerm = -heaterRegulator->proportionalGain * error;

    heaterRegulator->integratorState += error;

    validateRegulatorIntegralState(heaterRegulator);

    integralTerm = -heaterRegulator->integralGain * heaterRegulator->integratorState;
    derivativeTerm = -heaterRegulator->derivativeGain * (heaterRegulator->lastPosition - position);

    heaterRegulator->lastPosition = position;

    result = proportionalTerm + integralTerm + derivativeTerm;

    Serial.print("Result before validation: ");
    Serial.print(result);
    Serial.print('\n');

    validateRegulatorOutput(&result);

    Serial.print("Result after validation: ");
    Serial.print(result);
    Serial.print('\n');
    return result;
}

uint16_t adjustHeaterOutputPWM(ADC_READ data){

    if (data.UR < 500 || optimalCjConfig.UR != 0 || data.UB > MINIMUM_BATTERY_ADC_VALUE) {
        return calculateHeaterOutput(data.UR);
    } else {
        return 0;
    }
}

