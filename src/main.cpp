#include <Arduino.h>
#include "cj125.h"
#include "regulator.h"
#include "analog_write.h"


uint32_t programTime = 0;
uint32_t readValuesIntervalTime = 0;
uint32_t displayValuesIntervalTime = 0;
uint16_t PWM;

ADC_READ cjValues;


void setup() {
  Serial.begin(115200);
  
  

  cj125PinInitialize();
  cj125PinSetup();
  cj125SpiInitalize();

  // cj125Startup();
  // cj125Calibration();
  // cj125HeatSensor();

  // readCjValues();

  
}

void loop() {
  
  programTime = millis();
  if(!isBatteryAlright()) setup();
  
  if(programTime - readValuesIntervalTime > 20)
  {
    cjValues = readCjValues();
    PWM = adjustHeaterOutputPWM(cjValues);
    setHeaterPWM(PWM);
    if(programTime - displayValuesIntervalTime > 1000)
    {
      displayValues();
    }
  }
  delay(1000);
}


