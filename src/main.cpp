#include <Arduino.h>
#include "cj125.h"
#include "regulator.h"
#include "analog_write.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

uint32_t programTime = 0;
uint32_t readValuesIntervalTime = 0;
uint32_t displayValuesIntervalTime = 0;
uint16_t PWM;

ADC_READ cjValues;

String btOutputString;

BluetoothSerial BTSerial;

void startup()
{

  cj125Startup();
  cj125Calibration();
  cj125HeatSensor();
}

void setup() {
  Serial.begin(115200);
  BTSerial.begin("AFR");
  
  cj125PinInitialize();
  cj125PinSetup();
  cj125SpiInitalize();
  startup();
 
}

void loop() {
  
  programTime = millis();
  if(!isBatteryAlright()) {setHeaterPWM(0); startup();}
  
  if(programTime - readValuesIntervalTime > 10)
  {
    cjValues = readCjValues();
    PWM = adjustHeaterOutputPWM(cjValues);
    setHeaterPWM(PWM);
    if(programTime - displayValuesIntervalTime > 1000)
    {
      displayValues();
      btOutputString = assembleBtOutputString();
      BTSerial.print(btOutputString);
      delay(500);
    }
  }
}


