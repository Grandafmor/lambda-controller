#include "cj125.h"
#include "SPI.h"
#include "logger.h"
#include "analog_write.h"
#include "conversion.h"
#include "BluetoothSerial.h"

SPIClass *hspi = NULL;
CJ125_RESPONSE responseStatus = STATUS_STARTUP;

static const uint32_t spiClk = 2000000; // 2 MHz
static uint32_t programTime = 0;
static uint32_t actionTime = 0;
static uint32_t pwmIntervalTime = 0;
uint16_t heaterPWM = 0;
float powerSupply = 0;

#define HEATER_PWM_PIN 33

ADC_READ optimalCjConfig = {0, 0, 0};
ADC_READ cjReadValues = {0, 0, 0};

int LEDstate = LOW;

void cj125PinInitialize()
{
  pinMode(UR_ANALOG_READ_PIN, INPUT);
  pinMode(UB_ANALOG_READ_PIN, INPUT);
  pinMode(UA_ANALOG_READ_PIN, INPUT);
  pinMode(HEATER_PWM_PIN, OUTPUT);
  pinMode(CJ125_SS, OUTPUT);
  logInfo("spi pin initialization complete");
}

void cj125PinSetup()
{
  analogSetPinAttenuation(UA_ANALOG_READ_PIN, ADC_11db);
  analogSetPinAttenuation(UB_ANALOG_READ_PIN, ADC_11db);
  analogSetPinAttenuation(UR_ANALOG_READ_PIN, ADC_11db);
}

void cj125SpiInitalize()
{
  SPI.begin(CJ125_SCLK, CJ125_MISO, CJ125_MOSI, CJ125_SS);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  digitalWrite(CJ125_SS, HIGH);
  logInfo("spi initialization succesfull");
}

void cj125Startup()
{
  logInfo("CJ125 StartUp");
  while (true)
  {
    programTime = millis();
    if (programTime - actionTime > 1000)
    {
      actionTime = programTime;
      responseStatus = cj125SendRequest(DIAGNOSTIC);
      cjReadValues.UB = analogRead(UB_ANALOG_READ_PIN);

      if (responseStatus == STATUS_NO_POWER)
        logError("Low power");
      else if (responseStatus == STATUS_NO_SENSOR)
        logError("No sensor");

      else if (responseStatus == STATUS_OK && cjReadValues.UB >= MINIMUM_BATTERY_ADC_VALUE)
      {
        logInfo("Device is ready");
        return;
      }
    }
  }
}

void cj125Calibration()
{
  logInfo("Starting calibration");
  cj125SendRequest(FIRST_INIT_CALIBRATION);

  delay(1000);

  optimalCjConfig.UA = (analogRead(UA_ANALOG_READ_PIN));
  optimalCjConfig.UR = (analogRead(UR_ANALOG_READ_PIN));

  cj125SendRequest(FIRST_INIT_MODE_17V);

  Serial.print("UA_Optimal (?? = 1.00): ");
  Serial.print((float)optimalCjConfig.UA / 4095 * 3.3 / 7800 * 23000 * 0.66);
  Serial.print(" (?? = ");
  Serial.print(translateLambdaValue(optimalCjConfig.UA), BIN);
  Serial.print(")\n\r");
  Serial.print("UR_Optimal: ");
  logInfo(String(optimalCjConfig.UR));
  Serial.print((float)optimalCjConfig.UR / 4095 * 3.3 / 7800 * 23000);
  Serial.print("\n\r");
}

void condensationPhase()
{
  powerSupply = (float)cjReadValues.UB / 4095 * 3.3 / 3300 * 18300;
  heaterPWM = (2 / powerSupply) * 255;
  setHeaterPWM(heaterPWM);
  logInfo(String(powerSupply));
  logInfo("Condensation phase please wait");
  delay(5000);

  logInfo("Done");
}

void rampUpPhase()
{
  float UHeater = 8.5;
  logInfo("Ramp Up phase please wait");
  while (UHeater < 13.0)
  {

    heaterPWM = (UHeater / powerSupply) * 255;

    setHeaterPWM(heaterPWM);

    UHeater += 0.4;
    logInfo(".");
    delay(1000);
  }
  logInfo("Done");
}

void optimalHeatingPhase()
{
  logInfo("Optimal heating phase");
  while (analogRead(UR_ANALOG_READ_PIN) > optimalCjConfig.UR)
  {
    logInfo(".");
    delay(1000);
  }
  setHeaterPWM(0);
  logInfo("Done");
}

void cj125HeatSensor()
{
  condensationPhase();
  rampUpPhase();
  optimalHeatingPhase();
}

boolean isAdcLambdaValueInRange(uint16_t data)
{
  return data >= MINIMUM_LAMBDA_ADC_VALUE && data <= MAXIMUM_LAMBDA_ADC_VALUE;
}

float translateLambdaValue(uint16_t data)
{
  logInfo(String(data));
  float result;

  if (isAdcLambdaValueInRange(data))
  {
    result = pgm_read_float_near(LAMBDA_CONVERSION_VALUE + (data - MINIMUM_LAMBDA_ADC_VALUE));
  }
  else if (data > MAXIMUM_LAMBDA_ADC_VALUE)
  {
    result = MAXIMUM_LAMBDA_VALUE;
  }
  else
  {
    result = MINIMUM_LAMBDA_VALUE;
  }

  return result;
}

boolean isAdcOxygenValueInRange(uint16_t data)
{
  return data >= MINIMUM_OXYGEN_ADC_VALUE && data <= MAXIMUM_OXYGEN_ADC_VALUE;
}

float translateOxygenValue(uint16_t data)
{
  float result = 0;

  if (isAdcOxygenValueInRange(data))
  {
    result = pgm_read_float_near(OXYGEN_CONVERSION_VALUE + (data - MINIMUM_OXYGEN_ADC_VALUE));
  }
  else if (data > MAXIMUM_OXYGEN_ADC_VALUE)
  {
    result = MAXIMUM_OXYGEN_VALUE;
  }
  else
  {
    result = MINIMUM_OXYGEN_VALUE;
  }
  return result;
}

CJ125_RESPONSE cj125SendRequest(CJ125_REQUEST data)
{
  CJ125_RESPONSE Response;

  digitalWrite(CJ125_SS, LOW);

  Response = static_cast<CJ125_RESPONSE>(SPI.transfer16(data));

  digitalWrite(CJ125_SS, HIGH);

  return Response;
}

ADC_READ readCjValues()
{
  responseStatus = cj125SendRequest(DIAGNOSTIC);

  cjReadValues.UA = analogRead(UA_ANALOG_READ_PIN);
  cjReadValues.UB = analogRead(UB_ANALOG_READ_PIN);
  cjReadValues.UR = analogRead(UR_ANALOG_READ_PIN);

  return cjReadValues;
}

boolean isBatteryAlright()
{
  if (cjReadValues.UB > MINIMUM_BATTERY_ADC_VALUE)
  {
    return true;
  }
  else
  {
    logInfo("Battery is low");
    return false;
  }
}

String assembleBtOutputString()
{
  const float LAMBDA_VALUE = translateLambdaValue(cjReadValues.UA);
  const float OXYGEN_CONTENT = translateOxygenValue(cjReadValues.UA);

  String btString = ":";
  btString += ",";

  if (responseStatus == STATUS_OK)
  {
    if (isAdcLambdaValueInRange(cjReadValues.UA))
    {
      btString += String(LAMBDA_VALUE, 2);
      btString += ",";
    }
    else
    {
      // btString += "-";
      btString += "oor";
      btString += ",";
    }

    if (isAdcOxygenValueInRange(cjReadValues.UA / 4))
    {
      btString += String(OXYGEN_CONTENT, 2);
      btString += ",";
    }
    else
    {
      btString += "oor";
      btString += ",";
    }
    btString += "Status OK";
  }
  if (responseStatus == STATUS_NO_POWER)
  {
    btString += "error";
    btString += ",";
    btString += "error";
    btString += ",";
    btString += "No power";
  }
  if (responseStatus == STATUS_NO_SENSOR)
  {
    btString += "error";
    btString += ",";
    btString += "error";
    btString += ",";
    btString += "No sensor";
  }
    btString += ",";
    btString += "#";
    return btString;
}

void displayValues()
{
  const float LAMBDA_VALUE = translateLambdaValue(cjReadValues.UA);
  const float OXYGEN_CONTENT = translateOxygenValue(cjReadValues.UA);

  if (responseStatus == STATUS_OK)
  {

    String txString = "Measuring";
    txString += ", UA_ADC: ";
    txString += String((float)cjReadValues.UA / 4095 * 3.3 / 7800 * 23000);
    txString += ", UR_ADC: ";
    txString += String((float)cjReadValues.UR / 4095 * 3.3 / 7800 * 23000);
    txString += ", UB_ADC: ";
    txString += String((float)cjReadValues.UB / 4095 * 3.3 / 3300 * 18300);

    if (isAdcLambdaValueInRange(cjReadValues.UA))
    {
      txString += ", Lambda: ";
      txString += String(LAMBDA_VALUE, 2);
    }
    else
    {
      txString += ", Lambda -";
    }

    if (isAdcOxygenValueInRange(cjReadValues.UA))
    {
      txString += ", Oxygen: ";
      txString += String(OXYGEN_CONTENT, 2);
      txString += "%";
    }
    else
    {
      txString += ", Oxygen - ";
    }

    logInfo(txString);
  }
}

void setHeaterPWM(uint16_t PWM)
{
  if (PWM > 255) PWM = 255;
  else if (PWM < 0) PWM = 0;
  analogWrite(HEATER_PWM_PIN, PWM);
}
