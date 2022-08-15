#include "logger.h"

String programTimeStamp()
{
    uint32_t time = millis() / 1000;
    uint8_t hours = time / 3600;
    uint8_t mins = (time - (hours * 3600)) / 60;
    uint8_t seconds = time - (hours * 3600) - (mins * 60);
    String timeStampTemplate = "[" + String(hours) + ":" + String(mins) +  ":" + String(seconds) + "] - ";

    return timeStampTemplate;
}

void logError(CJ125_RESPONSE response)
{
     Serial.print(programTimeStamp());
     Serial.print("ERROR, CJ125: 0x");
     Serial.print(response, HEX);
     Serial.print("\n\r");
}

void logError(String message)
{
     Serial.print(programTimeStamp());
     Serial.print("ERROR, ");
     Serial.print(message);
     Serial.print("\n\r");
}

void logInfo(CJ125_RESPONSE response)
{
     Serial.print(programTimeStamp());
     Serial.print("INFO, CJ125: 0x");
     Serial.print(response, HEX);
     Serial.print("\n\r");
}

void logInfo(CJ125_REQUEST request)
{
     Serial.print(programTimeStamp());
     Serial.print("INFO, CJ125: 0x");
     Serial.print(request, HEX);
     Serial.print("\n\r");
}

void logInfo(String message){
    Serial.print(programTimeStamp());
    Serial.print("INFO, ");
    Serial.print(message);
    Serial.print("\n\r");
}


