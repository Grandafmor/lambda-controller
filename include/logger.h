#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "cj125.h"


void logError(CJ125_RESPONSE);

void logInfo(CJ125_RESPONSE);

void logError(String message);

void logInfo(String message);

void logInfo(CJ125_REQUEST);

#endif
