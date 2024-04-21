#ifndef __TEST_H__
#define __TEST_H__
#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <SPI.h>
#include <esp32-hal-log.h>

void Motor1_P1();
void Motor2_P1();
void Motor3_P1();
void Motor1_P2();
void Motor2_P2();
void Motor3_P2();
void Motor1_Stop();
void Motor2_Stop();
void Motor3_Stop();
void Motors_Stop();

#endif