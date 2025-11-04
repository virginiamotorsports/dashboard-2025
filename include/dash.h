// Dashboard.h ============= Hatcher Ring
#ifndef DASH_H
#define DASH_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IntervalTimer.h>             // Added so timer types are known - avoid using millis all the time

// CAN unpacker
#include "ecu_dash2.h"

// === OLED Setup ===
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDR      0x3D
extern Adafruit_SSD1306 display;

#define NEOPIXEL_PIN        20
#define NEOPIXEL_COUNT      16
#define NEOPIXEL_BRIGHTNESS 20
extern Adafruit_NeoPixel strip;

#define BATTERY_LEVEL_CYCLE_MS  2000
#define ECU_WAKE_PIN        21
#define ECU_WAKE_DELAY_MS   50 //??????????????????????????????????????????????????????????????????????????????
#define BUZZER_PIN          12

extern bool                ecuWakeTriggered;
extern unsigned long       bootTime;
extern IntervalTimer       timer;



// — CAN-state globals —
extern struct ecu_dash2_motor_status_t motorStatus;
extern struct ecu_dash2_fault_status_t faultStatus;
extern struct ecu_dash2_car_status_t   carStatus;    // renamed from checksStatus
extern bool                            prevRTD;
extern uint16_t                        motorRPM;
extern float                           batteryPercent;
extern struct ecu_dash2_bms_info_t     bmsInfo;
extern struct ecu_dash2_cooling_t      coolingStatus;


#endif // DASH_H
