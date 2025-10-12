// Dashboard.h ============= Hatcher Ring
#ifndef DASH_H
#define DASH_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IntervalTimer.h>             // Added so timer types are known

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

extern const int           ECU_WAKE_PIN;
extern const unsigned long ECU_WAKE_DELAY_MS;
extern bool                ecuWakeTriggered;
extern unsigned long       bootTime;

extern const uint8_t       BUZZER_PIN;
extern IntervalTimer       timer;

extern const int           LED_PINS[];
extern const int           NUM_LEDS;
extern IntervalTimer       ledTimer;
extern volatile int        currentLedIndex;

extern const unsigned long BATTERY_LEVEL_CYCLE_MS;
extern unsigned long       batteryStartTime;

// — CAN-state globals —
extern struct ecu_dash2_motor_status_t motorStatus;
extern struct ecu_dash2_fault_status_t faultStatus;
extern struct ecu_dash2_car_status_t   carStatus;    // renamed from checksStatus
extern bool                            prevRTD;
extern uint16_t                        motorRPM;
extern float                           batteryPercent;
extern struct ecu_dash2_bms_info_t     bmsInfo;

// === Display / LED update functions ===
void updateOLED(uint16_t rpm,
                float batteryPercent,
                const struct ecu_dash2_fault_status_t &faults,
                const struct ecu_dash2_car_status_t &carStatus);

void updateNeoPixels(float batteryPercent,
                     const struct ecu_dash2_fault_status_t &faults);

void toggleNextLED();  // LED ISR

#endif // DASH_H
