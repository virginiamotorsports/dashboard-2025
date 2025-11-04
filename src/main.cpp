// dash.cpp
#include "dash.h"
#include "ecu_dash2.h"
#include <string.h>
#include <stdio.h>     
#include <FlexCAN_T4.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>



// had to put this here for it to compile lolz
void playRTDAlert();


IntervalTimer timer;            
IntervalTimer ledTimer;           
bool ecuWakeTriggered = false;    
unsigned long bootTime = 0;       

// — Definitions of external globals —
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel strip(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// — CAN state —
struct ecu_dash2_motor_status_t motorStatus;
struct ecu_dash2_fault_status_t faultStatus;
struct ecu_dash2_car_status_t   carStatus; // renamed from checksStatus
struct ecu_dash2_cooling_t coolingStatus; 
struct ecu_dash2_bms_info_t     bmsInfo;
bool prevRTD = false;
uint16_t motorRPM = 0;
float batteryPercent = 0.0f;


// — CAN bus object —
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANbus;

// debug 
static uint16_t lastCanIds[3] = {0, 0, 0};


// set clearPin for buzzer interrupt
void clearPin() {
  digitalWrite(BUZZER_PIN, LOW);
  timer.end();
}

void setup() {
  pinMode(ECU_WAKE_PIN, OUTPUT);
  digitalWrite(ECU_WAKE_PIN, LOW);
  bootTime = millis();


  strip.begin();
  strip.setBrightness(4);
  strip.show();

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting...");
  display.display();

  pinMode(BUZZER_PIN, OUTPUT);

  CANbus.begin();
  CANbus.setBaudRate(500000);

  // start serial logging 
  Serial.begin(115200);
  Serial.println("Serial logging started");

  //initialize CAN message structs
  ecu_dash2_motor_status_init(&motorStatus);
  ecu_dash2_fault_status_init(&faultStatus);
  ecu_dash2_car_status_init(&carStatus);
  ecu_dash2_bms_info_init(&bmsInfo);
  ecu_dash2_cooling_init(&coolingStatus);

}


void loop() {
  unsigned long now = millis();

    // ─── Debug: loop tick ───────────────────────────
  Serial.print("loop @ ");
  Serial.print(now);
  Serial.println(" ms");

  
  // Wake ECU after delay
  if (!ecuWakeTriggered && now - bootTime >= ECU_WAKE_DELAY_MS) {
    digitalWrite(ECU_WAKE_PIN, HIGH);
    ecuWakeTriggered = true;
  }

  // Read incoming CAN frames
  CAN_message_t msg;
  while (CANbus.read(msg)) {
    // print raw CAN frame to computer terminal 
    Serial.print("RX CAN ID=0x");
    Serial.print(msg.id, HEX);
    Serial.print(" len=");
    Serial.print(msg.len);
    Serial.print(" data=");
    for (int i = 0; i < msg.len; i++) {
      Serial.print(msg.buf[i], HEX);
      Serial.print(' ');
    }
    Serial.println();


    //  Debug: push into last‐IDs buffer 
    lastCanIds[2] = lastCanIds[1];
    lastCanIds[1] = lastCanIds[0];
    lastCanIds[0] = msg.id;


    switch (msg.id) {
      case ECU_DASH2_MOTOR_STATUS_FRAME_ID:
        ecu_dash2_motor_status_unpack(&motorStatus, msg.buf, msg.len);
        motorRPM = motorStatus.inv_fast_motor_speed;
        break;

      case ECU_DASH2_FAULT_STATUS_FRAME_ID:
        ecu_dash2_fault_status_unpack(&faultStatus, msg.buf, msg.len);
        break;

      case ECU_DASH2_CAR_STATUS_FRAME_ID:
        ecu_dash2_car_status_unpack(&carStatus, msg.buf, msg.len);
      
        if (prevRTD == 0 && carStatus.rtd == 1) {
          playRTDAlert();  // Rising edge: 0 -> 1
        }

        prevRTD = carStatus.rtd;  // Update state regardless
        break;

      case ECU_DASH2_BMS_INFO_FRAME_ID:
        ecu_dash2_bms_info_unpack(&bmsInfo, msg.buf, msg.len);
        batteryPercent = bmsInfo.battery_percentage / 100.0f;
        break;

      case ECU_DASH2_COOLING_FRAME_ID:
        ecu_dash2_cooling_unpack(&coolingStatus, msg.buf, msg.len);
        break;
    }
  }


  //  Debug: dump last three CAN IDs 
  Serial.print("Last IDs = [0x");
  Serial.print(lastCanIds[0], HEX);
  Serial.print(", 0x");
  Serial.print(lastCanIds[1], HEX);
  Serial.print(", 0x");
  Serial.print(lastCanIds[2], HEX);
  Serial.println("]");


  updateOLED(motorRPM,
             batteryPercent,
             faultStatus,
             carStatus);

  updateNeoPixels(batteryPercent,
                  faultStatus);

  delay(10);
}

void playRTDAlert() {
  // JARVIS - ENTER READY TO DRIVE
  digitalWrite(BUZZER_PIN, HIGH);
  timer.begin(clearPin, 2000000); // buzz for two seconds
}



// OLED update
static const uint16_t MAX_MPH     = 85;
static const int     BAR_MAX_WIDTH = 126;
static const int     RPM_BAR_Y     = 20;
static const int     BAR_BORDER_H  = 16;
static const int     BAR_FILL_H    = 14;
// smoothing constants:
static const float   RPM_SMOOTH_RISE = 0.25f;  // fast rise
static const float   RPM_SMOOTH_FALL = 0.05f;  // slow fall

void updateOLED(uint16_t rpm,
                float batteryPercent,
                const struct ecu_dash2_fault_status_t &faults,
                const struct ecu_dash2_car_status_t &carStatus) {

  display.clearDisplay();


  //intialize gear ratio and wheel diameter
  static const float GEAR_RATIO = 3.273f;
  static const float WHEEL_DIAMETER_M = .4064f;
  static const float MPS_TO_MPH = 2.23694f;

  // --- Numeric speed in mph ---
  float wheelRps = (rpm / GEAR_RATIO) / 60.0f;            // rev/sec
  float wheelCirc = WHEEL_DIAMETER_M * M_PI;             // m/rev
  float speedMps = wheelRps * wheelCirc;                 // m/s
  int   speedMph = (int)roundf(speedMps * MPS_TO_MPH);   // mph
 

  // — apply smoothing for the RPM bar —        idk claude and chat say its a good idea i test difference with and without during testing
  static float displayMph = 0.0f;
  float targetMph = speedMph;
  float alpha = (targetMph > displayMph)
                  ? RPM_SMOOTH_RISE
                  : RPM_SMOOTH_FALL;
  displayMph += (targetMph - displayMph) * alpha;

  // set textSize
  int textSize = 2;
  display.setTextSize(textSize);

  // --- MPH text (above bar, centered) ---
  display.setTextSize(2);
  char buf[16];
  snprintf(buf, sizeof(buf), "%d rpm", speedMph);
  int16_t len    = strlen(buf);
  int16_t textW  = len * 6 * 2;             // 6px per char × size 2
  int16_t xText  = (128 - textW) / 2;
  int16_t yText  = RPM_BAR_Y - (8 * 2) - 2;  // 8px char height × size2 + margin
  display.setCursor(xText, yText);
  display.print(buf);

  // — MPH bar —
  display.drawRect(0, RPM_BAR_Y, 128, BAR_BORDER_H, SSD1306_WHITE);
  int16_t rpmBarW = (int32_t)displayMph * BAR_MAX_WIDTH / MAX_MPH;
  display.fillRect(1, RPM_BAR_Y + 1, rpmBarW, BAR_FILL_H, SSD1306_WHITE);
   
  //display fault bottom right if any  
  char fbuf[32] = "";
  if (faults.imd_fault)  strcat(fbuf, "IMD ");
  if (faults.bppc_fault) strcat(fbuf, "BPPC ");
  if (faults.apps_fault) strcat(fbuf, "APPS ");
  if (faults.bmsc_fault) strcat(fbuf, "BMSC ");
  // trim trailing space
  int flen = strlen(fbuf);
  if (flen > 0 && fbuf[flen-1] == ' ') fbuf[len-1] = '\0';

  if (fbuf[0]) {
    display.setTextSize(2);
    int textWidth = strlen(fbuf) * 6;            // 6px per char at size 1
    int x = 128 - textWidth - 1;                 // 1px padding from right
    int y = 64 - 8 - 1;                          // 8px tall font + 1px padding
    display.setCursor(x, y);
    display.print(fbuf);
  }


}

void updateNeoPixels(float batteryPercent,
                     const struct ecu_dash2_fault_status_t &faults) {
  strip.clear();

  // Battery pixels 4–11
  float pts = batteryPercent * 8.0f;
  for (int i = 0; i < 8; i++) {
    float pb = constrain(pts - i, 0.0f, 1.0f);
    uint8_t r = 0, g = 0, b = 0;
    if (i == 0)        r = 255 * pb;
    else if (i <= 2) { r = 255 * pb; g = 150 * pb; }
    else               g = 255 * pb;
    strip.setPixelColor(5 + i, strip.Color(r, g, b));
  }

  // Critical faults: IMD (13), BSPD (16)
  if (faults.imd_fault)  strip.setPixelColor(13, strip.Color(255, 0, 0));
  if (faults.bppc_fault) strip.setPixelColor(16, strip.Color(255, 0, 0));

  // Non-critical faults: APPS (14), BMSC (15)
  if (faults.apps_fault) strip.setPixelColor(14, strip.Color(255, 150, 0));
  if (faults.bmsc_fault) strip.setPixelColor(15, strip.Color(255, 150, 0));

  strip.show();
}
