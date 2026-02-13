/*
* C file for a remote Heltec ESP32 LoRa Module
* This file contains 2 modes of operation.
* Mode 1 - Manual: The module acts as a receiver (RX), reading 
* packets of input data from TX.
* Mode 2 - Analysis (Auto): The module acts a transmitter (TX), sending 
* packets of data containing metrics to a local module (RX).
* * Battery Monitor Wiring: *
* [Battery +]---[10k]---|
* [GPIO2]
* [Battery -]---[2k]----|
* Last updated: 2/11/2026
* Author: JoseAngel Socorro Pulido
*/
#include "heltec_unofficial.h"

#define FREQ 915.0  // Frequency band (US: 915E6)
#define BW   125.0  // Bandwidth
#define SF   7      // Spreading Factor (lower -> faster)
#define CR   5      // Coding Rate
#define STICK_THRESH 0.3  // threshold for detecting movement

// Battery Monitor Settings ====================
const int BAT_PIN = 2;  // GPIO 2 (ADC1_CH1)
const float R1 = 10000.0;  // 10k Ohm
const float R2 = 2000.0;  // 2k Ohm
const float V_REF = 3.3; 

// Low Battery Safety Settings =================
// 10.5V is approx 3.5V per cell for a 3S LiPo.
// This leaves enough headroom for the Pi to shut down safely.
#define LOW_BAT_THRESH 10.5  
unsigned long lowBatTimer = 0;
bool shutdownTriggered = false;
const int LOW_BAT_TIMEOUT = 5000; // 5 seconds of sustained low voltage

// Function Protypes ===========================
void updateDisp(String dir, float x, float y, float voltage);
void fixDisp();
float getVoltage();
void checkBatteryHealth(float voltage);


void setup() {
  heltec_setup();
  
  // Init Radio
  Serial.print("RX Setup... ");
  int state = radio.begin(FREQ, BW, SF, CR);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Success!");
  } else {
    while (true);
  }

  // ADC Init
  analogSetAttenuation(ADC_11db);  // Allows reading up to ~3.3V
  
  fixDisp();

  heltec_led(100);  // turn on LED 100%

  display.clear();
  display.drawString(0, 0, "RX ready");
  display.drawString(0, 15, "Waiting for signal...");
  display.drawString(0, 53, "Battery: " + String(getVoltage(), 1) + "V");
  display.display();
}

void loop() {
  heltec_loop();

  String cmd;
  
  // Always monitor battery, even if no packets are coming in
  float currentVolts = getVoltage();
  checkBatteryHealth(currentVolts);

  // If we already triggered shutdown, stop processing loop to avoid interference
  if (shutdownTriggered) {
    heltec_led(50); // Dim LED to indicate "Done"
    delay(1000);
    return; 
  }

  // Check for incoming LoRa packet
  int state = radio.receive(cmd);

  if (state == RADIOLIB_ERR_NONE) {
    // Packet Received!

    cmd.trim();
    Serial.println(cmd);

    // Read Shutdown 
    if (cmd == "CMD:SHUTDOWN") {
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 25, "Shutting Down...");
      display.display();

      delay(3000);  // delay 3 seconds
    }
    // Read Mode Toggle 
    else if (cmd == "CMD:MODE_AUTO" || cmd == "CMD:MODE_MANUAL") {
      display.clear();
      display.setFont(ArialMT_Plain_16);

      if (cmd == "CMD:MODE_AUTO") {
        display.drawString(0, 0, "AUTO NAV");
        display.drawString(0, 25, "Running...");
      }
      else {
        display.drawString(0, 0, "MANUAL NAV");
        display.drawString(0, 25, "Use D-Pad or Left Joystick...");
      }
      display.display();

      // Flash LED twice
      heltec_led(0); delay(45);
      heltec_led(100); delay(45);
      heltec_led(0); delay(45);
      heltec_led(100); 

      delay(2000);  // delay 2 seconds
    }
    // Read Movement 
    else if (cmd.startsWith("M:")) {
      // parse e.g "M:-0.50,0.80"
      int commaidx = cmd.indexOf(',');
      if (commaidx != -1) {
        String xStr = cmd.substring(2, commaidx);
        String yStr = cmd.substring(commaidx + 1);
        float x = xStr.toFloat();
        float y = yStr.toFloat();

        // Decide Direction
        String dir = "STOP";
        if (abs(x) > STICK_THRESH || abs(y) > STICK_THRESH) {
          if (abs(x) > abs(y)) {
            dir = (x < 0) ? "LEFT" : "RIGHT";  // if x < 0: left | else: right
          }
          else {
            dir = (y < 0) ? "UP" : "DOWN";
          }
        }
        updateDisp(dir, x, y, currentVolts);
      }
    }
    // Raw message backup
    else {
      
        display.clear();
        display.drawString(0, 0, "Raw Msg:");
        display.drawString(0, 15, cmd);
        display.display();
    }
    
    delay(50);  // delay for blink
    heltec_led(100);  // turn LED solid white
  }
}

float getVoltage() {
  long sum = 0;
  // Take 10 samples (smoothes noise)
  for(int i=0; i<10; i++){
    sum += analogRead(BAT_PIN);
    delay(1);
  }
  float avg_adc = sum / 10.0;

  // Compute Voltage at Pin (0 - 3.3V)
  float pin_voltage = avg_adc * (V_REF / 4095.0);

  // Compute Actual Battery Voltage
  float bat_voltage = pin_voltage * ((R1 + R2) / R2);

  return bat_voltage;
}

void checkBatteryHealth(float voltage) {
  // If voltage is critically low
  if (voltage < LOW_BAT_THRESH) {
    
    // Start timer if not started
    if (lowBatTimer == 0) {
      lowBatTimer = millis();
    }
    
    // If timer exceeds safety window (5 seconds)
    if (millis() - lowBatTimer > LOW_BAT_TIMEOUT && !shutdownTriggered) {
      shutdownTriggered = true;

      // 1. Tell the Pi to shutdown
      Serial.println("CMD:SHUTDOWN");

      // 2. Alert on OLED
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "!!! CRITICAL BAT !!!");
      display.drawString(0, 20, "Voltage: " + String(voltage, 2) + "V");
      display.drawString(0, 40, "Shutting Down Pi...");
      display.display();
      
      // 3. Alert on LED (Red strobe effect via brightness if standard led)
      // Heltec built-in LED is single color usually, so we just blink fast
      for(int i=0; i<10; i++) {
        heltec_led(0); delay(100);
        heltec_led(100); delay(100);
      }
    }
  } else {
    // Reset timer if voltage recovers (e.g. motor stopped)
    lowBatTimer = 0;
  }
}

void updateDisp(String dir, float x, float y, float voltage) {
  /* Draws the UI */

  display.clear();
  
  // Main Direction
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, dir);
  
  // Debug Stats
  display.setFont(ArialMT_Plain_10);
  String stats = "X: " + String(x, 2) + "  Y: " + String(y, 2);
  display.drawString(0, 30, stats);
  
  // Signal Strength
  String rssi = "RSSI: " + String(radio.getRSSI());
  display.drawString(0, 42, rssi);

  // Battery Voltage
  // Mapping 3S Lipo Range: 11.1V (0%) to 12.6V (100%)
  int pct = map(voltage * 100, 1110, 1260, 0, 100);
  pct = constrain(pct, 0, 100);

  String batStr = "Battery: " + String(voltage, 1) + "V (" + String(pct) + "%)";
  display.drawString(0, 53, batStr);
  
  display.display();
}

void fixDisp() {
  /* Forces VEXT on and manually overrides I2C to correct dim OLED */
  pinMode(36, OUTPUT); 
  digitalWrite(36, LOW); // Active LOW
  delay(50); 

  Wire.beginTransmission(0x3c);
  Wire.write(0x80); Wire.write(0x8D); // charge pump
  Wire.write(0x80); Wire.write(0x14); // enable 
  Wire.endTransmission();
  
  Wire.beginTransmission(0x3c);
  Wire.write(0x80); Wire.write(0xAF); // display ON
  Wire.endTransmission();
}