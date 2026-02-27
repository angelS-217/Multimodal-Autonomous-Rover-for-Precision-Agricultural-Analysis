/*
* C file for a local Heltec ESP32 LoRa Module
* This file contains 2 modes of operation
* Mode 1 - Manual: The module acts as a transmitter (TX), sending 
*   input data from a controller to a remote module RX.
* Mode 2 - Analysis (Auto): The module acts a receiver (RX), reading 
*   packets of data containing metrics, sent from a remote module TX.
* 
* Last updated: 1/10/2026
* Author: JoseAngel Socorro Pulido
*/
#include "heltec_unofficial.h" 

#define FREQ 915.0  // Frequency band (MHz) (US: 915)
#define BW   500.0  // Bandwidth (kHz)
#define SF   7      // Spreading Factor (lower -> faster)
#define CR   5      // Coding Rate
#define STICK_THRESH 0.3  // Threshold for detecting movement

void fixDisp();
void updateDisp(String dir, float x, float y);
void readAndDisp(String cmd);

void setup() {
  // Handles Serial, Display, Radio Power
  heltec_setup(); 

  Serial.begin(115200);  // standard serial (BAUD:115200)

  // Initialize LoRa
  Serial.print("Initializing Radio... ");
  int state = radio.begin(FREQ, BW, SF, CR);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Success!");
  } 
  else {
    Serial.print("Failed, code ");
    Serial.println(state);
    while (true); // Stop if radio fails
  }

  fixDisp();
  heltec_led(100);  // LED solid white when waiting

  // Display Message
  display.clear();
  display.drawString(0, 0, "TX ready");
  display.drawString(0, 15, "Waiting for signal...");
  display.display();
}

void loop() {
  heltec_loop(); // Handles button updates if needed

  // Read from Python Script
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() > 0) {

      // Shutdown Logic
      if (cmd == "CMD:SHUTDOWN") {
        heltec_led(0);  // LED off

        // 1. Send shutdown command over LoRa
        radio.transmit(cmd);

        // 2. Show Alert on Screen
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "!!! Shutting Down...!!!");
        display.display();

        delay(2000);  // Hold screen to see message
        heltec_led(100);  // LED on
      }
      // Mode Switch Logic
      else if (cmd == "CMD:MODE_AUTO" || cmd == "CMD:MODE_MANUAL") {
        heltec_led(0);  // LED off

        // 1. Send Mode Switch Command
        radio.transmit(cmd);

        // 2. Update Display depending on Mode
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, "*** Changing Mode ***");

        if (cmd == "CMD:MODE_AUTO") {
          display.drawString(0, 20, "AUTO NAV");
        }
        else {
          display.drawString(0, 20, "MANUAL NAV");
        }
        display.display();

        delay(1500);
        heltec_led(100);
      }
      // Send Manual Movement Logic
      else {
        heltec_led(0);  // LED off
        radio.transmit(cmd);  // Send data packet
        readAndDisp(cmd);  // Visual Feedback
        delay(20);
        heltec_led(100);  // LED on
      }
    }
  }
}

void readAndDisp(String cmd) {
  /* Helper to interpret gamepad command string and display on OLED */

  if (cmd.startsWith("M:")) {
    int commaIdx = cmd.indexOf(',');
    if (commaIdx != -1) {
        String xStr = cmd.substring(2, commaIdx);
        String yStr = cmd.substring(commaIdx + 1);
        float x = xStr.toFloat();
        float y = yStr.toFloat();

        // Determine Direction
        String dir = "STOP";
        if (abs(x) > STICK_THRESH || abs(y) > STICK_THRESH) {
            if (abs(x) > abs(y)) {
                dir = (x < 0) ? "LEFT" : "RIGHT";
            } else {
                dir = (y < 0) ? "UP" : "DOWN";
            }
        }
        updateDisp(dir, x, y);
    }
  } else {
      // Fallback for non-movement commands (e.g., "STOP" or raw text)
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Sending:");
      display.drawString(0, 20, cmd);
      display.display();
  }
}

void updateDisp(String dir, float x, float y) {
  /* Helper to update OLED */

  display.clear();

  // Line 1: Direction (big)
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "TX: " + dir);
  
  // Line 2: Raw Values (small)
  display.setFont(ArialMT_Plain_10);
  String stats = "X: " + String(x, 2) + "  Y: " + String(y, 2);
  display.drawString(0, 30, stats);
  
  // Line 3: Status
  display.drawString(0, 45, "Status: Transmitting");
  
  display.display();
}

void fixDisp() {
  /*
  * Helper to manually force the VEXT and override the I2C on the ESP32 to
  * display the brightness at its original factory settings (really bright!)
  */

  // 1. Force External Power (VEXT) ON
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

  // pinMode(36, OUTPUT); 
  // digitalWrite(36, LOW); // Active LOW
  // delay(50); 

  // // 2. Manual I2C Override
  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0xFD); // command lock
  // Wire.write(0x80); Wire.write(0x12); // unlock (just in case)
  // Wire.endTransmission();
  // delay(10);

  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0xAE); // display OFF (sleep)
  // Wire.endTransmission();
  // delay(10);

  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0x8D); // charge pump setting
  // Wire.write(0x80); Wire.write(0x14); // enable 
  // Wire.endTransmission();
  // delay(10);
  
  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0x81); // contrast control
  // Wire.write(0x80); Wire.write(0xFF); // max (255)
  // Wire.endTransmission();

  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0xD9); // pre-charge period
  // Wire.write(0x80); Wire.write(0xF1); // max
  // Wire.endTransmission();

  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0xDB); // VCOMH Deselect
  // Wire.write(0x80); Wire.write(0x40); // max brightness
  // Wire.endTransmission();

  // Wire.beginTransmission(0x3c);
  // Wire.write(0x80); Wire.write(0xAF); // display ON (wake)
  // Wire.endTransmission();
}