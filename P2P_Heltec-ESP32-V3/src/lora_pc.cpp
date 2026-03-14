/*
* C file for a local Heltec ESP32 LoRa Module (Base Station Controller)
* This file contains 2 modes of operation
* Mode 1 - Manual: The module acts as a transmitter (TX), sending 
* input data from a controller to a remote module RX.
* Mode 2 - Analysis (Auto): The module acts a receiver (RX), reading 
* packets of data containing metrics, sent from a remote module TX.
* * Last updated: 3/1/2026
* Author: JoseAngel Socorro Pulido
*/
#include "heltec_unofficial.h" 

#define FREQ 915.0  // Frequency band (MHz) (US: 915)
#define BW   500.0  // Bandwidth (kHz)
#define SF   7      // Spreading Factor (lower -> faster)
#define CR   5      // Coding Rate
#define STICK_THRESH 0.3  // threshold for detecting movement

void fixDisp();
void updateDisp(String dir, float x, float y);
void readAndDisp(String cmd);

void setup() {
  heltec_setup(); 
  Serial.begin(115200); 

  Serial.print("Initializing Radio... ");
  int state = radio.begin(FREQ, BW, SF, CR);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Success!");
  } 
  else {
    Serial.print("Failed, code ");
    Serial.println(state);
    while (true); 
  }

  fixDisp();
  heltec_led(100); 

  display.clear();
  display.drawString(0, 0, "Base Station");
  display.drawString(0, 15, "Ready & Listening...");
  display.display();

  radio.startReceive();
}

void loop() {
  heltec_loop(); 

  // ===================================================================
  // 1. Check for incoming LoRa packet (Telemetry from Rover)
  // ===================================================================
  String incomingMsg;
  int rxState = radio.readData(incomingMsg);
  
  if (rxState == RADIOLIB_ERR_NONE) {
    incomingMsg.trim();
    if (incomingMsg.startsWith("LOG:")) {
      Serial.println(incomingMsg);
      heltec_led(0); delay(20); heltec_led(100);
    }
    radio.startReceive();
  }

  // ===================================================================
  // 2. Check for incoming USB Serial commands (Joystick from Laptop)
  // ===================================================================
  if (Serial.available()) {
    
    // Read the single freshest command without overwriting
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() > 0) {

      // Shutdown Logic
      if (cmd == "CMD:SHUTDOWN") {
        heltec_led(0); 
        radio.transmit(cmd);
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 20, "!!! Shutting Down...!!!");
        display.display();
        // REMOVED: delay(2000); 
        heltec_led(100); 
        radio.startReceive(); 
      }
      
      // Mode Switch Logic
      else if (cmd == "CMD:MODE_AUTO" || cmd == "CMD:MODE_MANUAL") {
        heltec_led(0); 
        radio.transmit(cmd);
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, "*** Changing Mode ***");
        if (cmd == "CMD:MODE_AUTO") {
          display.drawString(0, 20, "AUTO NAV");
        } else {
          display.drawString(0, 20, "MANUAL NAV");
        }
        display.display();
        // REMOVED: delay(1500);
        heltec_led(100);
        radio.startReceive(); 
      }
      
      // Send Manual Movement Logic
      else {
        heltec_led(0);  
        
        radio.transmit(cmd);  
        radio.startReceive(); 
        
        readAndDisp(cmd);  
        delay(10); 
        heltec_led(100);  
      }
    }
  }
}

void readAndDisp(String cmd) {
  if (cmd.startsWith("M:")) {
    int commaIdx = cmd.indexOf(',');
    if (commaIdx != -1) {
        String xStr = cmd.substring(2, commaIdx);
        String yStr = cmd.substring(commaIdx + 1);
        float x = xStr.toFloat();
        float y = yStr.toFloat();

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
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "Sending:");
      display.drawString(0, 20, cmd);
      display.display();
  }
}

void updateDisp(String dir, float x, float y) {
  display.clear();
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, "TX: " + dir);
  display.setFont(ArialMT_Plain_10);
  String stats = "X: " + String(x, 2) + "  Y: " + String(y, 2);
  display.drawString(0, 30, stats);
  display.drawString(0, 45, "Status: Transmitting");
  display.display();
}

void fixDisp() {
  pinMode(36, OUTPUT); 
  digitalWrite(36, LOW); 
  delay(50); 
  Wire.beginTransmission(0x3c);
  Wire.write(0x80); Wire.write(0x8D); 
  Wire.write(0x80); Wire.write(0x14); 
  Wire.endTransmission();
  Wire.beginTransmission(0x3c);
  Wire.write(0x80); Wire.write(0xAF); 
  Wire.endTransmission();
}