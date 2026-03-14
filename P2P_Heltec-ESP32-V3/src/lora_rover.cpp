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
* Last updated: 3/1/2026
* Author: JoseAngel Socorro Pulido
*/
#include "heltec_unofficial.h"

#define FREQ 915.0  // Frequency band (MHz) (US: 915)
#define BW   500.0  // Bandwidth (kHz)
#define SF   7      // Spreading Factor (lower -> faster)
#define CR   5      // Coding Rate
#define STICK_THRESH 0.3  // threshold for detecting movement

// Battery Monitor Settings ====================
const int BAT_PIN = 2;  // GPIO 2 (ADC1_CH1)
const float R1 = 10000.0;  // 10k Ohm
const float R2 = 2000.0;  // 2k Ohm
const float V_REF = 3.3; 

// Low Battery Safety Settings =================
#define LOW_BAT_THRESH 9  
unsigned long lowBatTimer = 0;
bool shutdownTriggered = false;
const int LOW_BAT_TIMEOUT = 5000; // 5 seconds of sustained low voltage

// Function Prototypes ===========================
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
  
  float currentVolts = getVoltage();
  checkBatteryHealth(currentVolts);

  if (shutdownTriggered) {
    heltec_led(50); 
    delay(1000);
    return; 
  }

  // ===================================================================
  // Check if the Pi wants to transmit a CSV Log to the laptop
  // ===================================================================
  if (Serial.available() > 0) {
    String outMsg = Serial.readStringUntil('\n');
    outMsg.trim();
    
    if (outMsg.startsWith("LOG:")) {
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 25, "Transmitting Log...");
      display.display();
      
      radio.transmit(outMsg);
      radio.startReceive();
      
      updateDisp("AUTO", 0, 0, currentVolts); 
    }
  }

  // ===================================================================
  // Check for incoming LoRa packet (Joystick Commands from Laptop)
  // ===================================================================
  
  int state = radio.receive(cmd);

  if (state == RADIOLIB_ERR_NONE) {
    cmd.trim();
    Serial.println(cmd);

    // Read Shutdown 
    if (cmd == "CMD:SHUTDOWN") {
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 25, "Shutting Down...");
      display.display();
      // REMOVED: delay(3000); 
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

      // REMOVED: delay(2000);  
    }
    // Read Movement 
    else if (cmd.startsWith("M:")) {
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
            dir = (x < 0) ? "LEFT" : "RIGHT";
          }
          else {
            dir = (y < 0) ? "UP" : "DOWN";
          }
        }
        updateDisp(dir, x, y, currentVolts);
      }
    }
    else {
        display.clear();
        display.drawString(0, 0, "Raw Msg:");
        display.drawString(0, 15, cmd);
        display.display();
    }
    
    delay(50);  
    heltec_led(100);  
  }
}

float getVoltage() {
  long sum = 0;
  for(int i=0; i<10; i++){
    sum += analogRead(BAT_PIN);
    delay(1);
  }
  float avg_adc = sum / 10.0;
  float pin_voltage = avg_adc * (V_REF / 4095.0);
  float bat_voltage = pin_voltage * ((R1 + R2) / R2);

  return bat_voltage;
}

void checkBatteryHealth(float voltage) {
  if (voltage < LOW_BAT_THRESH) {
    if (lowBatTimer == 0) {
      lowBatTimer = millis();
    }
    
    if (millis() - lowBatTimer > LOW_BAT_TIMEOUT && !shutdownTriggered) {
      shutdownTriggered = true;

      Serial.println("CMD:SHUTDOWN");

      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "LOW BATTERY");
      display.drawString(0, 20, "Voltage: " + String(voltage, 2) + "V");
      display.setFont(ArialMT_Plain_10);
      display.drawString(0, 40, "Shutting Down Pi...");
      display.display();
      
      for(int i=0; i<10; i++) {
        heltec_led(0); delay(100);
        heltec_led(100); delay(100);
      }
    }
  } else {
    lowBatTimer = 0;
  }
}

void updateDisp(String dir, float x, float y, float voltage) {
  display.clear();
  
  display.setFont(ArialMT_Plain_24);
  display.drawString(0, 0, dir);
  
  display.setFont(ArialMT_Plain_10);
  String stats = "X: " + String(x, 2) + "  Y: " + String(y, 2);
  display.drawString(0, 30, stats);
  
  String rssi = "RSSI: " + String(radio.getRSSI());
  display.drawString(0, 42, rssi);

  int pct = map(voltage * 100, 1110, 1260, 0, 100);
  pct = constrain(pct, 0, 100);

  String batStr = "Battery: " + String(voltage, 1) + "V (" + String(pct) + "%)";
  display.drawString(0, 53, batStr);
  
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