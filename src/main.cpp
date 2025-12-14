#include <Arduino.h>
#include <SPI.h>
#include <STM32duinoBLE.h>
#include <permaDefs.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

// RX, TX expected from permaDefs.h
SoftwareSerial mySerial(TX, RX);

// BLE Service
BLEService dataService("0X189A");

// Single BLE Characteristic (for data transfer)
BLECharacteristic dataCharacteristic("9e150970-35ad-400c-b46d-08ed71f07709", BLERead | BLEWrite | BLENotify, 128, false);

// LED pin
constexpr uint8_t LED_PIN = PA15;

// DTR pin (optional use only)
constexpr uint8_t DTR_PIN = PA4;

// Buffer for incoming serial data
const size_t SERIAL_BUF_SIZE = 512;
char serialBuffer[SERIAL_BUF_SIZE];
size_t bufferIndex = 0;

// Button debouncing
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 50; // 50ms debounce time

reqPing rPing;
settings set;

void blinkLed(int bw, int pause) {
  digitalToggle(LED_PIN);
  delay(bw);
  digitalToggle(LED_PIN);
  delay(pause);
}

bool isValidJson(const char* str, size_t len) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, str, len);
  return (error == DeserializationError::Ok);
}

void checkButton() {
  // Read the current state of the button
  bool reading = digitalRead(SWITCH);
  
  // If the button state has changed, reset the debounce timer
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed, consider it a stable reading
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // If the button state has changed from what we recorded
    if (reading != currentButtonState) {
      currentButtonState = reading;
      
      // If button is pressed (pulled LOW)
      if (currentButtonState == LOW) {
        // Toggle DTR pin
        digitalToggle(DTR_PIN);
        SerialUSB.print("Button pressed! DTR toggled to: ");
        SerialUSB.println(digitalRead(DTR_PIN) ? "HIGH" : "LOW");
        delay(100);
        digitalToggle(DTR_PIN);
        SerialUSB.print("Button pressed! DTR toggled to: ");
        SerialUSB.println(digitalRead(DTR_PIN) ? "HIGH" : "LOW");
      }
    }
  }
  
  lastButtonState = reading;
}

void processSerialData() {
  while (mySerial.available()) {
    char c = mySerial.read();
    
    // Add character to buffer
    if (bufferIndex < SERIAL_BUF_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
      
      // Check for newline (end of JSON message)
      if (c == '\n' || c == '\r') {
        serialBuffer[bufferIndex] = '\0'; // Null terminate
        
        // Trim trailing whitespace
        while (bufferIndex > 0 && (serialBuffer[bufferIndex - 1] == '\n' || 
               serialBuffer[bufferIndex - 1] == '\r' || 
               serialBuffer[bufferIndex - 1] == ' ')) {
          bufferIndex--;
          serialBuffer[bufferIndex] = '\0';
        }
        
        // Check if we have actual data
        if (bufferIndex > 0) {
          // Validate JSON
          if (isValidJson(serialBuffer, bufferIndex)) {
            // Print to serial monitor
            SerialUSB.println(serialBuffer);
            
            // Send over BLE
            dataCharacteristic.writeValue(serialBuffer);
          }
        }
        
        // Reset buffer
        bufferIndex = 0;
      }
    } else {
      // Buffer overflow - reset
      SerialUSB.println("Buffer overflow - resetting");
      bufferIndex = 0;
    }
  }
}

void processBLEData() {
  if (dataCharacteristic.written()) {
    SerialUSB.println("Data Characteristic written");
    
    char setIn[128];
    int valueLen = dataCharacteristic.valueLength();
    dataCharacteristic.readValue(setIn, valueLen);
    setIn[valueLen] = '\0'; // Null terminate
    
    SerialUSB.print("Received: ");
    SerialUSB.println(setIn);
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, setIn);
    
    if (error) {
      SerialUSB.print("JSON parsing failed: ");
      SerialUSB.println(error.c_str());
      return;
    }
    
    // Handle ping request
    if (doc.containsKey("ID") && doc.containsKey("Msg")) {
      uint16_t id = doc["ID"];
      int msg = doc["Msg"];
      
      SerialUSB.print(F("Received Ping - ID: "));
      SerialUSB.print(id);
      SerialUSB.print(F(", Message: "));
      SerialUSB.println(msg);
      
      rPing.tag = id;
      rPing.request = (byte)msg;
      
      // Send binary struct over SoftwareSerial
      mySerial.write((uint8_t*)&rPing, sizeof(rPing));
      SerialUSB.println("Sent reqPing via SoftwareSerial");
    }
    
    // Handle settings update
    else if (doc.containsKey("ID") && doc.containsKey("gfrq")) {
      set.gpsFrq = doc["gfrq"];
      set.gpsTout = doc["gto"];
      set.hdop = doc["hdop"];
      
      SerialUSB.println(F("Received Settings"));
      SerialUSB.print(F("GPS Freq: ")); SerialUSB.println(set.gpsFrq);
      SerialUSB.print(F("GPS Timeout: ")); SerialUSB.println(set.gpsTout);
      SerialUSB.print(F("HDOP: ")); SerialUSB.println(set.hdop);
      
      // Send binary struct over SoftwareSerial
      mySerial.write((byte*)&set, sizeof(set));
      SerialUSB.println("Sent settings via SoftwareSerial");
    }
    
    // Unknown JSON format
    else {
      SerialUSB.println("Unknown JSON format received");
    }
  }
}

void setup() {
  delay(2000);
  SerialUSB.begin(115200);
  SerialUSB.println("ArcTrack BLE-Serial Bridge (JSON Mode)");
  pinMode(BLELED, OUTPUT);
  pinMode(SWITCH, INPUT_PULLUP);
  pinMode(DTR_PIN, OUTPUT); // Changed to OUTPUT so we can toggle it
  pinMode(PWRLED, OUTPUT);
  digitalWrite(BLELED, HIGH);
  digitalWrite(PWRLED, HIGH);
  digitalWrite(DTR_PIN, HIGH); // Initialize DTR to LOW

  // Start SoftwareSerial
  mySerial.begin(9600);
  SerialUSB.println("SoftwareSerial started on RX/TX");

  // BLE setup
  SerialUSB.println("Initializing BLE...");
  if (!BLE.begin()) {
    SerialUSB.println("Starting BLE failed!");
    while (1) blinkLed(100, 100);
  }

  SerialUSB.println("BLE started!");
  SerialUSB.print("BLE Address: ");
  SerialUSB.println(BLE.address());

  BLE.setLocalName(tagID);
  BLE.setAdvertisedService(dataService);

  dataService.addCharacteristic(dataCharacteristic);

  BLE.addService(dataService);

  if (!BLE.advertise()) {
    SerialUSB.println("Failed to start advertising!");
  } else {
    SerialUSB.println("BLE advertising started!");
  }
  
  SerialUSB.println("Ready to receive JSON data...");
  SerialUSB.println("Press button on MOSI pin to toggle DTR");
}

void loop() {
  // Check button state and toggle DTR if pressed
  checkButton();
  
  // Process incoming serial data (from device -> BLE)
  processSerialData();
  
  // Process incoming BLE data (from BLE -> device)
  processBLEData();
  
  // Handle BLE events
  BLE.poll();
  
  // Small delay to prevent overwhelming the loop
  delay(10);
}