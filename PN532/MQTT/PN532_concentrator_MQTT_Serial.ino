/*
  Project: Arduino-based PN532 RFID Concentrator
  Author: Thomas Seitz (thomas.seitz@tmrci.org)
  Version: 1.0.8
  Date: 2023-05-17
  Description: A sketch for an Arduino-based RFID concentrator that supports up to 8 RFID readers, sends the data to an MQTT broker,
  and outputs data to Serial clients. Uses DHCP for obtaining Arduino's IP address.
*/

// Include Libraries
#include <SPI.h>             // SPI communication library
#include <Adafruit_PN532.h>  // PN532 NFC/RFID library
#include <string.h>          // String manipulation library
#include <Ethernet.h>        // Ethernet communication library
#include <PubSubClient.h>    // MQTT library

// #define USE_MQTT  // Uncomment this line to use MQTT. Comment it to use Serial only.

// MQTT base topics - Change to match your JMRI MQTT topics for Reporters and Sensors
const String REPORTER_BASE_TOPIC = "TMRCI/dt/reporter/RFID/";
const String SENSOR_BASE_TOPIC = "TMRCI/dt/sensor/RFID_sensor/";

// Ethernet configuration
byte mac[] = { 0x90, 0xA2, 0xDA, 0x85, 0xE1, 0x7E };  // MAC address for the Ethernet shield

// MQTT configuration
IPAddress mqttServer(192, 168, 1, 11);   // IP address of the MQTT broker
const int mqttPort = 1883;               // Port number for the MQTT broker

EthernetClient ethClient;            // Create an Ethernet client
PubSubClient mqttClient(ethClient);  // Create an MQTT client using the Ethernet client

// RFIDReader struct for managing RFID readers
struct RFIDReader {
  char id;
  uint8_t ssPin;
  Adafruit_PN532 pn532;
  byte nuid[7];
  bool cardRead;
  unsigned long lastReadTime;

  RFIDReader()
    : id(0), ssPin(0), pn532(Adafruit_PN532(0, &SPI)), cardRead(false) {}
};

// Configure RFID reader pins and IDs for Arduino Uno or Nano
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
const uint8_t ssPins[] = { 9, 8, 7, 6, 5, A0 };
const char readerID[] = { 'A', 'B', 'C', 'D', 'E', 'F' };
// Configure RFID reader pins and IDs for Arduino Mega 2560
#elif defined(ARDUINO_AVR_MEGA2560)
const uint8_t ssPins[] = { 45, 44, 43, 42, 41, 40, 39, 38 };
const char readerID[] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H' };
#endif

// Calculate the number of RFID readers
const int numReaders = sizeof(ssPins) / sizeof(ssPins[0]);
// Create an array of RFIDReader instances
RFIDReader readers[numReaders];

// Flag to check if the Ethernet connection is established
bool isEthernetConnected = false;

void setup() {
  // Initialize serial communication and SPI
  Serial.begin(9600);
  SPI.begin();

  // Configure pin 10 for the Ethernet shield
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // Set pin 53 as OUTPUT for the Arduino Mega 2560
  #if defined(ARDUINO_AVR_MEGA2560)
    pinMode(53, OUTPUT);
  #endif

  // Configure pin 4 for the SD card on the Ethernet shield
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  #ifdef USE_MQTT
    // Start Ethernet connection with the given MAC address
    if (Ethernet.begin(mac) == 1) {
      isEthernetConnected = true;
      delay(1000);
    } else {
      // Ethernet shield not connected or network not available
      isEthernetConnected = false;
    }

    // If Ethernet is connected, start the server and set the MQTT server
    if (isEthernetConnected) {
      mqttClient.setServer(mqttServer, mqttPort);
    }
  #endif

  // Initialize RFID readers
  for (uint8_t i = 0; i < numReaders; i++) {
    readers[i].ssPin = ssPins[i];
    readers[i].id = readerID[i];
    readers[i].cardRead = false;
    readers[i].pn532 = Adafruit_PN532(ssPins[i], &SPI);
    readers[i].pn532.begin();

    // Check if the RFID reader is detected and configure it
    if (readers[i].pn532.getFirmwareVersion()) {
      readers[i].pn532.SAMConfig();
      // Print information about the detected reader
    } else {
      readers[i].ssPin = 0;  // Mark as not detected
    }
  }

  // Set built-in LED pin as OUTPUT
  pinMode(LED_BUILTIN, OUTPUT);

  // Count the number of detected readers
  uint8_t detectedReaders = 0;
  for (uint8_t i = 0; i < numReaders; i++) {
    if (readers[i].ssPin != 0) {
      detectedReaders++;
    }
  }

  // Blink the on-board LED based on the number of detected readers
  for (uint8_t i = 0; i < detectedReaders; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    if (i < detectedReaders - 1) {
      delay(200);
    }
  }
}

// Variables for tracking MQTT reconnect attempts
unsigned long lastReconnectAttempt = 0;
unsigned long reconnectInterval = 5000;

// Function to attempt MQTT reconnection
void mqtt_reconnect() {
  // Get the current time
  unsigned long now = millis();

  // Check if the reconnect interval has passed since the last attempt
  if (now - lastReconnectAttempt > reconnectInterval) {
    // Update the timestamp of the last reconnect attempt
    lastReconnectAttempt = now;

    // Try to connect to the MQTT broker with the client ID "Arduino"
    if (mqttClient.connect("Arduino")) {
      // Connection successful (uncomment the line below for debugging)
      // Serial.println("MQTT connected");
    } else {
      // Connection failed (uncomment the lines below for debugging)
      // Serial.print("MQTT connection failed, rc=");
      // Serial.print(mqttClient.state());
      // Serial.println(" try again in 5 seconds");
    }
  }
}

// Main loop function
void loop() {
  #ifdef USE_MQTT
    // Check if Ethernet is connected
    if (isEthernetConnected) {
      mqttClient.loop();
    }
  #endif

  // Loop through all RFID readers
  for (uint8_t i = 0; i < numReaders; i++) {
    // Skip this iteration if the reader is not detected
    if (readers[i].ssPin == 0) {
      continue;
    }
    
    // Check if enough time has passed since the last read
    if (millis() - readers[i].lastReadTime < 500) {
      continue;
    }

    uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };
    uint8_t uidLength;
    // Read the card UID from the RFID reader
    uint8_t success = readers[i].pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

    // If a card is successfully read
    if (success) {
      // Copy the UID to the reader's nuid array
      memcpy(readers[i].nuid, uid, 7);
      readers[i].cardRead = true;
      
      // Update the last read time
      readers[i].lastReadTime = millis();

      // Calculate the UID checksum
      byte checksum = readers[i].nuid[0];
      for (byte j = 1; j < 5; j++) {
        checksum ^= readers[i].nuid[j];
      }

      // Output the reader ID and UID data to Serial and Ethernet client
      // (the following sections handle both Serial and Ethernet client output)

      // Output the reader ID
      Serial.print(readers[i].id);

      // Output the UID
      for (byte j = 0; j < 5; j++) {
        Serial.print(readers[i].nuid[j] < 0x10 ? "0" : "");
        Serial.print(readers[i].nuid[j], HEX);
      }

      // Output the UID checksum
      Serial.print(checksum < 0x10 ? "0" : "");
      Serial.print(checksum, HEX);

      // Output line endings and a '>' symbol (MERG Concentrator format)
      Serial.write(0x0D);  // CR
      Serial.write(0x0A);  // LF
      Serial.write('>');   // ETX replaced by '>'

      #ifdef USE_MQTT
        // If Ethernet is connected
        if (isEthernetConnected) {
          // Reconnect to the MQTT broker if necessary
          if (!mqttClient.connected()) {
            mqtt_reconnect();
          }
        
          // Publish the UID data to the MQTT reporter topic
          String reporterTopic = REPORTER_BASE_TOPIC + String(readers[i].id) + "/";
          String tagData = "";
          for (byte j = 0; j < 5; j++) {  // Only include the 5-byte tag data, exclude the checksum
            tagData += String(readers[i].nuid[j] < 0x10 ? "0" : "");
            tagData += String(readers[i].nuid[j], HEX);
          }
          mqttClient.publish(reporterTopic.c_str(), tagData.c_str());

          // Publish an "ACTIVE" status to the MQTT sensor topic
          String sensorTopic = SENSOR_BASE_TOPIC + String(readers[i].id) + "/";
          mqttClient.publish(sensorTopic.c_str(), "ACTIVE");
        }
           #endif
    } else {
      // If no card is read and the reader was previously in "card read" state
      if (readers[i].cardRead) {
        // Output the reader ID and an "empty" UID to Serial and Ethernet client
        // (the following sections handle both Serial and Ethernet client output)

        // Output the reader ID
        Serial.print(readers[i].id);

        // Output an "empty" UID
        for (byte j = 0; j < 5; j++) {
          Serial.print("00");
        }

        // Output an "empty" checksum
        Serial.print("00");

        // Output line endings and a '>' symbol (MERG Concentrator format)
        Serial.write(0x0D);  // CR
        Serial.write(0x0A);  // LF
        Serial.write('>');   // ETX replaced by '>'

        #ifdef USE_MQTT
          // If Ethernet is connected
          if (isEthernetConnected) {
            // Reconnect to the MQTT broker if necessary
            if (!mqttClient.connected()) {
              mqtt_reconnect();
            }
            
            // Publish an "empty" UID to the MQTT reporter topic
            String reporterTopic = REPORTER_BASE_TOPIC + String(readers[i].id) + "/";
            mqttClient.publish(reporterTopic.c_str(), "");

            // Publish an "INACTIVE" status to the MQTT sensor topic
            String sensorTopic = SENSOR_BASE_TOPIC + String(readers[i].id) + "/";
            mqttClient.publish(sensorTopic.c_str(), "INACTIVE");
          }
        #endif

        // Reset the reader's "card read" state and UID
        readers[i].cardRead = false;
        memset(readers[i].nuid, 0, sizeof(readers[i].nuid));
      }
    }
  }
}
