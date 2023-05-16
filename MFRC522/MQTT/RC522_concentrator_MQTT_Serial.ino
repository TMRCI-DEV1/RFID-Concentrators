/*
  Project: Arduino-based MFRC522 RFID Concentrator
  Author: Thomas Seitz (thomas.seitz@tmrci.org)
  Version: 1.0.7
  Date: 2023-05-16
  Description: A sketch for an Arduino-based RFID concentrator that supports up to 8 RFID readers, sends the data to an MQTT broker,
  and outputs data to Serial clients. Uses DHCP for obtaining Arduino's IP address.
*/

// Include required libraries
#include <SPI.h>           // SPI library for communicating with the MFRC522 reader
#include <MFRC522.h>       // MFRC522 library for reading RFID cards
#include <Ethernet.h>      // Ethernet library for the Ethernet shield
#include <PubSubClient.h>  // PubSub Client library for the MQTT connection  

// Define MAC address, IP address, and MQTT server
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress mqttServer(192, 168, 1, 200);
const char* mqttSensorTopicBase = "TMRCI/dt/sensor/RFID_sensor/"; // Replace with the the appropriate JMRI Sensor topic
const char* mqttReporterTopicBase = "TMRCI/dt/reporter/RFID/";    // Replace with the appropriate JMRI Reporter topic

// Define SS and RST pins, and reader assignments based on Arduino board type
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
const uint8_t ssPins[] = {5, 6, 7, A0, A1, A2};
const uint8_t rstPins[] = {4, 8, 9, A3, A4, A5};
const uint8_t readerAssignment[] = {1, 2, 3, 4, 5, 6}; // Assign reader numbers based on SS and RST pins
#elif defined(ARDUINO_AVR_MEGA2560)
const uint8_t ssPins[] = {5, 6, 7, 8, 9, A0, A1, A2};
const uint8_t rstPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
const uint8_t readerAssignment[] = {1, 2, 3, 4, 5, 6, 7, 8}; // Assign reader numbers based on SS and RST pins
#endif

// Calculate the number of readers
const int numReaders = sizeof(ssPins) / sizeof(ssPins[0]);

// Define reader IDs
const char readerID[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};

// Define RFIDReader struct for handling RFID reader data and state
struct RFIDReader {
  char id;
  uint8_t ssPin;
  uint8_t rstPin;
  MFRC522 mfrc522;
  byte nuid[7];
  bool isConnected;
  bool tagPresent;

  // Constructor for initializing RFIDReader struct
  RFIDReader() : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)), isConnected(false), tagPresent(false) {} // Initialize tagPresent to false
};

// Create an array of RFIDReader structs
RFIDReader readers[numReaders];

// Ethernet client objects
EthernetClient ethClient; // MQTT client
EthernetClient client;    // Ethernet client
PubSubClient mqttClient(ethClient);

// Declare a variable to track the Ethernet connection status
bool isEthernetConnected = false;

// Setup function - initializes Serial, SPI, and Ethernet, sets pin modes, initializes RFID readers, and blinks the LED
void setup() {
  // Initialize serial communication and SPI
  Serial.begin(9600);
  SPI.begin();

  // Start Ethernet connection with the given MAC address
  Ethernet.begin(mac);
  delay(1000);

  // Set the MQTT server
  mqttClient.setServer(mqttServer, 1883); // Replace 1883 with your MQTT broker's port if different

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
    mqttClient.setServer(mqttServer, 1883); // Replace 1883 with your MQTT broker's port if different
  }

  // Initialize RFID readers
  for (uint8_t i = 0; i < numReaders; i++) {
    readers[i].id = readerID[i];
    readers[i].ssPin = ssPins[i];
    readers[i].rstPin = rstPins[i];
    readers[i].mfrc522 = MFRC522(readers[i].ssPin, readers[i].rstPin);
    readers[i].mfrc522.PCD_Init();
    readers[i].mfrc522.PCD_SetAntennaGain(readers[i].mfrc522.RxGain_max);

    // Check if the reader is connected
    if (readers[i].mfrc522.PCD_PerformSelfTest()) {
      readers[i].isConnected = true;

      // Print debugging information (commented out)
      // Serial.print("Reader ");
      // Serial.print(readers[i].id);
      // Serial.print(" detected on SS pin ");
      // Serial.println(readers[i].ssPin);
    } else {
      readers[i].isConnected = false;
    }
  }

  // Blink the LED to indicate the number of detected readers
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < numReaders; i++) {
    if (readers[i].isConnected) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
}

// MQTT reconnect function - attempts to reconnect to the MQTT broker
unsigned long lastAttemptTime = 0;

void reconnect() {
  // Continue trying to reconnect until connected
  while (!mqttClient.connected()) {
    // Attempt to establish an MQTT connection (commented out)
    // Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("arduinoClient")) {
      // Connection was successful (commented out)
      // Serial.println("connected");
    } else {
      // Connection failed, check if 5 seconds have passed before retrying (commented out)
      // Serial.print("failed, rc=");
      // Serial.print(mqttClient.state());
      // Serial.println(" try again in 5 seconds");
      unsigned long currentMillis = millis();
      if (currentMillis - lastAttemptTime > 5000) {
        lastAttemptTime = currentMillis;
      } else {
        // If 5 seconds haven't passed yet, return and do other tasks
        return;
      }
    }
  }
}

// Function to send output to Serial only
void sendOutputToSerial(RFIDReader &reader) {
  // Calculate the checksum from the reader's NUID
  byte checksum = reader.nuid[0];
  for (uint8_t j = 1; j < 5; j++) {
    checksum ^= reader.nuid[j];
  }

  // Send reader ID to Serial connection
  Serial.write(reader.id);

  // Send reader ID to Ethernet client, if connected
  if (isEthernetConnected && ethClient.connected()) {
    ethClient.write(reader.id);
  }

  // Send NUID and checksum to both Serial and Ethernet connections
  for (uint8_t j = 0; j < 5; j++) {
    // Send NUID to Serial connection
    Serial.print(reader.nuid[j] < 0x10 ? "0" : "");
    Serial.print(reader.nuid[j], HEX);
  }

  // Send checksum to Serial connection
  Serial.print(checksum < 0x10 ? "0" : "");
  Serial.print(checksum, HEX);

  // Output line endings and a '>' symbol (MERG Concentrator format)
  Serial.write(0x0D); // CR
  Serial.write(0x0A); // LF
  Serial.write('>');  // ETX replaced by '>'
}

// Function to get the RFID data (NUID) as a string
String getRFIDData(RFIDReader &reader) {
  String rfidData = ""; // Declare the rfidData here
  
  // Add NUID to the RFID data string
  for (uint8_t j = 0; j < 5; j++) { // Loop for 5-byte UID
    if (reader.nuid[j] < 0x10) {
      rfidData += "0";
    }
    rfidData += String(reader.nuid[j], HEX);
  }
  
  // Return the formatted RFID data string
  return rfidData;
}

void loop() {
  // Maintain Ethernet connection
  Ethernet.maintain();

  // Reconnect to MQTT broker if not connected
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop(); // Keep MQTT client connected and process incoming messages

  // Check for new cards and read card UIDs
  for (uint8_t i = 0; i < numReaders; i++) {
    if (readers[i].mfrc522.PICC_IsNewCardPresent() && readers[i].mfrc522.PICC_ReadCardSerial()) {
      readers[i].tagPresent = true;
  
      // Update the 'nuid' field with the card's UID
      for (uint8_t j = 0; j < readers[i].mfrc522.uid.size; j++) {
        readers[i].nuid[j] = readers[i].mfrc522.uid.uidByte[j];
      }

      // Get RFID data and construct MQTT topics
      String rfidData = getRFIDData(readers[i]);
      String topicSensor = String(mqttSensorTopicBase) + String(readers[i].id) + "/";
      String topicReporter = String(mqttReporterTopicBase) + String(readers[i].id) + "/";

      // Always send output to Serial and Ethernet connections
      sendOutputToSerial(readers[i]);

      // Send data via MQTT if Ethernet is connected
      if (isEthernetConnected) {
        mqttClient.connect("RFID_reader");
        mqttClient.publish(topicSensor.c_str(), "ACTIVE");
        mqttClient.publish(topicReporter.c_str(), rfidData.c_str());
        mqttClient.disconnect();
      }

      // Halt card processing and stop encryption
      readers[i].mfrc522.PICC_HaltA();
      readers[i].mfrc522.PCD_StopCrypto1();
    } else {
      readers[i].tagPresent = false;

      // If a tag is no longer detected, set the sensor to INACTIVE and clear the reporter
      String topicSensor = String(mqttSensorTopicBase) + String(readers[i].id) + "/";
      String topicReporter = String(mqttReporterTopicBase) + String(readers[i].id) + "/";
      if (isEthernetConnected) {
        mqttClient.connect("RFID_reader");
        mqttClient.publish(topicSensor.c_str(), "INACTIVE");
        mqttClient.publish(topicReporter.c_str(), ""); // Clear the reporter by sending an empty payload
        mqttClient.disconnect();
      }
    }
  }
}
