/*
  Project: Arduino-based PN532 RFID Concentrator
  Author: Thomas Seitz (thomas.seitz@tmrci.org)
  Version: 1.0.6
  Date: 2023-05-17
  Description: A sketch for an Arduino-based RFID concentrator that supports up to 8 RFID readers 
  and outputs data to Ethernet and Serial clients.
*/

// Uncomment the following line to enable Ethernet communication
// #define USE_ETHERNET

// Include required libraries
#include <SPI.h>            // SPI library for communicating with the PN532 reader
#include <Adafruit_PN532.h> // PN532 library for reading RFID cards
#include <string.h>         // C headers library
#ifdef USE_ETHERNET
  #include <Ethernet.h>     // Ethernet library for the Ethernet shield
#endif

// Ethernet configuration
#ifdef USE_ETHERNET
  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress ip(192, 168, 1, 177);
  unsigned int serverPort = 8888; // Replace with the desired port number
  EthernetServer server(serverPort); // Start the Ethernet server
#endif

struct RFIDReader {
  char id;
  uint8_t ssPin;
  Adafruit_PN532 pn532;
  byte nuid[7];
  bool cardRead;

  RFIDReader() : id(0), ssPin(0), pn532(Adafruit_PN532(0, &SPI)), cardRead(false) {}
};

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
const uint8_t ssPins[] = {9, 8, 7, 6, 5, A0};
const char readerID[] = {'A', 'B', 'C', 'D', 'E', 'F'};
#elif defined(ARDUINO_AVR_MEGA2560)
const uint8_t ssPins[] = {45, 44, 43, 42, 41, 40, 39, 38};
const char readerID[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
#endif

const int numReaders = sizeof(ssPins) / sizeof(ssPins[0]);
RFIDReader readers[numReaders];

bool isEthernetConnected = false;

#ifdef USE_ETHERNET
  EthernetClient client;  // Move this line here
#endif

void setup() {
  Serial.begin(9600);
  SPI.begin();

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  // Set pin 53 as OUTPUT for the Arduino Mega 2560
  #if defined(ARDUINO_AVR_MEGA2560)
    pinMode(53, OUTPUT);
  #endif

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  #ifdef USE_ETHERNET
    if (Ethernet.begin(mac) == 1) {
      isEthernetConnected = true;
      delay(1000);
    } else {
      isEthernetConnected = false;
    }

    if (isEthernetConnected) {
      server.begin();
    }
  #endif

  for (uint8_t i = 0; i < numReaders; i++) {
    readers[i].ssPin = ssPins[i];
    readers[i].id = readerID[i];
    readers[i].cardRead = false;
    readers[i].pn532 = Adafruit_PN532(ssPins[i], &SPI);
    readers[i].pn532.begin();

    if (readers[i].pn532.getFirmwareVersion()) {
      readers[i].pn532.SAMConfig();
      // Print information about the detected reader
      // Serial.print("Reader ");
      // Serial.print(readers[i].id);
      // Serial.print(" detected on SS pin: ");
      // Serial.println(readers[i].ssPin);
    } else {
      readers[i].ssPin = 0; // Mark as not detected
    }
  }

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

void loop() {
  #ifdef USE_ETHERNET
    if (isEthernetConnected) {
      client = server.available();
    }
  #endif

  for (uint8_t i = 0; i < numReaders; i++) {
    if (readers[i].ssPin == 0) {
      continue;
    }

    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t uidLength;
    uint8_t success = readers[i].pn532.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

    if (success) {
      memcpy(readers[i].nuid, uid, 7);
      readers[i].cardRead = true;

      byte checksum = readers[i].nuid[0];
      for (byte j = 1; j < 5; j++) {
        checksum ^= readers[i].nuid[j];
      }

      // Send output to Serial connection
      Serial.write(readers[i].id);
      for (byte j = 0; j < 5; j++) {
        Serial.print(readers[i].nuid[j] < 0x10 ? "0" : "");
        Serial.print(readers[i].nuid[j], HEX);
      }
      Serial.print(checksum < 0x10 ? "0" : "");
      Serial.print(checksum, HEX);
      Serial.write(0x0D);
      Serial.write(0x0A);
      Serial.write('>');

      #ifdef USE_ETHERNET
        // Send output to Ethernet client if connected
        if (client) {
          client.write(readers[i].id);
          for (byte j = 0; j < 5; j++) {
            client.print(readers[i].nuid[j] < 0x10 ? "0" : "");
            client.print(readers[i].nuid[j], HEX);
          }
          client.print(checksum < 0x10 ? "0" : "");
          client.print(checksum, HEX);
          client.write(0x0D); // CR
          client.write(0x0A); // LF
          client.write('>');  // ETX replaced by '>'
        }
      #endif

      delay(500); // Add a delay to avoid multiple reads (Comment out or modify if required)
    } else {
      readers[i].cardRead = false;
    }
  }
}
