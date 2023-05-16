/*
  Project: Arduino-based MFRC522 RFID Concentrator
  Author: Thomas Seitz (thomas.seitz@tmrci.org)
  Version: 1.0.5
  Date: 2023-05-16
  Description: A sketch for an Arduino-based RFID concentrator that supports up to 8 RFID readers 
  and outputs data to Ethernet and Serial clients.
*/

// Include required libraries
#include <SPI.h>           // SPI library for communicating with the MFRC522 reader
#include <MFRC522.h>       // MFRC522 library for reading RFID cards
#include <Ethernet.h>      // Ethernet library for the Ethernet shield

// Ethernet configuration
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);  // Replace with the desired IP address
unsigned int serverPort = 8888;  // Replace with the desired port number
EthernetServer server(serverPort);

// Define the SS (Slave Select) and RST (Reset) pins for each reader
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
const uint8_t ssPins[] = {5, 6, 7, A0, A1, A2};
const uint8_t rstPins[] = {4, 8, 9, A3, A4, A5};
const uint8_t readerAssignment[] = {1, 2, 3, 4, 5, 6}; // Assign reader numbers based on SS and RST pins
#elif defined(ARDUINO_AVR_MEGA2560)
const uint8_t ssPins[] = {5, 6, 7, 8, 9, A0, A1, A2};
const uint8_t rstPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
const uint8_t readerAssignment[] = {1, 2, 3, 4, 5, 6, 7, 8}; // Assign reader numbers based on SS and RST pins
#endif

const int numReaders = sizeof(ssPins) / sizeof(ssPins[0]);
const char readerID[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};

struct RFIDReader {
  char id;
  uint8_t ssPin;
  uint8_t rstPin;
  MFRC522 mfrc522;
  byte nuid[7];
  bool isConnected;

  RFIDReader() : id(0), ssPin(0), rstPin(0), mfrc522(MFRC522(0, 0)), isConnected(false) {}
};

RFIDReader readers[numReaders];
EthernetClient client;
bool isEthernetConnected = false;

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

  Ethernet.begin(mac, ip);

  if (Ethernet.hardwareStatus() == EthernetNoHardware || Ethernet.linkStatus() == LinkOFF) {
    // Serial.println("Ethernet shield not connected or network not available");
    isEthernetConnected = false;
  } else {
    isEthernetConnected = true;
  }

  for (uint8_t i = 0; i < numReaders; i++) {
    readers[i].ssPin = ssPins[i];
    readers[i].rstPin = rstPins[i];
    readers[i].id = readerID[readerAssignment[i] - 1];
    readers[i].mfrc522 = MFRC522(readers[i].ssPin, readers[i].rstPin);
    readers[i].mfrc522.PCD_Init();
    readers[i].mfrc522.PCD_SetAntennaGain(readers[i].mfrc522.RxGain_max);

    // Check if the reader is connected
    if (readers[i].mfrc522.PCD_PerformSelfTest()) {
      readers[i].isConnected = true;

      // Print debugging information
      // Serial.print("Reader ");
      // Serial.print(readers[i].id);
      // Serial.print(" detected on SS pin ");
      // Serial.println(readers[i].ssPin);
    } else {
      readers[i].isConnected = false;
    }
  }

  if (isEthernetConnected) {
    server.begin();
  }

  // Blink the LED to indicate the number of detected readers
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < numReaders; i++) {
    if (readers[i].isConnected) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(300);
      digitalWrite(LED_BUILTIN, LOW);
      delay(300);
    }
  }
}

void loop() {
  if (isEthernetConnected) {
    if (!client.connected()) {
      client.stop();
      client = server.accept();
      /*if (client) {
        Serial.println("Client connected");
      }*/
    }
  }

  for (uint8_t i = 0; i < numReaders; i++) {
    if (readers[i].isConnected && readers[i].mfrc522.PICC_IsNewCardPresent() && readers[i].mfrc522.PICC_ReadCardSerial()) {
      for (uint8_t j = 0; j < readers[i].mfrc522.uid.size; j++) {
        readers[i].nuid[j] = readers[i].mfrc522.uid.uidByte[j];
      }

      byte checksum = readers[i].nuid[0];
      for (uint8_t j = 1; j < 5; j++) {
        checksum ^= readers[i].nuid[j];
      }

      // Send output to Serial connection
      Serial.write(readers[i].id);

      // Send output to Ethernet client, if connected
      if (isEthernetConnected && client.connected()) {
        client.write(readers[i].id);
      }

      for (uint8_t j = 0; j < 5; j++) {
        // Send output to Serial connection
        Serial.print(readers[i].nuid[j] < 0x10 ? "0" : "");
        Serial.print(readers[i].nuid[j], HEX);

        // Send output to Ethernet client, if connected
        if (isEthernetConnected && client.connected()) {
          client.print(readers[i].nuid[j] < 0x10 ? "0" : "");
          client.print(readers[i].nuid[j], HEX);
        }
      }

      // Send output to Serial connection
      Serial.print(checksum < 0x10 ? "0" : "");
      Serial.print(checksum, HEX);

      // Send output to Ethernet client, if connected
      if (isEthernetConnected && client.connected()) {
        client.print(checksum < 0x10 ? "0" : "");
        client.print(checksum, HEX);
      }

      // Send output to Serial connection
      Serial.write(0x0D);
      Serial.write(0x0A);
      Serial.write('>');

      // Send output to Ethernet client, if connected
      if (isEthernetConnected && client.connected()) {
        client.write(0x0D); // CR
        client.write(0x0A); // LF
        client.write('>');  // ETX replaced by '>'
      }

      readers[i].mfrc522.PICC_HaltA();
      readers[i].mfrc522.PCD_StopCrypto1();
    }
  }
}