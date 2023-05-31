// Multi PN532 Reader Uno/Mega WiFi v2
// For use with Uno WiFi and Mega Wifi Clones

//PN532         Uno       Mega
//SCK           D13       D52
//MISO          D12       D50
//MOSI          D11       D51
//SS              See #defined
//VCC           5v         5v
//GND           GND        GND
//IRQ             Not Used
//RSTO            Not Used

//Arduino Uno WiFi (7 Readers)
//SS Pins: 10, 8, 6, 4, 2, A1, A3

//Arduino Mega WiFi (8 Readers)
//SS Pins: 10, 8, 6, 4, 2, A1, A3, A5


#include <SPI.h>
#include <Adafruit_PN532.h>
#include <string.h>
 
struct RFIDReader {
  char id;
  uint8_t ssPin;
  Adafruit_PN532 pn532;
  byte nuid[7];
  bool cardRead;
 
  RFIDReader() : id(0), ssPin(0), pn532(Adafruit_PN532(0, &SPI)), cardRead(false) {}
};
 
const uint8_t possibleSSPins[] = {10, 8, 6, 4, 2, A1, A3, A5};
const int numPossibleReaders = sizeof(possibleSSPins) / sizeof(possibleSSPins[0]);
const char readerID[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
 
RFIDReader readers[numPossibleReaders];
 
void setup() {
  Serial.begin(115200);
  SPI.begin();
 
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
    readers[i].ssPin = possibleSSPins[i];
    readers[i].id = readerID[i];
    readers[i].cardRead = false;
    readers[i].pn532 = Adafruit_PN532(possibleSSPins[i], &SPI);
    readers[i].pn532.begin();
 
    if (readers[i].pn532.getFirmwareVersion()) {
      readers[i].pn532.SAMConfig();

      // Print information about the detected reader
      // Comment out for operation, uncomment for debugging
      //Serial.print("Reader ");
      //Serial.print(readers[i].id);
      //Serial.print(" detected on SS pin: ");
      //Serial.println(readers[i].ssPin);
    } else {
      readers[i].ssPin = 0; // Mark as not detected
    }
  }
}
 
void loop() {
  for (uint8_t i = 0; i < numPossibleReaders; i++) {
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
 
      delay(500); // Add a delay to avoid multiple reads
    } else {
      readers[i].cardRead = false;
    }
  }
}
