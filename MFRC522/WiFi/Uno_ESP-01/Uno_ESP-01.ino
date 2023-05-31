//Multi MFRC522 Reader Uno WiFi v1

//MFRC522     Nano/Uno      Mega
//3.3v          3.3v        3.3v      
//RST             See #define
//GND           GND         GND
//IRQ             Not Used
//MISO          D12         D50
//MOSI          D11         D51
//SCK           D13         D52
//SS/SDA          See #define

//Arduino Uno (7 Readers)
//SS pins: 8, 6, 4, A0, A2, A4
//RST pins: 7, 5, 3, A1, A3, A5


// Import Libraries
#include <SPI.h>           // SPI library for communicating with the MFRC522 reader
#include <MFRC522.h>       // MFRC522 library for reading RFID cards

 
// Define the SS (Slave Select) and RST (Reset) pins for each reader
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
const uint8_t ssPins[] = {8, 6, 4, 2, A0, A2, A4};
const uint8_t rstPins[] = {7, 5, 3, 1, A1, A3, A5};
const uint8_t readerAssignment[] = {1, 2, 3, 4, 5, 6, 7}; // Assign reader numbers based on SS and RST pins
#elif defined(ARDUINO_AVR_MEGA2560)
const uint8_t ssPins[] = {8, 6, 4, 2, A0, A2, A4, A6};
const uint8_t rstPins[] = {7, 5, 3, 1, A1, A3, A5, A7};
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
 
void setup() {
  Serial.begin(115200);
  SPI.begin();
 
  for (uint8_t i = 0; i < numReaders; i++) {
    readers[i].ssPin = ssPins[i];
    readers[i].rstPin = rstPins[i];
    readers[i].id = readerID[readerAssignment[i] - 1]; // Assign the reader ID based on readerAssignment array
    readers[i].mfrc522 = MFRC522(readers[i].ssPin, readers[i].rstPin);
    readers[i].mfrc522.PCD_Init();
 
    // Check if the reader is connected
    if (readers[i].mfrc522.PCD_PerformSelfTest()) {
      readers[i].isConnected = true;
      readers[i].mfrc522.PCD_SetAntennaGain(readers[i].mfrc522.RxGain_max);
 
      // Print debugging information  
      //Comment out for operation, uncomment for debugging in USB-MCU mode only
      //Serial.print("Reader ");
      //Serial.print(readers[i].id);
      //Serial.print(" detected on SS pin ");
      //Serial.println(readers[i].ssPin);
    } else {
      readers[i].isConnected = false;
    }
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
 
void loop()
{
  for (uint8_t i = 0; i < numReaders; i++) {
    if (readers[i].mfrc522.PICC_IsNewCardPresent() && readers[i].mfrc522.PICC_ReadCardSerial()) {
      for (uint8_t j = 0; j < readers[i].mfrc522.uid.size; j++) {
        readers[i].nuid[j] = readers[i].mfrc522.uid.uidByte[j];
      }
 
      byte checksum = readers[i].nuid[0];
      for (uint8_t j = 1; j < 5; j++) {
        checksum ^= readers[i].nuid[j];
      }

      Serial.write(readers[i].id);
 
      for (uint8_t j = 0; j < 5; j++) {
        Serial.print(readers[i].nuid[j] < 0x10 ? "0" : "");
        Serial.print(readers[i].nuid[j], HEX);
      }
 
      Serial.print(checksum < 0x10 ? "0" : "");
      Serial.print(checksum, HEX);
 
      Serial.write(0x0D);
      Serial.write(0x0A);
      Serial.write('>');

      readers[i].mfrc522.PICC_HaltA();
      readers[i].mfrc522.PCD_StopCrypto1();
    }
  }
}
