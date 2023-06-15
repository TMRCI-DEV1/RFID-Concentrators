//ESP8266 and OLED WiFi Sketch for use with Arduino UnoWiFi v5
 
//0.96 inch I2C OLED Screen Connections to ESP Header Pins
// +3.3v to +3.3v
// GND to GND
// SDA to GPIO4
// SCL to GPIO12
 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
 
const byte rxPin = 9;
const byte txPin = 10;
const byte sdaPin = 4;
const byte sclPin = 12;
 
WiFiUDP ntpUDP;
 
NTPClient timeClient(ntpUDP, 0); // Use 0 as the offset for GMT timezone
 
 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C //Usually 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
 
 
// WiFi Credentials
const char* ssid = "xxxxxxxxx"; // Add your SSID here
const char* password = "xxxxxxxx"; // Add your password here
const int serverPort = 8888; // JMRI port
 
WiFiServer server(serverPort);
WiFiClient client;
 
String receivedData = "";
 
 
void setup() {
  Serial.begin(9600);
  Wire.begin(4, 12);  //X=SDA, Y=SCL
  pinMode(rxPin, INPUT); // Set RX pin as input
  pinMode(txPin, OUTPUT); // Set TX pin as output
 
  delay(1000);
  Serial.println('\n');
 
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
 
 
    display.clearDisplay();
 
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 24);
  display.clearDisplay();
 
  //Display Standby Text
  display.println("Standby...");
  display.display();
  delay(2000);
 
 
 
 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to: ");
  Serial.println(ssid);
 
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Connecting to: ");
  display.print(ssid);
  display.display();
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
 
  timeClient.begin();
  timeClient.setSummerTime(true); // Enable automatic DST adjustment
 
  Serial.println('\n');
  Serial.print("Connected to: ");
  Serial.println(ssid);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
 
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connected to: ");
  display.println("");
  display.println(ssid);
  display.println("");
  display.println("");
  display.print("IP: ");
  display.print(WiFi.localIP());
  display.display();
 
  server.begin();
}
 
 
void loop() {
  if (!client || !client.connected()) {
    client = server.available();
 
    if (client) {
      timeClient.update();
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Client Connected");
      display.println("");
      display.print("IP: ");
      display.print(WiFi.localIP());
      display.display();
    }
  }
 
 
  if (client && client.connected())
    if (Serial.available() > 0) {
      char receivedChar = Serial.read();
      client.write(receivedChar);
 
      if (receivedChar == '>') { // Check for '>' character as the end of a tag
        // Process the tag (display on OLED)
 
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Client Connected");
        display.println("");
        display.print("IP: ");
        display.println(WiFi.localIP());
        display.println("");
        display.println("");
        display.println("Last Tag:");
        display.print(receivedData);
 
        display.print("At time: ");
        display.print(timeClient.getFormattedTime());
        display.display();
 
        // Clear the receivedData for the next tag
        receivedData = "";
      } else {
        // Keep appending characters until a full tag is read
        receivedData += receivedChar;
      }
    }
}
