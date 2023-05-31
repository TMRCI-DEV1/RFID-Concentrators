//WiFi Sketch for use with Arduino Uno/Mega WiFi
 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
 
// WiFi Credentials
const char* ssid = "xxxxxx"; //Enter your SSID here
const char* password = "xxxxx"; //Enter your password here
const int serverPort = 8888; // JMRI port
 
WiFiServer server(serverPort);
WiFiClient client;
 
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println('\n');
 
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
 
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
 
  server.begin();
}
 
void loop() {
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("");
    }
  }
 
  if (client && client.connected())
    if (Serial.available()) {
      while (Serial.available()) {
        char data = Serial.read();
        client.write(data);
      }
    }
  }
