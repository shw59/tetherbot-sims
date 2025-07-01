/* 
  pico2w_wifi_test.ino

  Raspberry Pi Pico 2 W WiFi Station Demo
  Use WiFi library to connect Pico 2 W to wifi in station mode
*/

// include the WiFi Library
#include <WiFi.h>

// replace with your network credentials
const char* ssid = "REPLACE_WITH_SSID";
const char* password = "REPLACE_WITH_PASSWORD";

// initialize WiFiClient object for the Pico to send messages over wifi
WiFiClient client;

void setup() {

  // start the serial monitor
  Serial.begin(115200);

  // operate in wifi station mode
  WiFi.mode(WIFI_STA);

  // start wifi with supplied parameters
  WiFi.begin(ssid, password);

  // print periods on monitor while establishing connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    delay(500);
  }

  // connection established
  Serial.println("");
  Serial.print("Pico 2 W is connected to WiFi network ");
  Serial.println(WiFi.SSID());

  // print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());

}

void loop() {

  delay(2000);

  // print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());

}
