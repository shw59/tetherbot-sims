/* 
  pico2w_wifi_test.ino

  Raspberry Pi Pico 2 W WiFi Station Demo
  Use WiFi library to connect Pico 2 W to wifi in station mode
*/

// include the WiFi Library
#include <WiFi.h>
#include "secrets.h" // include secrets.h for wifi credentials

// replace with your network credentials
const char* ssid = LAB_SSID;
const char* password = LAB_PW;

// replace with IP address and port of your server to receive messages from the Pico 2 W
const char* host = LAPTOP_IP_ADDRESS_LAB;
const int port = 5001;

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

  if (client.connect(host, port)) {
    Serial.println("Connected to server");
    client.println("Hello from Pico 2 W!");
  }
}

void loop() {
  delay(2000);

  while (!client.connected()) {
    client.connect(host, port);
    Serial.println("Lost connection, connecting to server...");
  }

  // print IP Address
  Serial.print("Sending data to ");
  Serial.println(LAPTOP_IP_ADDRESS_LAB);
  client.println("Pico IP Address:" + WiFi.localIP().toString());
}
