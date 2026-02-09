/**
   PostHTTPClient.ino

    Created on: 21.11.2016

*/

#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

#ifndef STASSID
#define STASSID "GBVOnlineServices"
#define STAPSK "GBViscool"
#endif

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    Serial.print("From Web: ");
    Serial.println((char*)payload);
  }
}

void setup() {

  Serial.begin(115200);

  WiFi.begin(STASSID, STAPSK);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output

  // WebSocket setup
  webSocket.begin("10.214.96.136", 5000, "/ws");
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();

  // Send Serial input to web
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    Serial.print("Sending: ");
    Serial.println(msg);
    webSocket.sendTXT(msg);
  }
}
