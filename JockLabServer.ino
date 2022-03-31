/*
 * Documentation for the following libraries:
 * <WiFi.h> - https://www.arduino.cc/en/Reference/WiFi
 * <AsyncTCP.h> - https://github.com/me-no-dev/AsyncTCP/blob/master/src/AsyncTCP.h
 * <ESPAsyncWebServer.h> - https://github.com/me-no-dev/AsyncTCP/blob/master/src/AsyncTCP.h
 */
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "web.h"

AsyncWebServer server(80); //port 80 -> constructor: AsyncServer(uint16_t port);

const char* ssid     = "Jocklab Server";// "ESP32-Access-Point";
const char* password = "ElijahMoore";

//input parameters to server
const char* RepetitionsInput = "Repetitions";
const char* DistanceInput = "Distance";
const char* SpeedInput = "Speed";

/*
 * -simple HTML web page -> check "ESPAsyncWebServer README.md" 
 * for more information
 * -Look at HTTP GET request
 */

void NotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200); //set baud rate and pins
  WiFi.softAP(ssid, password);
  
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  

  // set root to HTML page
  server.on("/", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", webpageCode);
  });

  //handle HTTP get requests
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println(buildTransmission(request));
    returnPage(request); 
  });
  
  server.onNotFound(NotFound);
  server.begin();
  
}

//listen for client
void loop() {}

//build transmission in Distance
String buildTransmission (AsyncWebServerRequest *request) {
  String returnString = "";
  return returnString + 
  request->getParam(DistanceInput)->value() +
  "-" +
  request->getParam(SpeedInput)->value() +
  "-" +
  request->getParam(RepetitionsInput)->value();
}

void returnPage(AsyncWebServerRequest *request) {
  request->send(200, "text/html", "Data Successfuly Sent!" "<br><a href=\"/\">Send Another Transmission</a>");
}
