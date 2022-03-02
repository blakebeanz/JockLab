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
#include <SPIFFS.h>

#define RXD2 16
#define TXD2 17

AsyncWebServer server(80); //port 80 -> constructor: AsyncServer(uint16_t port);

//create access point
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
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    Distance: <input type="text" name="Distance">
    <input type="submit" value="Update">
  </form><br>
  <form action="/get">
    Speed: <input type="text" name="Speed">
    <input type="submit" value="Update">
  </form><br>
  <form action="/get">
    Repetitions: <input type="text" name="Repetitions">
    <input type="submit" value="Update">
  </form>
</body></html>)rawliteral";

/*
 * Server not found function. Learn about request variables in the library
 * at: https://github.com/me-no-dev/ESPAsyncWebServer#request-variables
 */
void NotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200); //set baud rate and pins
  /*WiFi.mode(WIFI_STA); //may be different for soft access point
  WiFi.begin(ssid, password);*/
  WiFi.softAP(ssid, password);
  /*if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }*/
  /*while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wifi connecting...");
    delay(500);
  }*/
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  //add space and print IP Address
  /*Serial.println();
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP()); //return IPAddress struct*/

  // sets path of index to index/html
  server.on("/", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  //code for handling HTTP Get calls
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String InputTransmission;
    String InputParam;
    // GET repetitions from server <ESP_IP>/get?Repetitions=<inputMessage>
    if (request->hasParam(RepetitionsInput)) {
      InputTransmission = request->getParam(RepetitionsInput)->value();
      InputParam = RepetitionsInput;
    }
    // GET distance value on <ESP_IP>/get?Distance=<inputMessage>
    else if (request->hasParam(DistanceInput)) {
      InputTransmission = request->getParam(DistanceInput)->value();
      InputParam = DistanceInput;
    }
    // GET speec value on <ESP_IP>/get?Speed=<inputMessage>
    else if (request->hasParam(SpeedInput)) {
      InputTransmission = request->getParam(SpeedInput)->value();
      InputParam = SpeedInput;
    }
    else {
      InputTransmission = "NULL";
      InputParam = "NULL";
    }
    Serial.println(InputTransmission);
    //Serial.write(1);
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" 
                                     + InputParam + ") with value: " + InputTransmission +
                                     "<br><a href=\"/\">Return to Home Page</a>");
  });

  server.onNotFound(NotFound);
  server.begin();
  
}

void loop() {
  
}
