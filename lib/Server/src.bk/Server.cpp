//
// A simple server implementation showing how to:
//  * serve static messages
//  * read GET and POST parameters
//  * handle missing pages / 404s
//

#include "Server.h"
#include <Arduino.h>
#include <WiFi.h>
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <ArduinoJson.h>
#include <Render.h>
#include <FrequencySensor.h>

WebServer::WebServer(int port, const char* ssid, const char* password, RenderMode3_t *render,
    Audio_Processor_t *audio) {

    this->render = render;
    this->audio = audio;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.printf("WiFi Failed!\n");
        return;
    }

    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    server = new AsyncWebServer(port);

    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "LED Panel v.1 server ready");
    });

    server->on("/", HTTP_POST, postHandler);

    server->onNotFound([] (AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });

    server->begin();
}

void WebServer::postHandler(AsyncWebServerRequest *request) {

}
