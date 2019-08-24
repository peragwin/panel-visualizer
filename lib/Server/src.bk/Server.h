#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Render.h>
#include <FrequencySensor.h>

class WebServer {
    private:
        RenderMode3_t *render;
        Audio_Processor_t *audio;
        AsyncWebServer *server;

        static void postHandler(AsyncWebServerRequest *request);

    public:
        WebServer(int port, const char* ssid, const char* password,
            RenderMode3_t *render, Audio_Processor_t *audio);

        void Start(void);
};
