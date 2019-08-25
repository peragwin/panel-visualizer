#include <Arduino.h>
#include "FreeRTOS.h"
// #define PxMATRIX_SPI_SPEED 30000000
// #define PxMATRIX_COLOR_DEPTH 16
// #include <PxMatrix.h>
#include <driver/i2s.h>
#include <I2S.h>
#include <SPI.h>
#include <WindowBuffer.h>
#include <FrequencySensor.h>
#include <Render.h>
#include <color.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>

// #define P_LAT 22
// #define P_A 19
// #define P_B 23
// #define P_C 18
// #define P_D 5
// #define P_OE 2

// #define display_draw_time 0
// #define SCAN_RATE 16

#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT * 3 / 4)

#define I2S_NUM I2S_NUM_0
#define I2S_BCK 26
#define I2S_WS 25
#define I2S_SD 27

// PxMATRIX display(64,32,P_LAT, P_OE,P_A,P_B,P_C,P_D);

// void IRAM_ATTR display_updater(){
//   // Increment the counter and set the time of ISR
//   portENTER_CRITICAL_ISR(&timerMux);
//   display.display(display_draw_time);
//   portEXIT_CRITICAL_ISR(&timerMux);
// }

// long lastDisplay = 0;
// float displayFps = 0;

// void display_update_enable(bool is_enable)
// {
//   if (is_enable)
//   {
//   for (;;) {

//     display.display(0);

//     long now = micros();
//     displayFps = 1000000. / ((float)now - (float)lastDisplay);
//     lastDisplay = now;
//     // timer = timerBegin(0, 40, true);
//     // timerAttachInterrupt(timer, &display_updater, true);
//     // timerAlarmWrite(timer, 1000, true);
//     // timerAlarmEnable(timer);
//   }
//   // else
//   // {
//   //   timerDetachInterrupt(timer);
//   //   timerAlarmDisable(timer);
//   // }
//   }
// }

SPIClass vspi(VSPI);
Color_RGB8 displayBuffer[DISPLAY_BUFFER_SIZE];

void HandleESPError(esp_err_t err, String msg) {
  if (err != ESP_OK) {
    Serial.println("ESP ERROR!");
    Serial.println(err);
    Serial.println(msg);
  }
}

#define AUDIO_INPUT_FRAME_SIZE 256
#define AUDIO_BUFFER_SIZE 512
#define CHANNEL_NUMBER 0

static const i2s_config_t i2s_config = {
  mode : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
  sample_rate : 48000,
  bits_per_sample : I2S_BITS_PER_SAMPLE_32BIT,
  channel_format : I2S_CHANNEL_FMT_RIGHT_LEFT,
  communication_format : (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB | I2S_COMM_FORMAT_I2S),
  intr_alloc_flags: 0,
  dma_buf_count: 4,
  dma_buf_len: AUDIO_INPUT_FRAME_SIZE / 2,
  use_apll : true
};

static const i2s_pin_config_t i2s_pin_config = {
  bck_io_num : I2S_BCK,
  ws_io_num : I2S_WS,
  data_out_num : I2S_PIN_NO_CHANGE,
  data_in_num : I2S_SD
};

I2S *i2sHandle;
QueueHandle_t audioReady;

void setupI2S() {
  // audioReady = xQueueCreate(8, 4);
  // i2sHandle = new I2S(0, audioReady);
  
  // i2sHandle->allocateDMABuffers(4*AUDIO_BUFFER_SIZE/AUDIO_INPUT_FRAME_SIZE, AUDIO_INPUT_FRAME_SIZE);
  // i2sHandle->initAudioRXMode(&i2s_pin_config, &i2s_config);

  HandleESPError(
		i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL),
		"failed to install i2s driver");
	HandleESPError(
		i2s_set_pin(I2S_NUM, &i2s_pin_config),
		"failed to config i2s pins");
	HandleESPError(
		i2s_set_sample_rates(I2S_NUM, i2s_config.sample_rate),
		"failed to set i2s sample rate");
}

void startI2S() {
  i2sHandle->startRX();
}

#define NUM_BUCKETS 16
#define NUM_FRAMES 60

Audio_Processor_t *audioProcessor;
int rawBuffer[AUDIO_INPUT_FRAME_SIZE*2];
long processTime = 0;

TaskHandle_t audioUpdateTask;
void processAudioUpdate(void *arg) {

  WindowBuffer *audioBuffer = new WindowBuffer(AUDIO_BUFFER_SIZE);
  
  float convBuffer[AUDIO_INPUT_FRAME_SIZE];
  float frame[AUDIO_BUFFER_SIZE];

  audioProcessor = NewAudioProcessor(AUDIO_BUFFER_SIZE, NUM_BUCKETS, NUM_FRAMES, NULL);


  setupI2S();

  TickType_t xlastWakeTime = xTaskGetTickCount();
  
  for (;;) {

    size_t bytesRead;
    HandleESPError(
      i2s_read(I2S_NUM, rawBuffer, 4*2*AUDIO_INPUT_FRAME_SIZE, &bytesRead, (size_t)(AUDIO_INPUT_FRAME_SIZE / 48)),
        "failed to rx i2s");
    if (!bytesRead) continue;

    vTaskDelayUntil(&xlastWakeTime, AUDIO_INPUT_FRAME_SIZE / 48);

    long now = micros();

    for (int i = CHANNEL_NUMBER; i < 2*AUDIO_INPUT_FRAME_SIZE; i+=2) {
      convBuffer[i/2] = (float)(rawBuffer[i] >> 8);
    }

    audioBuffer->push(convBuffer, AUDIO_INPUT_FRAME_SIZE);
    audioBuffer->get(frame, AUDIO_BUFFER_SIZE);

    Audio_Process(audioProcessor, frame);

    processTime = micros() - now;
  }
}

TaskHandle_t render_task;
TaskHandle_t subRender1;
TaskHandle_t subRender2;
void render(void *arg);
void renderLeft(void *arg);
void renderRight(void *arg);

TaskHandle_t draw_task;
void draw(void *arg);

TaskHandle_t server_task;
void serve(void *arg);

void setup() {
  Serial.begin(115200);
  
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  // SPI.setHwCs(true);
  SPI.setFrequency(8000000);
  SPI.begin(14, 12, 13, 4);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  // display.begin(SCAN_RATE);
  // display.setCursor(0,0);
  // display.print("Hello World");
  // display.setBrightness(255);
  // display.setFastUpdate(true);

  xTaskCreate(
    render,
    "render",
    8000,
    NULL,
    6,
    &render_task
  );

  xTaskCreatePinnedToCore(
    renderLeft,
    "renderLeft",
    8000,
    NULL,
    5,
    &subRender1,
    1
  );

  xTaskCreatePinnedToCore(
    renderRight,
    "renderRight",
    8000,
    NULL,
    5,
    &subRender2,
    0
  );

  xTaskCreatePinnedToCore(
    draw,
    "draw",
    16000,
    NULL,
    1,
    &draw_task,
    1
  );

  xTaskCreatePinnedToCore(
    processAudioUpdate,
    "audioUpdate",
    24000,
    NULL,
    4,
    &audioUpdateTask,
    0
  );

  xTaskCreatePinnedToCore(
    serve,
    "server",
    24000,
    NULL,
    2,
    &server_task,
    0
  );
  // serve(NULL);


  // display_update_enable(true);
}

float clutBuffer[90*25*3];
RenderMode3_t *renderer;

void showDisplay(int currentBuffer) {
  Color_ABGRf *buffer = renderer->buffer[renderer->currentBuffer];
  Color_ABGRf copy[DISPLAY_BUFFER_SIZE/3];
  for (int i = 0; i < DISPLAY_BUFFER_SIZE/3; i++) {
    copy[i] = buffer[i];
  }

  for (int y = 0; y < DISPLAY_HEIGHT/2; y++) {
    for (int x = 0; x < DISPLAY_WIDTH/2; x++) {
      int bidx = x + y * DISPLAY_WIDTH/2;
      Color_ABGRf c = copy[bidx];

      displayBuffer[bidx] = {
        (uint8_t)(255 * c.r),
        (uint8_t)(255 * c.g),
        (uint8_t)(255 * c.b)
      };
    }
  }
  // for (int i = 0; i < DISPLAY_BUFFER_SIZE / 128; i++) {
    digitalWrite(4, LOW);
    vTaskDelay(1);
    SPI.writeBytes((uint8_t*)displayBuffer, DISPLAY_BUFFER_SIZE);
    digitalWrite(4, HIGH);
  // }
}
void setPixel(int x, int y, Color_ABGR c) {
  // display.drawPixel(x, y, display.color565(c.r, c.g, c.b));
}

int targetFrameTime = 25;

void render(void *arg) {
  Serial.println("start render");

  Render3_Params_t renderParams = {
        .warpScale = 16,
        .warpOffset = .6,
        .scaleScale = 1.5,
        .scaleOffset = .5,
        .aspect = 1,
    };

    ColorParams_t colorParams = {
        .valueScale = 1.5,
        .valueOffset = 0,
        .saturationScale = .6,
        .saturationOffset = .1,
        .alphaScale = 0,
        .alphaOffset = 0,
        .maxAlpha = 0.5,
        .period = DISPLAY_WIDTH * 3,
        .colorScale = .05,
        .gamut = {
            .red = 1,
            .green = 1,
            .blue = 1,
        },
        .clut = clutBuffer,
    };

  renderer = NewRender3(DISPLAY_WIDTH, DISPLAY_HEIGHT, NUM_BUCKETS, NUM_FRAMES,
        &renderParams, &colorParams,
        setPixel,
        showDisplay
    );

  TickType_t lastTime = xTaskGetTickCount();

  long t = micros();

  for (;;) {
    if (!audioProcessor) {
      vTaskDelay(100);
      continue;
    }

    Render3(renderer, audioProcessor->fs->drivers);

    vTaskDelayUntil(&lastTime, targetFrameTime);
  }
}

void renderLeft(void *arg) {
  while (!renderer || !audioProcessor) {
    vTaskDelay(100);
  }

  for (;;) {
    Render3Left(renderer, audioProcessor->fs->drivers);
  }
}

void renderRight(void *arg) {
  while (!renderer || !audioProcessor) {
    vTaskDelay(100);
  }

  for (;;) {
    Render3Right(renderer, audioProcessor->fs->drivers);
  }
}

void draw(void *arg) {
  while (!renderer) {
    vTaskDelay(100);
  }

  for (;;) {
    Render3Write(renderer);
  }
}

void loop() {
  // Serial.println("loop");
  // delay(1000);
}

const char* ssid = "dlink-AF94";
const char* password = "aipwz24505";
AsyncWebServer server(80);

void makeJsonResponse(JsonObject root) {
  root["frameRate"] = 1000 / targetFrameTime;

  JsonObject renderParams = root.createNestedObject("renderParams");
  renderParams["valueScale"] = renderer->colorParams->valueScale;
  renderParams["valueOffset"] = renderer->colorParams->valueOffset;
  renderParams["saturationScale"] = renderer->colorParams->saturationScale;
  renderParams["saturationOffset"] = renderer->colorParams->saturationOffset;
  renderParams["alphaScale"] = renderer->colorParams->alphaScale;
  renderParams["alphaOffset"] = renderer->colorParams->alphaOffset;
  renderParams["maxAlpha"] = renderer->colorParams->maxAlpha;
  renderParams["period"] = renderer->colorParams->period;
  renderParams["colorScale"] = renderer->colorParams->colorScale;
  renderParams["period"] = renderer->colorParams->period;
  renderParams["warpScale"] = renderer->params->warpScale;
  renderParams["warpOffset"] = renderer->params->warpOffset;
  renderParams["scaleScale"] = renderer->params->scaleScale;
  renderParams["scaleOffset"] = renderer->params->scaleOffset;
  renderParams["aspect"] = renderer->params->aspect;

  JsonObject audioParams = root.createNestedObject("audioParams");
  audioParams["gain"] = audioProcessor->fs->config->gain;
  audioParams["offset"] = audioProcessor->fs->config->offset;
  audioParams["diffGain"] = audioProcessor->fs->config->diffGain;
  audioParams["diffAbs"] = audioProcessor->fs->config->diffAbs;
  audioParams["sync"] = audioProcessor->fs->config->sync;
  audioParams["preemph"] = audioProcessor->fs->config->preemph;
  audioParams["mode"] = audioProcessor->fs->config->mode;
  audioParams["columnDivider"] = audioProcessor->fs->config->columnDivider;

  JsonObject gcParams = audioParams.createNestedObject("gainController");
  gcParams["kp"] = audioProcessor->fs->gc->kp;
  gcParams["kd"] = audioProcessor->fs->gc->kd;
  JsonArray gcFilterParams = gcParams.createNestedArray("filterParams");
  gcFilterParams.add(audioProcessor->fs->gc->filter->params[0]);
  gcFilterParams.add(audioProcessor->fs->gc->filter->params[1]);

  JsonObject filterParams = audioParams.createNestedObject("filterParams");
  JsonArray gainFilterParams = filterParams.createNestedArray("gainFilter");
  gainFilterParams.add(audioProcessor->fs->gainFilter->params[0]);
  gainFilterParams.add(audioProcessor->fs->gainFilter->params[1]);
  JsonArray gainFeedbackParams = filterParams.createNestedArray("gainFeedback");
  gainFeedbackParams.add(audioProcessor->fs->gainFeedback->params[0]);
  gainFeedbackParams.add(audioProcessor->fs->gainFeedback->params[1]);
  JsonArray diffFilterParams = filterParams.createNestedArray("diffFilter");
  diffFilterParams.add(audioProcessor->fs->diffFilter->params[0]);
  diffFilterParams.add(audioProcessor->fs->diffFilter->params[1]);
  JsonArray diffFeedbackParams = filterParams.createNestedArray("diffFeedback");
  diffFeedbackParams.add(audioProcessor->fs->diffFeedback->params[0]);
  diffFeedbackParams.add(audioProcessor->fs->diffFeedback->params[1]);
  JsonArray scaleFilterParams = filterParams.createNestedArray("scaleFilter");
  scaleFilterParams.add(audioProcessor->fs->scaleFilter->params[0]);
  scaleFilterParams.add(audioProcessor->fs->scaleFilter->params[1]);
  scaleFilterParams.add(audioProcessor->fs->scaleFilter->params[2]);
  scaleFilterParams.add(audioProcessor->fs->scaleFilter->params[3]);
}

void setParamsFromJson(JsonObject root) {

  Serial.println("set params from json");
  bool hasKey;

  hasKey = root.containsKey("frameRate");
  if (hasKey) {
    int frameRate = root["frameRate"];
    targetFrameTime = 1000 / frameRate;
  }

  JsonObject renderParams = root.getMember("renderParams");
  if (!renderParams.isNull()) {
    Serial.println("rendr parms not null");
    hasKey = renderParams.containsKey("valueScale");
    if (hasKey) renderer->colorParams->valueScale = renderParams.getMember("valueScale");
    hasKey = renderParams.containsKey("valueOffset");
    if (hasKey) {
      Serial.println("value offset set");
      renderer->colorParams->valueOffset = renderParams.getMember("valueOffset");
    } else {
      float f = renderParams["valueOffset"];
      serializeJson(renderParams, Serial); //.println(f);
    }
    hasKey = renderParams.containsKey("saturationScale");
    if (hasKey) renderer->colorParams->saturationScale = renderParams.getMember("saturationScale");
    hasKey = renderParams.containsKey("saturationOffset");
    if (hasKey) renderer->colorParams->saturationOffset = renderParams.getMember("saturationOffset");
    hasKey = renderParams.containsKey("alphaScale");
    if (hasKey) renderer->colorParams->alphaScale = renderParams.getMember("alphaScale");
    hasKey = renderParams.containsKey("alphaOffset");
    if (hasKey) renderer->colorParams->alphaOffset = renderParams.getMember("alphaOffset");
    hasKey = renderParams.containsKey("maxAlpha");
    if (hasKey) renderer->colorParams->maxAlpha = renderParams.getMember("maxAlpha");
    hasKey = renderParams.containsKey("period");
    if (hasKey) renderer->colorParams->period = renderParams.getMember("period");
    hasKey = renderParams.containsKey("colorScale");
    if (hasKey) renderer->colorParams->colorScale = renderParams.getMember("colorScale");
    hasKey = renderParams.containsKey("period");
    if (hasKey) renderer->colorParams->period = renderParams.getMember("period");
    hasKey = renderParams.containsKey("warpScale");
    if (hasKey) renderer->params->warpScale = renderParams.getMember("warpScale");
    hasKey = renderParams.containsKey("warpOffset");
    if (hasKey) renderer->params->warpOffset = renderParams.getMember("warpOffset");
    hasKey = renderParams.containsKey("scaleScale");
    if (hasKey) renderer->params->scaleScale = renderParams.getMember("scaleScale");
    hasKey = renderParams.containsKey("scaleOffset");
    if (hasKey) renderer->params->scaleOffset = renderParams.getMember("scaleOffset");
    hasKey = renderParams.containsKey("aspect");
    if (hasKey) renderer->params->aspect = renderParams.getMember("aspect");
  }

  JsonObject audioParams = root.getMember("audioParams");
  if (!audioParams.isNull()) {
    hasKey = audioParams.containsKey("gain");
    if (hasKey) audioProcessor->fs->config->gain = audioParams["gain"];
    hasKey = audioParams.containsKey("offset");
    if (hasKey) audioProcessor->fs->config->offset = audioParams["offset"];
    hasKey = audioParams.containsKey("diffGain");
    if (hasKey) audioProcessor->fs->config->diffGain = audioParams["diffGain"];
    hasKey = audioParams.containsKey("diffAbs");
    if (hasKey) audioProcessor->fs->config->diffAbs = audioParams["diffAbs"];
    hasKey = audioParams.containsKey("sync");
    if (hasKey) audioProcessor->fs->config->sync = audioParams["sync"];
    hasKey = audioParams.containsKey("preemph");
    if (hasKey) audioProcessor->fs->config->preemph = audioParams["preemph"];
    hasKey = audioParams.containsKey("mode");
    if (hasKey) audioProcessor->fs->config->mode = audioParams["mode"];
    hasKey = audioParams.containsKey("columnDivider");
    if (hasKey) audioProcessor->fs->config->columnDivider = audioParams["columnDivider"];

    JsonObject gcParams = audioParams.getMember("gainController");
    if (!gcParams.isNull()) {
      hasKey = gcParams.containsKey("kp");
      if (hasKey) audioProcessor->fs->gc->kp = gcParams["kp"];
      hasKey = gcParams.containsKey("kd");
      if (hasKey) audioProcessor->fs->gc->kd = gcParams["kd"];

      JsonArray gcFilterParams = gcParams.getMember("filterParams");
      if (!gcFilterParams.isNull() && gcFilterParams.size() == 2) {
        for (int i = 0; i < 2; i++) {
          audioProcessor->fs->gc->filter->params[i] = gcFilterParams.getElement(i);
        }
      }
    }

    JsonObject filterParams = audioParams.getMember("filterParams");
    if (!filterParams.isNull()) {
      JsonArray params;
      params = filterParams.getMember("gainFilter");
      if (!params.isNull() && params.size() == 2) {
        for (int i = 0; i < 2; i++) {
          audioProcessor->fs->gainFilter->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("gainFeedback");
      if (!params.isNull() && params.size() == 2) {
        for (int i = 0; i < 2; i++) {
          audioProcessor->fs->gainFeedback->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("diffFilter");
      if (!params.isNull() && params.size() == 2) {
        for (int i = 0; i < 2; i++) {
          audioProcessor->fs->diffFilter->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("diffFeedback");
      if (!params.isNull() && params.size() == 2) {
        for (int i = 0; i < 2; i++) {
          audioProcessor->fs->diffFeedback->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("scaleFilter");
      if (!params.isNull() && params.size() == 4) {
        for (int i = 0; i < 4; i++) {
          audioProcessor->fs->scaleFilter->params[i] = params.getElement(i);
        }
      }
    }
  }
}

void makeDebugResponse(JsonObject root, bool getRawInput) {
  JsonArray buckets = root.createNestedArray("buckets");
  for (int i = 0; i < audioProcessor->fs->size; i++) {
    buckets.add(audioProcessor->buckets[i]);
  }
  
  if (getRawInput) {
    JsonArray rawInput = root.createNestedArray("rawInput");
    for (int i = CHANNEL_NUMBER; i < AUDIO_INPUT_FRAME_SIZE*2; i+=2) {
      rawInput.add(rawBuffer[i]);
    }
  }

  JsonObject perf = root.createNestedObject("perf");
  perf["renderFps"] = renderer->fps;
  // perf["displayFps"] = displayFps;
  perf["renderInitTime"] = renderer->initTime;
  perf["renderWarpTime"] = renderer->warpTime * renderer->rows * renderer->columns;
  perf["renderColorTime"] = renderer->colorTime * renderer->rows * renderer->columns;
  perf["renderDrawTime"] = renderer->drawTime;
  perf["renderWriteTime"] = renderer->writeTime;
  perf["queueLeftTime"] = renderer->queueLeftTime;
  perf["queueRightTime"] = renderer->queueRightTime;
  perf["processLeftTime"] = renderer->processLeftTime;
  perf["processRightTime"] = renderer->processRightTime;

  perf["audioProcessTime"] = processTime;
  
  JsonObject drivers = root.createNestedObject("drivers");
  
  JsonArray amp = drivers.createNestedArray("amp");
  float *ampValues = FS_GetColumn(audioProcessor->fs->drivers, 0);
  for (int i = 0; i < audioProcessor->fs->size; i++) {
    amp.add(ampValues[i]);
  }

  JsonArray diff = drivers.createNestedArray("diff");
  for (int i = 0; i < audioProcessor->fs->size; i++) {
    diff.add(audioProcessor->fs->drivers->diff[i]);
  }

  JsonArray energy = drivers.createNestedArray("energy");
  for (int i = 0; i < audioProcessor->fs->size; i++) {
    energy.add(audioProcessor->fs->drivers->energy[i]);
  }

  JsonArray scales = drivers.createNestedArray("scales");
  for (int i = 0; i < audioProcessor->fs->size; i++) {
    scales.add(audioProcessor->fs->drivers->scales[i]);
  }

  serializeJson(root, Serial);
}

void serve(void *arg) {
  while (!renderer || !audioProcessor) vTaskDelay(10);

  // WebServer server(80, ssid, password, renderer, audioProcessor);

  Serial.println("Init WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi Failed!\n");
    return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncJsonResponse * response = new AsyncJsonResponse();
    response->addHeader("Server", "LED Panel Server");
    JsonObject root = response->getRoot();
    makeJsonResponse(root);
    response->setLength();

    request->send(response);
  });

  AsyncCallbackJsonWebHandler* handler = new AsyncCallbackJsonWebHandler("/",
    [](AsyncWebServerRequest *request, JsonVariant json) {
      JsonObject obj = json.as<JsonObject>();
  
      setParamsFromJson(obj);

      AsyncJsonResponse * response = new AsyncJsonResponse();
      response->addHeader("Server", "LED Panel Server");
      JsonObject root = response->getRoot();
      makeJsonResponse(root);
      response->setLength();

      request->send(response);
  });
  server.addHandler(handler);

  // server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
  //   [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
  //     if (len > 1024) request->send(400, "text/plain", "request body too large");

  //     DynamicJsonDocument doc(1024);
  //     DeserializationError error = deserializeJson(doc, data, len);
  //     if (error) {
  //       request->send(400, "text/plain", "could not parse request body");
  //       return;
  //     }

  // });

  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncJsonResponse * response = new AsyncJsonResponse(8196);
    response->addHeader("Server", "LED Panel Server");
    JsonObject root = response->getRoot();
    
    // bool getRawInput = request->getParam("rawInput");
    makeDebugResponse(root, false);

    response->setLength();
    request->send(response);
  });

  server.onNotFound([] (AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  });

  Serial.println("begin server...");

  server.begin();

  for (;;) {
    vTaskDelay(1000);
  }
}