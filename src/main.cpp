#include <Arduino.h>
#include "FreeRTOS.h"
#include <driver/i2s.h>
#include <WindowBuffer.h>
#include <FrequencySensor.h>
#include <Render.h>
#include <color.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncJson.h>
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT * 3 / 4)

#define NUM_BUCKETS 16
#define NUM_FRAMES 64

#define I2S_NUM I2S_NUM_0
#define I2S_BCK 22
#define I2S_WS 2
#define I2S_SD 32
#define I2S_SO 33

#include "wifi_credentials.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

AsyncWebServer server(80);

MatrixPanel_I2S_DMA *display = nullptr;
Color_RGB8 displayBuffer[DISPLAY_BUFFER_SIZE];

MatrixPanel_I2S_DMA *setupDisplay()
{
  HUB75_I2S_CFG mxconfig(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  mxconfig.driver = HUB75_I2S_CFG::FM6126A;

  auto display = new MatrixPanel_I2S_DMA(mxconfig);
  display->setBrightness8(255); // 75%
  display->setMinRefreshRate(60);
  if (!display->begin())
  {
    Serial.println("failed to allocate for display!");
  }
  return display;
}

void HandleESPError(esp_err_t err, String msg)
{
  if (err != ESP_OK)
  {
    Serial.println("ESP ERROR!");
    Serial.println(err);
    Serial.println(msg);
  }
}

#define AUDIO_INPUT_FRAME_SIZE 256
#define AUDIO_BUFFER_SIZE 512
#define CHANNEL_NUMBER 0

void setupI2S()
{
  i2s_config_t i2s_config = {
    mode : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate : 48000,
    bits_per_sample : I2S_BITS_PER_SAMPLE_32BIT,
    channel_format : I2S_CHANNEL_FMT_RIGHT_LEFT,
    communication_format : (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB | I2S_COMM_FORMAT_I2S),
    intr_alloc_flags : 0,
    dma_buf_count : 4,
    dma_buf_len : AUDIO_INPUT_FRAME_SIZE / 2,
    use_apll : true
  };

  i2s_pin_config_t i2s_pin_config = {
    bck_io_num : I2S_BCK,
    ws_io_num : I2S_WS,
    data_out_num : I2S_SO,
    data_in_num : I2S_SD
  };

  HandleESPError(
      i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL),
      "failed to install i2s driver");
  HandleESPError(
      i2s_set_pin(I2S_NUM, &i2s_pin_config),
      "failed to config i2s pins");
  HandleESPError(
      i2s_set_sample_rates(I2S_NUM, i2s_config.sample_rate),
      "failed to set i2s sample rate");

  i2s_start(I2S_NUM_0);
}

Audio_Processor_t *audioProcessor;
int rawBuffer[AUDIO_INPUT_FRAME_SIZE * 2];
long processTime = 0;

TaskHandle_t audioUpdateTask;
void processAudioUpdate(void *arg)
{

  WindowBuffer *audioBuffer = new WindowBuffer(AUDIO_BUFFER_SIZE);

  float convBuffer[AUDIO_INPUT_FRAME_SIZE];
  float frame[AUDIO_BUFFER_SIZE];

  audioProcessor = NewAudioProcessor(AUDIO_BUFFER_SIZE, NUM_BUCKETS, NUM_FRAMES, NULL);

  setupI2S();

  TickType_t xlastWakeTime = xTaskGetTickCount();

  for (;;)
  {

    size_t bytesRead;
    HandleESPError(
        i2s_read(I2S_NUM, rawBuffer, 4 * 2 * AUDIO_INPUT_FRAME_SIZE, &bytesRead, (size_t)(AUDIO_INPUT_FRAME_SIZE / 48)),
        "failed to rx i2s");
    if (!bytesRead)
      continue;

    vTaskDelayUntil(&xlastWakeTime, AUDIO_INPUT_FRAME_SIZE / 48);

    long now = micros();

    for (int i = CHANNEL_NUMBER; i < 2 * AUDIO_INPUT_FRAME_SIZE; i += 2)
    {
      convBuffer[i / 2] = (float)((rawBuffer[i] + rawBuffer[i + 1]) >> 8);
    }

    audioBuffer->push(convBuffer, AUDIO_INPUT_FRAME_SIZE);
    audioBuffer->get(frame, AUDIO_BUFFER_SIZE);

    Audio_Process(audioProcessor, frame);

    processTime = micros() - now;
  }
}

TaskHandle_t render_task;
TaskHandle_t renderLeftTask;
TaskHandle_t renderRightTask;
void render(void *arg);
void renderLeft(void *arg);
void renderRight(void *arg);

TaskHandle_t draw_task;
void draw(void *arg);

TaskHandle_t server_task;
void serve(void *arg);

void setup()
{
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
      render,
      "render",
      8000,
      NULL,
      6,
      &render_task,
      1);

  xTaskCreatePinnedToCore(
      processAudioUpdate,
      "audioUpdate",
      24000,
      NULL,
      4,
      &audioUpdateTask,
      0);

  serve(NULL);
}

float clutBuffer[90 * 25 * 3];
Render3 *renderer;

// frame time in ms (ticks)
int targetFrameTime = 6;
float renderFps = 0.0;

void render(void *arg)
{
  Serial.println("start render");

  Render3_Params_t renderParams = {
      .warpScale = 16,
      .warpOffset = .6,
      .scaleScale = 2,
      .scaleOffset = .5,
      .aspectX = 1,
      .aspectY = 1,
  };

  ColorParams_t colorParams = {
      .valueScale = 2,
      .valueOffset = 0,
      .saturationScale = .6,
      .saturationOffset = .1,
      .alphaScale = 1.,
      .alphaOffset = 0.,
      .maxAlpha = 0.5,
      .period = DISPLAY_WIDTH * 3,
      .colorScale = .25,
      .gamut = {
          .red = 1,
          .green = 1,
          .blue = 1,
      },
      .clut = clutBuffer,
  };

  auto display = setupDisplay();

  auto show = [=](Render3 *r) {
    // Serial.println("show display!");
    Color_RGB *buffer = renderer->getCurrentBuffer();
    auto h = DISPLAY_HEIGHT / 2;
    auto w = DISPLAY_WIDTH / 2;
    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        int bidx = x + y * w;
        Color_RGB c = buffer[bidx];
        Color_RGB8 c8 = {
            (uint8_t)(255 * c.r),
            (uint8_t)(255 * c.g),
            (uint8_t)(255 * c.b),
        };
        display->drawPixelRGB888(w + x, h + y, c8.r, c8.g, c8.b);
        display->drawPixelRGB888(w - x - 1, h + y, c8.r, c8.g, c8.b);
        display->drawPixelRGB888(w + x, h - y - 1, c8.r, c8.g, c8.b);
        display->drawPixelRGB888(w - x - 1, h - y - 1, c8.r, c8.g, c8.b);
      }
    }
  };

  renderer = new Render3(DISPLAY_WIDTH, DISPLAY_HEIGHT, NUM_FRAMES, NUM_BUCKETS,
                         &renderParams, &colorParams,
                         show);

  int frameCount = 0;
  TickType_t fpsTime = xTaskGetTickCount();

  for (;;)
  {
    if (!audioProcessor)
    {
      vTaskDelay(100);
      continue;
    }

    TickType_t lastTime = xTaskGetTickCount();
    frameCount++;
    if (frameCount % 256 == 0)
    {
      auto now = xTaskGetTickCount();
      auto e = 1000.0 / ((float)(now - fpsTime) + 0.001);
      fpsTime = now;
      renderFps = .9 * renderFps + .1 * 256.0 * e;
    }

    renderer->render(audioProcessor->fs->drivers);

    vTaskDelayUntil(&lastTime, targetFrameTime);
  }
}

void loop()
{
  Serial.println(WiFi.localIP());
  WiFi.printDiag(Serial);
  delay(8000);
}

void makeJsonResponse(JsonObject root)
{
  root["targetFrameRate"] = 1000 / targetFrameTime;
  root["actualFrameRate"] = renderFps;

  JsonArray size = root.createNestedArray("size");
  size.add(renderer->getRows());
  size.add(renderer->getColumns());

  auto cp = renderer->getColorParams();
  auto ps = renderer->getParams();
  JsonObject renderParams = root.createNestedObject("renderParams");
  renderParams["valueScale"] = cp->valueScale;
  renderParams["valueOffset"] = cp->valueOffset;
  renderParams["saturationScale"] = cp->saturationScale;
  renderParams["saturationOffset"] = cp->saturationOffset;
  renderParams["alphaScale"] = cp->alphaScale;
  renderParams["alphaOffset"] = cp->alphaOffset;
  renderParams["maxAlpha"] = cp->maxAlpha;
  renderParams["period"] = cp->period;
  renderParams["colorScale"] = cp->colorScale;
  renderParams["period"] = cp->period;
  renderParams["warpScale"] = ps->warpScale;
  renderParams["warpOffset"] = ps->warpOffset;
  renderParams["scaleScale"] = ps->scaleScale;
  renderParams["scaleOffset"] = ps->scaleOffset;
  renderParams["aspectX"] = ps->aspectX;
  renderParams["aspectY"] = ps->aspectY;

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

void setParamsFromJson(JsonObject root)
{

  Serial.println("set params from json");
  bool hasKey;

  hasKey = root.containsKey("targetFrameRate");
  if (hasKey)
  {
    int frameRate = root["targetFrameRate"];
    targetFrameTime = 1000 / frameRate;
  }

  JsonArray size = root["size"];
  if (!size.isNull() && size.size() == 2)
  {
    int rows = size.getElement(0);
    int columns = size.getElement(1);
    AP_SetSize(audioProcessor, rows, columns);
    renderer->setSize(rows, columns);
  }

  auto cp = renderer->getColorParams();
  auto ps = renderer->getParams();

  JsonObject renderParams = root.getMember("renderParams");
  if (!renderParams.isNull())
  {
    Serial.println("rendr parms not null");
    hasKey = renderParams.containsKey("valueScale");
    if (hasKey)
      cp->valueScale = renderParams.getMember("valueScale");
    hasKey = renderParams.containsKey("valueOffset");
    if (hasKey)
    {
      Serial.println("value offset set");
      cp->valueOffset = renderParams.getMember("valueOffset");
    }
    // else
    // {
    //   float f = renderParams["valueOffset"];
    //   serializeJson(renderParams, Serial); //.println(f);
    // }
    hasKey = renderParams.containsKey("saturationScale");
    if (hasKey)
      cp->saturationScale = renderParams.getMember("saturationScale");
    hasKey = renderParams.containsKey("saturationOffset");
    if (hasKey)
      cp->saturationOffset = renderParams.getMember("saturationOffset");
    hasKey = renderParams.containsKey("alphaScale");
    if (hasKey)
      cp->alphaScale = renderParams.getMember("alphaScale");
    hasKey = renderParams.containsKey("alphaOffset");
    if (hasKey)
      cp->alphaOffset = renderParams.getMember("alphaOffset");
    hasKey = renderParams.containsKey("maxAlpha");
    if (hasKey)
      cp->maxAlpha = renderParams.getMember("maxAlpha");
    hasKey = renderParams.containsKey("period");
    if (hasKey)
      cp->period = renderParams.getMember("period");
    hasKey = renderParams.containsKey("colorScale");
    if (hasKey)
      cp->colorScale = renderParams.getMember("colorScale");
    hasKey = renderParams.containsKey("period");
    if (hasKey)
      cp->period = renderParams.getMember("period");
    hasKey = renderParams.containsKey("warpScale");
    if (hasKey)
      ps->warpScale = renderParams.getMember("warpScale");
    hasKey = renderParams.containsKey("warpOffset");
    if (hasKey)
      ps->warpOffset = renderParams.getMember("warpOffset");
    hasKey = renderParams.containsKey("scaleScale");
    if (hasKey)
      ps->scaleScale = renderParams.getMember("scaleScale");
    hasKey = renderParams.containsKey("scaleOffset");
    if (hasKey)
      ps->scaleOffset = renderParams.getMember("scaleOffset");
    hasKey = renderParams.containsKey("aspectX");
    if (hasKey)
      ps->aspectX = renderParams.getMember("aspectX");
    hasKey = renderParams.containsKey("aspectY");
    if (hasKey)
      ps->aspectY = renderParams.getMember("aspectY");
    hasKey = renderParams.containsKey("aspect");
  }

  JsonObject audioParams = root.getMember("audioParams");
  if (!audioParams.isNull())
  {
    hasKey = audioParams.containsKey("gain");
    if (hasKey)
      audioProcessor->fs->config->gain = audioParams["gain"];
    hasKey = audioParams.containsKey("offset");
    if (hasKey)
      audioProcessor->fs->config->offset = audioParams["offset"];
    hasKey = audioParams.containsKey("diffGain");
    if (hasKey)
      audioProcessor->fs->config->diffGain = audioParams["diffGain"];
    hasKey = audioParams.containsKey("diffAbs");
    if (hasKey)
      audioProcessor->fs->config->diffAbs = audioParams["diffAbs"];
    hasKey = audioParams.containsKey("sync");
    if (hasKey)
      audioProcessor->fs->config->sync = audioParams["sync"];
    hasKey = audioParams.containsKey("preemph");
    if (hasKey)
      audioProcessor->fs->config->preemph = audioParams["preemph"];
    hasKey = audioParams.containsKey("mode");
    if (hasKey)
      audioProcessor->fs->config->mode = audioParams["mode"];
    hasKey = audioParams.containsKey("columnDivider");
    if (hasKey)
      audioProcessor->fs->config->columnDivider = audioParams["columnDivider"];

    JsonObject gcParams = audioParams.getMember("gainController");
    if (!gcParams.isNull())
    {
      hasKey = gcParams.containsKey("kp");
      if (hasKey)
        audioProcessor->fs->gc->kp = gcParams["kp"];
      hasKey = gcParams.containsKey("kd");
      if (hasKey)
        audioProcessor->fs->gc->kd = gcParams["kd"];

      JsonArray gcFilterParams = gcParams.getMember("filterParams");
      if (!gcFilterParams.isNull() && gcFilterParams.size() == 2)
      {
        for (int i = 0; i < 2; i++)
        {
          audioProcessor->fs->gc->filter->params[i] = gcFilterParams.getElement(i);
        }
      }
    }

    JsonObject filterParams = audioParams.getMember("filterParams");
    if (!filterParams.isNull())
    {
      JsonArray params;
      params = filterParams.getMember("gainFilter");
      if (!params.isNull() && params.size() == 2)
      {
        for (int i = 0; i < 2; i++)
        {
          audioProcessor->fs->gainFilter->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("gainFeedback");
      if (!params.isNull() && params.size() == 2)
      {
        for (int i = 0; i < 2; i++)
        {
          audioProcessor->fs->gainFeedback->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("diffFilter");
      if (!params.isNull() && params.size() == 2)
      {
        for (int i = 0; i < 2; i++)
        {
          audioProcessor->fs->diffFilter->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("diffFeedback");
      if (!params.isNull() && params.size() == 2)
      {
        for (int i = 0; i < 2; i++)
        {
          audioProcessor->fs->diffFeedback->params[i] = params.getElement(i);
        }
      }
      params = filterParams.getMember("scaleFilter");
      if (!params.isNull() && params.size() == 4)
      {
        for (int i = 0; i < 4; i++)
        {
          audioProcessor->fs->scaleFilter->params[i] = params.getElement(i);
        }
      }
    }
  }
}

void makeDebugResponse(JsonObject root, bool getRawInput)
{
  JsonArray buckets = root.createNestedArray("buckets");
  for (int i = 0; i < audioProcessor->fs->size; i++)
  {
    buckets.add(audioProcessor->buckets[i]);
  }

  if (getRawInput)
  {
    JsonArray left = root.createNestedArray("left");
    JsonArray right = root.createNestedArray("right");
    for (int i = 0; i < AUDIO_INPUT_FRAME_SIZE * 2; i += 2)
    {
      left.add(rawBuffer[i]);
      right.add(rawBuffer[i + 1]);
    }
    return;
  }

  JsonObject perf = root.createNestedObject("perf");
  perf["renderFps"] = renderer->fps;
  perf["displayFps"] = renderFps;
  perf["renderInitTime"] = renderer->initTime;
  perf["renderColorTime"] = renderer->colorTime * renderer->getRows() * renderer->getColumns();
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
  for (int i = 0; i < audioProcessor->fs->size; i++)
  {
    amp.add(ampValues[i]);
  }

  JsonArray diff = drivers.createNestedArray("diff");
  for (int i = 0; i < audioProcessor->fs->size; i++)
  {
    diff.add(audioProcessor->fs->drivers->diff[i]);
  }

  JsonArray energy = drivers.createNestedArray("energy");
  for (int i = 0; i < audioProcessor->fs->size; i++)
  {
    energy.add(audioProcessor->fs->drivers->energy[i]);
  }

  JsonArray scales = drivers.createNestedArray("scales");
  for (int i = 0; i < audioProcessor->fs->size; i++)
  {
    scales.add(audioProcessor->fs->drivers->scales[i]);
  }

  // serializeJson(root, Serial);
}

void serve(void *arg)
{
  while (!renderer || !audioProcessor)
    vTaskDelay(10);

  // WebServer server(80, ssid, password, renderer, audioProcessor);

  Serial.println("Init WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);

  WiFi.begin(ssid, password);
  while (!WiFi.isConnected())
  {
    Serial.printf("WiFi Failed: %d\n", WiFi.status());
    vTaskDelay(2000);
  }

  auto addr = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(addr);

  // AsyncServer server(80);
  // AsyncWebServer server(addr, 80);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("got request to /");

    AsyncJsonResponse *response = new AsyncJsonResponse();
    response->addHeader("Server", "LED Panel Server");
    JsonObject root = response->getRoot();
    makeJsonResponse(root);
    response->setLength();

    request->send(response);
  });

  AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/",
                                                                         [](AsyncWebServerRequest *request, JsonVariant json) {
                                                                           JsonObject obj = json.as<JsonObject>();

                                                                           setParamsFromJson(obj);

                                                                           AsyncJsonResponse *response = new AsyncJsonResponse();
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

  server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("got /debug");

    AsyncJsonResponse *response = new AsyncJsonResponse(false, 8196);
    response->addHeader("Server", "LED Panel Server");
    JsonObject root = response->getRoot();

    // bool getRawInput = request->getParam("rawInput");
    makeDebugResponse(root, false);

    response->setLength();
    request->send(response);
  });

  server.on("/rawinput", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("get /rawinput");
    AsyncJsonResponse *response = new AsyncJsonResponse(false, 8196);
    JsonObject root = response->getRoot();
    makeDebugResponse(root, true);
    response->setLength();
    request->send(response);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    Serial.println("got request to 404");
    request->send(404, "text/plain", "Not found");
  });

  Serial.println("begin server...");

  server.begin();
}