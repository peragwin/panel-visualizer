#include <Arduino.h>
#include <driver/i2s.h>
#include <WindowBuffer.h>
#include <FrequencySensor.h>
#include <Render.h>
#include <color.h>
#include <WiFi.h>
#ifdef USE_ASYNC_TCP
// #include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
// #include <AsyncJson.h>
#endif
#include <ArduinoJson.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <Universe.h>
#include <audio_coef.h>
#include <BLEDevice.h>
#include "hsluv.h"
#include <Configlets.h>

// #define VUZIC_REVA
// #define VUZIC_REVB
#define VUZIC32_REVA

#ifdef VUZIC_REVA

#define I2S_BCK 14 // 21
#define I2S_WS 15  // 22
#define I2S_SD 4
// #define I2S_SO 23

#define BUTTON_PIN 39

// int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk;
// const HUB75_I2S_CFG::i2s_pins HUB75_DISPLAY_PINS = {25, 26, 27, 14, 12, 13, 32, 33, 5, 2, -1, 17, 15, 16};

#endif

#ifdef VUZIC_REVB

#define I2S_PDM
#define I2S_CLK 12
#define I2S_DAT 14

#define BUTTON_PIN 35

// int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk;
const HUB75_I2S_CFG::i2s_pins HUB75_DISPLAY_PINS = {15, 16, 17, 18, 19, 21, 32, 33, 5, 13, -1, 26, 27, 25};

#endif

#ifdef VUZIC32_REVA

#define I2S_PDM
#define I2S_CLK 12
#define I2S_DAT 14
#define A_BUTTON 36
#define B_BUTTON 0
#define J_BUTTON 39
#define IMU_INT1 34
#define IMU_INT2 35
#define I2S_SCL 23
#define I2S_SDA 22
#define POW_KEY 32
#define LED_OUT 33
#define BUTTON_PIN B_BUTTON

// int8_t r1, g1, b1, r2, g2, b2, a, b, c, d, e, lat, oe, clk;
const HUB75_I2S_CFG::i2s_pins HUB75_DISPLAY_PINS = {16, 17, 5, 19, 18, 21, 2, 4, 15, 13, -1, 26, 27, 25};

#endif

#define DISPLAY_WIDTH 64
#define DISPLAY_HEIGHT 32
#define DISPLAY_BUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT * 3 / 4)

#define NUM_BUCKETS 6 // 16
#define NUM_FRAMES 64

#define AUDIO_I2S_NUM I2S_NUM_0

#define AUDIO_TASK_CORE 0
#define RENDER_TASK_CORE 0
#define ITERATE_TASK_CORE 1

#include "wifi_credentials.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

Registry registry;

// AsyncWebServer *server;

// MatrixPanel_I2S_DMA *display = nullptr;
Color_RGB8 displayBuffer[DISPLAY_WIDTH * DISPLAY_HEIGHT] = {0};

MatrixPanel_I2S_DMA *setupDisplay()
{
  HUB75_I2S_CFG mxconfig(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  mxconfig.driver = HUB75_I2S_CFG::FM6126A;
  mxconfig.gpio = HUB75_DISPLAY_PINS;
  mxconfig.i2sspeed = mxconfig.HZ_20M;
  mxconfig.min_refresh_rate = 120;
  mxconfig.clkphase = false;

  auto display = new MatrixPanel_I2S_DMA(mxconfig);
  display->setBrightness8(255);
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
#ifdef I2S_PDM
  i2s_config_t i2s_config = {
    mode : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    sample_rate : 32000,
    bits_per_sample : I2S_BITS_PER_SAMPLE_16BIT,
    channel_format : I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format : I2S_COMM_FORMAT_STAND_I2S,
    intr_alloc_flags : ESP_INTR_FLAG_LEVEL1,
    dma_buf_count : 8,
    dma_buf_len : AUDIO_INPUT_FRAME_SIZE,
    use_apll : false,
    tx_desc_auto_clear : true,
  };
  i2s_pin_config_t i2s_pin_config = {
    mck_io_num : I2S_PIN_NO_CHANGE,
    bck_io_num : I2S_PIN_NO_CHANGE,
    ws_io_num : I2S_CLK,
    data_out_num : I2S_PIN_NO_CHANGE,
    data_in_num : I2S_DAT,
  };
#else
  i2s_config_t i2s_config = {
    mode : (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate : 48000,
    bits_per_sample : I2S_BITS_PER_SAMPLE_32BIT,
    channel_format : I2S_CHANNEL_FMT_RIGHT_LEFT,
    communication_format : I2S_COMM_FORMAT_STAND_I2S,
    intr_alloc_flags : ESP_INTR_FLAG_LEVEL1,
    dma_buf_count : 4,
    dma_buf_len : AUDIO_INPUT_FRAME_SIZE / 2,
    use_apll : false,
    tx_desc_auto_clear : true,
    fixed_mclk : 0,
    mclk_multiple : I2S_MCLK_MULTIPLE_256,
  };

  i2s_pin_config_t i2s_pin_config = {
    mck_io_num : I2S_PIN_NO_CHANGE,
    bck_io_num : I2S_BCK,
    ws_io_num : I2S_WS,
    data_out_num : I2S_PIN_NO_CHANGE,
    data_in_num : I2S_SD
  };
#endif

  HandleESPError(
      i2s_driver_install(AUDIO_I2S_NUM, &i2s_config, 0, NULL),
      "failed to install i2s driver");
  HandleESPError(
      i2s_set_pin(AUDIO_I2S_NUM, &i2s_pin_config),
      "failed to config i2s pins");
  // HandleESPError(
  //     i2s_set_sample_rates(AUDIO_I2S_NUM, i2s_config.sample_rate),
  //     "failed to set i2s sample rate");
#ifdef I2S_PDM
  HandleESPError(
      i2s_set_clk(AUDIO_I2S_NUM, i2s_config.sample_rate, i2s_config.bits_per_sample, I2S_CHANNEL_MONO),
      "failed to set i2s clock");
#else
  // HandleESPError(
  //     i2s_set_clk(AUDIO_I2S_NUM, i2s_config.sample_rate, i2s_config.bits_per_sample, I2S_CHANNEL_STEREO),
  //     "failed to set i2s clock");
#endif

  // HandleESPError(i2s_start(AUDIO_I2S_NUM), "failed to start i2s");
}

Audio_Processor_t *audioProcessor;
#ifdef I2S_PDM
int16_t rawBuffer[AUDIO_INPUT_FRAME_SIZE];
#else
int rawBuffer[AUDIO_INPUT_FRAME_SIZE * 2];
#endif
long processTime = 0;

TaskHandle_t audioUpdateTask;
void processAudioUpdate(void *arg)
{

  WindowBuffer<AUDIO_BUFFER_SIZE> audioBuffer;

  float convBuffer[AUDIO_INPUT_FRAME_SIZE];
  float frame[AUDIO_BUFFER_SIZE];

  audioProcessor = NewAudioProcessor(AUDIO_BUFFER_SIZE, NUM_BUCKETS, NUM_FRAMES, NULL);

  setupI2S();

  TickType_t xlastWakeTime = xTaskGetTickCount();

  for (;;)
  {

#ifdef I2S_PDM
    size_t frame_size = 2 * AUDIO_INPUT_FRAME_SIZE;
#else
    size_t frame_size = 4 * 2 * AUDIO_INPUT_FRAME_SIZE;
#endif

    size_t bytesRead;
    HandleESPError(
        i2s_read(AUDIO_I2S_NUM, rawBuffer, frame_size, &bytesRead, 999), //(size_t)(AUDIO_INPUT_FRAME_SIZE / 48)),
        "failed to rx i2s");
    if (!bytesRead)
    {
      Serial.println("no i2s read");
      continue;
    }

    vTaskDelayUntil(&xlastWakeTime, AUDIO_INPUT_FRAME_SIZE / 48);

    long now = micros();

#ifdef I2S_PDM
    for (int i = 0; i < AUDIO_INPUT_FRAME_SIZE; i++)
    {
      // Serial.printf("L: %08x\r\n", (int)rawBuffer[i] << 8);
      convBuffer[i] = (float)(rawBuffer[i]) * 256.0;
    }
    // Serial.printf("%0.8f\r\n", convBuffer[0]);
#else
    for (int i = CHANNEL_NUMBER; i < 2 * AUDIO_INPUT_FRAME_SIZE; i += 2)
    {
      // Serial.printf("L: %d R: %d\r\n", rawBuffer[i], rawBuffer[i + 1]);
      convBuffer[i / 2] = (float)((rawBuffer[i] + rawBuffer[i + 1]) >> 8);
    }
#endif

    audioBuffer.push(convBuffer, AUDIO_INPUT_FRAME_SIZE);
    audioBuffer.get(frame, AUDIO_BUFFER_SIZE);

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

auto render_fps = registry.newFloatValue("fps", "status");

void setup()
{
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT);

  xTaskCreatePinnedToCore(
      render,
      "render",
      8192,
      NULL,
      6,
      &render_task,
      RENDER_TASK_CORE);

  xTaskCreatePinnedToCore(
      processAudioUpdate,
      "audioUpdate",
      8192 * 3,
      NULL,
      4,
      &audioUpdateTask,
      AUDIO_TASK_CORE);

  xTaskCreatePinnedToCore([](void *arg)
                          {
    for (;;)
    {
      auto mode = WiFi.getMode();
      if (mode != WIFI_MODE_NULL)
      {
        Serial.println(WiFi.localIP());
        WiFi.printDiag(Serial);
      }
      Serial.printf("FPS: %0.2f\r\n", render_fps->value());

      vTaskDelay(pdMS_TO_TICKS(8000));
    } },
                          "info_task", 8192, NULL, 8, NULL, RENDER_TASK_CORE);

  // BLEDevice::init("vuzic-esp32");

  // serve(NULL);
}

uint8_t clutBuffer[90 * 25 * 3];
Render3 *renderer;

// frame time in ms (ticks)
int targetFrameTime = 6;

#define NUM_TYPES NUM_BUCKETS

void iterate(void *arg)
{
  Universe *universe = (Universe *)arg;
  auto vars = Configlets::VariableGroup("render.particle-life", registry);

  auto types = universe->GetTypes();
  float base_attractors[NUM_TYPES][NUM_TYPES];
  for (int i = 0; i < NUM_TYPES; i++)
  {
    for (int j = 0; j < NUM_TYPES; j++)
    {
      base_attractors[i][j] = types->Attract(i, j);
    }
  }

  float audio_coef[NUM_BUCKETS][NUM_TYPES];
  construct_audio_coefficients<NUM_BUCKETS, NUM_TYPES>(audio_coef);

  const auto some_constant = vars.newFloatValue("audio_effect", 0.1, 0.0, 1.0, 0.05);

  for (;;)
  {
    auto idx = audioProcessor->fs->columnIdx;
    auto audio = audioProcessor->fs->drivers->amp[idx];
    auto scale = audioProcessor->fs->drivers->scales;
    for (int i = 0; i < NUM_BUCKETS; i++)
    {
      auto aval = scale[i] * (audio[i] - 1.0f);
      auto a = min(max(aval, -4.0f), 4.0f);
      for (int j = 0; j < NUM_TYPES; j++)
      {
        types->SetAttract(i, j, base_attractors[i][j] + some_constant->value() * a * audio_coef[i][j]);
      }
    }

    universe->Step();

    xTaskNotify(render_task, 1, eNoAction);
    vPortYield();
  }
}

void render(void *arg)
{
  auto vars = Configlets::VariableGroup("render.particle-life", registry);

  Serial.println("particle life render");
  auto seed = esp_random();
  Serial.printf("seed: %d\r\n", seed);
  {
    auto s = vars.newIntValue("seed");
    s->value() = seed;
  }

  DynamicJsonDocument doc(2048);
  JsonObject js = doc.to<JsonObject>();
  registry.dumpJson(js);
  serializeJsonPretty(doc, Serial);

  Universe universe(NUM_TYPES, 128, 64, 32, seed);
  universe.ReSeed(0.0, 0.04, 0.0, 4.0, 4.0, 16.0, 0.2, false);
  universe.ToggleWrap();

  bool reseed = false;
  size_t last_reseed = millis();
  attachInterruptArg(
      BUTTON_PIN, [](void *arg)
      { *(bool *)arg = !digitalRead(BUTTON_PIN); },
      &reseed, FALLING);

  auto types = universe.GetTypes();
  for (int i = 0; i < NUM_TYPES; i++)
  {
    double r, g, b;
    hsluv2rgb(360. * (double)i / (double)NUM_TYPES, 100., 50., &r, &g, &b);
    types->SetColor(i, ColorRGB(255 * (r * r), 255 * (g * g), 255 * (b * b)));
  }

  Serial.println("setup universe");

  auto display = setupDisplay();
  Serial.println("setup display");

  auto drawParticle = [](Particle &p, ColorRGB c)
  {
    auto idx = (uint16_t)p.x + (uint16_t)p.y * DISPLAY_WIDTH;
    displayBuffer[idx].r = c.r;
    displayBuffer[idx].g = c.g;
    displayBuffer[idx].b = c.b;
  };

  int frameCount = 0;
  TickType_t fpsTime = xTaskGetTickCount();

  while (!audioProcessor)
  {
    vTaskDelay(10);
  }

  xTaskCreatePinnedToCore(iterate, "iterate_task", 4096, (void *)&universe, 5, NULL, ITERATE_TASK_CORE);

  auto color_scale = vars.newFloatValue("color_scale", 1.0, 0.0, 2.0, 0.01);
  auto lightness_scale = vars.newFloatValue("lightness_scale", 0.1, 0.0, 2.0, 0.01);
  auto lightness_offset = vars.newFloatValue("lightness_offset", 0.5, 0.0, 1.0, 0.01);
  auto color_spread = vars.newFloatValue("color_spread", 120.0 / NUM_TYPES, 0.0, 60., 1.);
  auto fade_value = vars.newFloatValue("fade", 0.96, 0.0, 1.0, 0.01);

  for (;;)
  {
    if (reseed)
    {
      reseed = false;
      auto now = millis();
      if (now - last_reseed > 200)
      {
        last_reseed = now;
        randomSeed(esp_random());
        universe.ReSeed(0.0, 0.04, 0.0, 5.0, 5.0, 24.0, 0.2, false);
      }
    }

    // fade display
    uint16_t fade = 256 * fade_value->value();
    for (auto &c : displayBuffer)
    {
      c.r = ((uint16_t)c.r * fade) >> 8;
      c.g = ((uint16_t)c.g * fade) >> 8;
      c.b = ((uint16_t)c.b * fade) >> 8;
    }

    auto idx = audioProcessor->fs->columnIdx;
    auto audio = audioProcessor->fs->drivers->amp[idx];
    auto scale = audioProcessor->fs->drivers->scales;
    auto energy = audioProcessor->fs->drivers->energy;
    auto ls = lightness_scale->value();
    auto lo = lightness_offset->value();
    auto csp = color_spread->value();
    auto csc = color_scale->value();
    for (int i = 0; i < NUM_BUCKETS; i++)
    {
      auto aval = scale[i] * (audio[i] - 1.0f);
      auto cval = ls * sigmoid(aval) + lo;

      double r, g, b;
      float hue = fmod(180 * csc * energy[i] / PI + csp * i, 360);
      if (hue < 0)
        hue += 360;
      hsluv2rgb(hue, 100., 100. * cval, &r, &g, &b);
      types->SetColor(i, ColorRGB(255 * r * r, 255 * g * g, 255 * b * b));
    }

    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
    universe.IterParticles(drawParticle);

    for (int16_t j = 0; j < DISPLAY_HEIGHT; j++)
    {
      for (int16_t i = 0; i < DISPLAY_WIDTH; i++)
      {
        auto &c = displayBuffer[i + j * DISPLAY_WIDTH];
        display->drawPixelRGB888(i, j, c.r, c.g, c.b);
      }
    }

    TickType_t lastTime = xTaskGetTickCount();
    if (frameCount++ % 32 == 0)
    {
      auto e = 1000.0 / ((float)(lastTime - fpsTime) + 0.001);
      fpsTime = lastTime;
      render_fps->value() = .9 * render_fps->value() + .1 * 32.0 * e;
    }

    vPortYield();
  }
}
/*
void render_old(void *arg)
{
  Serial.println("start render");

  Render3_Params_t renderParams = {
      .warpScale = 128.,
      .warpOffset = .6,
      .warpP = 0.1,
      .warpD = 0.,
      .warpI = 0.01,
      .scaleScale = 1.33,
      .scaleOffset = .5,
      .scaleP = 4.,
      .scaleD = 0.,
      .scaleI = 0.2,
      .aspectX = 1,
      .aspectY = 1,
  };

  ColorParams_t colorParams = {
      .valueScale = 1,
      .valueOffset = 0,
      .saturationScale = .6,
      .saturationOffset = .1,
      .alphaScale = 0.75,
      .alphaOffset = 0.25,
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

  auto show = [=](Render3 *r)
  {
    // Serial.println("show display!");
    Color_RGB8 *buffer = renderer->getCurrentBuffer();
    auto h = DISPLAY_HEIGHT / 2;
    auto w = DISPLAY_WIDTH / 2;
    for (int y = 0; y < h; y++)
    {
      for (int x = 0; x < w; x++)
      {
        int bidx = x + y * w;
        Color_RGB8 c8 = buffer[bidx];
        // Color_RGB8 c8 = {
        //     (uint8_t)(255 * c.r),
        //     (uint8_t)(255 * c.g),
        //     (uint8_t)(255 * c.b),
        // };
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
    if (frameCount++ % 32 == 0)
    {
      auto e = 1000.0 / ((float)(lastTime - fpsTime) + 0.001);
      fpsTime = lastTime;
      render_fps->value() = .9 * render_fps->value() + .1 * 32.0 * e;
    }

    renderer->render(audioProcessor->fs->drivers);

    // vTaskDelayUntil(&lastTime, 2); // targetFrameTime);
  }
}
*/
void loop()
{
  delay(100);
}

void makeJsonResponse(JsonObject root)
{
  root["targetFrameRate"] = 1000 / targetFrameTime;
  root["actualFrameRate"] = render_fps->value();

  JsonArray size = root.createNestedArray("size");
  size.add(renderer->getRows());
  size.add(renderer->getColumns());

  auto cp = renderer->getColorParams();
  auto ps = renderer->getParams();
  JsonObject rp = root.createNestedObject("renderParams");
  rp["valueScale"] = cp->valueScale;
  rp["valueOffset"] = cp->valueOffset;
  rp["saturationScale"] = cp->saturationScale;
  rp["saturationOffset"] = cp->saturationOffset;
  rp["alphaScale"] = cp->alphaScale;
  rp["alphaOffset"] = cp->alphaOffset;
  rp["maxAlpha"] = cp->maxAlpha;
  rp["period"] = cp->period;
  rp["colorScale"] = cp->colorScale;
  rp["period"] = cp->period;
  rp["warpScale"] = ps->warpScale;
  rp["warpOffset"] = ps->warpOffset;
  rp["warpP"] = ps->warpP;
  rp["warpD"] = ps->warpD;
  rp["warpI"] = ps->warpI;
  rp["scaleScale"] = ps->scaleScale;
  rp["scaleOffset"] = ps->scaleOffset;
  rp["scaleP"] = ps->scaleP;
  rp["scaleD"] = ps->scaleD;
  rp["scaleI"] = ps->scaleI;
  rp["aspectX"] = ps->aspectX;
  rp["aspectY"] = ps->aspectY;

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
    if (renderParams.containsKey("warpP"))
    {
      ps->warpP = renderParams["warpP"];
    }
    if (renderParams.containsKey("warpD"))
    {
      ps->warpD = renderParams["warpD"];
    }
    if (renderParams.containsKey("warpI"))
    {
      ps->warpI = renderParams["warpI"];
    }
    hasKey = renderParams.containsKey("scaleScale");
    if (hasKey)
      ps->scaleScale = renderParams.getMember("scaleScale");
    hasKey = renderParams.containsKey("scaleOffset");
    if (hasKey)
      ps->scaleOffset = renderParams.getMember("scaleOffset");
    if (renderParams.containsKey("scaleP"))
    {
      ps->scaleP = renderParams["scaleP"];
    }
    if (renderParams.containsKey("scaleD"))
    {
      ps->scaleD = renderParams["scaleD"];
    }
    if (renderParams.containsKey("scaleI"))
    {
      ps->scaleI = renderParams["scaleI"];
    }
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
  perf["displayFps"] = render_fps->value();
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

#ifdef USE_ASYNC_TCP
void serve(void *arg)
{
  while (!renderer || !audioProcessor)
    vTaskDelay(10);

  // WebServer server(80, ssid, password, renderer, audioProcessor);

  Serial.println("Init WiFi...");

  WiFi.mode(WIFI_AP);
  WiFi.setAutoReconnect(true);
  WiFi.setAutoConnect(true);

  // WiFi.begin(ssid, password);
  WiFi.softAP(ssid, password);
  WiFi.softAPsetHostname("vuzic-esp32");

  vTaskDelay(8000);

  server = new AsyncWebServer(80);
  // WiFi.begin();
  // while (!WiFi.isConnected())
  // {
  //   Serial.printf("WiFi Failed: %d\n", WiFi.status());
  //   vTaskDelay(2000);
  // }

  // auto addr = WiFi.localIP();
  auto addr = WiFi.softAPIP();
  Serial.print("IP Address: ");
  Serial.println(addr);

  // AsyncServer server(80);
  // AsyncWebServer server(addr, 80);

  server->on("/", HTTP_GET, [](AsyncWebServerRequest *request)
             {
              Serial.println("got request to /");

              AsyncJsonResponse *response = new AsyncJsonResponse();
              response->addHeader("Server", "LED Panel Server");
              JsonObject root = response->getRoot();
              makeJsonResponse(root);
              response->setLength();

              request->send(response); });

  AsyncCallbackJsonWebHandler *handler = new AsyncCallbackJsonWebHandler("/",
                                                                         [](AsyncWebServerRequest *request, JsonVariant json)
                                                                         {
                                                                           JsonObject obj = json.as<JsonObject>();

                                                                           setParamsFromJson(obj);

                                                                           AsyncJsonResponse *response = new AsyncJsonResponse();
                                                                           response->addHeader("Server", "LED Panel Server");
                                                                           JsonObject root = response->getRoot();
                                                                           makeJsonResponse(root);
                                                                           response->setLength();

                                                                           request->send(response);
                                                                         });
  server->addHandler(handler);

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

  server->on("/debug", HTTP_GET, [](AsyncWebServerRequest *request)
             {
              Serial.println("got /debug");

              AsyncJsonResponse *response = new AsyncJsonResponse(false, 8196);
              response->addHeader("Server", "LED Panel Server");
              JsonObject root = response->getRoot();

              // bool getRawInput = request->getParam("rawInput");
              makeDebugResponse(root, false);

              response->setLength();
              request->send(response); });

  server->on("/rawinput", HTTP_GET, [](AsyncWebServerRequest *request)
             {
                Serial.println("get /rawinput");
                AsyncJsonResponse *response = new AsyncJsonResponse(false, 8196);
                JsonObject root = response->getRoot();
                makeDebugResponse(root, true);
                response->setLength();
                request->send(response); });

  server->onNotFound([](AsyncWebServerRequest *request)
                     {
                      Serial.println("got request to 404");
                      request->send(404, "text/plain", "Not found"); });

  Serial.println("begin server...");

  server->begin();
}
#endif

extern "C"
{
  void app_main(void)
  {
    initArduino();
    setup();
  }
}
