#include "Render.h"
#include <FrequencySensor.h>
#include "color.h"
#include "hsluv.h"
#include <stdlib.h>
#include "Arduino.h"
#include <functional>
#include <math.h>

#define WRITE_SEMI_TIMEOUT 6000
#define RENDER_TASK_TIMEOUT 1000
#define RENDER_SYNC_TIMEOUT 2000
#define WRITE_QUEUE_TIMEOUT 3000

#define MAX_ROWS 16
#define MAX_COLUMNS 64

template <typename T>
void clut(T *t, float hue, float sat, float val, T *r, T *g, T *b)
{
    int ti = (int)(val / 4) + 25 * (int)(hue / 4);
    ti *= 3;
    *r = t[ti];
    *g = t[ti + 1];
    *b = t[ti + 2];
}

// static Color_ABGRf get_hsv(ColorParams_t *params, float amp, float phase, float phi)
// {
//     float vs = params->valueScale;
//     float vo = params->valueOffset;
//     float ss = params->saturationScale;
//     float so = params->saturationOffset;
//     float as = params->alphaScale;
//     float ao = params->alphaOffset;
//     float cs = params->colorScale;

//     float hue = fmod(180 * (cs * phi + phase) / PI, 360);
//     if (hue < 0)
//         hue += 360;

//     // float sat = sigmoid(ss * amp + so);
//     float val = ss * sigmoid(vs * amp + vo) + so;
//     float alp = sigmoid(as * amp + ao);

//     float r, g, b;
//     if (params->clut == NULL)
//     {
//         // hsluv2rgb(hue, 100, 100 * (double)val, &r, &g, &b);
//     }
//     else
//     {
//         clut<float>(params->clut, hue, 100, 100 * val, &r, &g, &b);
//     }
//     // r *= r;
//     // g *= g;
//     // b *= b;

//     Color_ABGRf c = {alp, (float)b, (float)g, (float)r};
//     return c;
// }

static Color_ABGR get_hsv8(ColorParams_t *params, float amp, float phase, float phi)
{
    float vs = params->valueScale;
    float vo = params->valueOffset;
    float ss = params->saturationScale;
    float so = params->saturationOffset;
    float as = params->alphaScale;
    float ao = params->alphaOffset;
    float cs = params->colorScale;

    float hue = fmod(180 * (cs * phi + phase) / PI, 360);
    if (hue < 0)
        hue += 360;

    // float sat = sigmoid(ss * amp + so);
    float val = ss * sigmoid(vs * amp + vo) + so;
    float alp = sigmoid(as * amp + ao);

    uint8_t r = 0, g = 0, b = 0;
    if (params->clut == NULL)
    {
        // hsluv2rgb(hue, 100, 100 * (double)val, &r, &g, &b);
    }
    else
    {
        clut<uint8_t>(params->clut, hue, 100, 100 * val, &r, &g, &b);
    }
    // r *= r;
    // g *= g;
    // b *= b;

    Color_ABGR c = {(uint8_t)(alp * 255), b, g, r};
    return c;
}

// QueueHandle_t startDraw;// = xQueueCreate(4, sizeof(int));
// // r->startDraw = startDraw;
// QueueHandle_t startWrite;// = xQueueCreate(4, sizeof(int));
// // r->startWrite = startWrite;
// EventGroupHandle_t drawDisplayGroup;// = xEventGroupCreate();
// // r->drawDisplayGroup = drawGroup;
// SemaphoreHandle_t cb0;// = xSemaphoreCreateMutex();
// SemaphoreHandle_t cb1;// = xSemaphoreCreateMutex();
// SemaphoreHandle_t currentBufferLock[2] = {cb0, cb1};
// // r->currentBufferLock[0] = cb0;
// // r->currentBufferLock[1] = cb1;

Render3::Render3(
    int displayWidth,
    int displayHeight,
    int columns,
    int rows,
    Render3_Params_t *params,
    ColorParams_t *colorParams,
    std::function<void(Render3 *)> show)
    : displayWidth(displayWidth), displayHeight(displayHeight),
      columns(columns), rows(rows),
      params(params), colorParams(colorParams),
      show(show)
{
    fps = 0.0;
    renderCount = 0;

    queueLeftTime = 0;
    queueRightTime = 0;
    processLeftTime = 0;
    processRightTime = 0;
    lastRender = xTaskGetTickCount();

    if (rows > MAX_ROWS)
        rows = MAX_ROWS;
    if (columns > MAX_COLUMNS)
        columns = MAX_COLUMNS;
    this->rows = rows;
    this->columns = columns;

    GridPoint_t *points = new GridPoint_t[MAX_ROWS * MAX_COLUMNS];
    this->points = points;
    initGridPoints(rows, columns);
    warper = new WarpController(rows);
    scaler = new WarpController(columns);

    if (this->colorParams->clut != NULL)
    {
        for (int h = 0; h < 90; h++)
        {
            for (int v = 0; v < 25; v++)
            {
                double re, g, b;
                hsluv2rgb((double)h * 4, 100, (double)v * 4, &re, &g, &b);

                int ti = 3 * ((int)v + 25 * (int)h);
                this->colorParams->clut[ti] = re * re * 255;
                this->colorParams->clut[ti + 1] = g * g * 255;
                this->colorParams->clut[ti + 2] = b * b * 255;
            }
        }
    }

    int dsize = displayWidth * displayHeight / 4;
    buffer = (Color_RGB8 **)malloc(2 * sizeof(Color_RGB8 *));
    buffer[0] = (Color_RGB8 *)malloc(dsize * sizeof(Color_RGB8));
    buffer[1] = (Color_RGB8 *)malloc(dsize * sizeof(Color_RGB8));
    this->currentBuffer = 0;

    startDraw = xQueueCreate(4, sizeof(int));
    startWrite = xQueueCreate(4, sizeof(int));
    drawDisplayGroup = xEventGroupCreate();
    currentBufferLock[0] = xSemaphoreCreateBinary();
    xSemaphoreGive(currentBufferLock[0]);
    currentBufferLock[1] = xSemaphoreCreateBinary();
    xSemaphoreGive(currentBufferLock[1]);

    createRenderSubtask(0, "render_0");
    createRenderSubtask(1, "render_1");
    createWriteSubtask();
}

void Render3::createRenderSubtask(int taskNum, const char *name)
{
    xTaskCreatePinnedToCore(
        (taskNum == 0 ? renderSubtask0 : renderSubtask1),
        name,
        8000,
        (void *)this,
        5,
        renderSubtasks + taskNum,
        taskNum);
}

void Render3::renderSubtask0(void *arg)
{
    Render3 *r = (Render3 *)arg;
    r->renderSubtask(0);
}

void Render3::renderSubtask1(void *arg)
{
    Render3 *r = (Render3 *)arg;
    r->renderSubtask(1);
}

void Render3::createWriteSubtask()
{
    xTaskCreatePinnedToCore(
        writeTask,
        "renderWrite",
        16000,
        (void *)this,
        1,
        &writeTaskHandle,
        1);
}

void Render3::initGridPoints(int rows, int columns)
{
    GridPoint_t p;
    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < columns; x++)
        {
            float xf = (float)x / (float)columns * params->aspectX;
            float yf = (float)y / (float)rows * params->aspectY;
            p = {
                .x = xf,
                .y = yf,
                .srcX = x,
                .srcY = y,
            };
            points[x + y * columns] = p;
        }
    }
}

void Render3::setSize(int rows, int columns)
{
    if (rows > MAX_ROWS)
        rows = MAX_ROWS;
    if (columns > MAX_COLUMNS)
        columns = MAX_COLUMNS;

    initGridPoints(rows, columns);
    this->rows = rows;
    this->columns = columns;
    // fixme.. but this method shouldn't really be needed, or if so, the whole thing should be reinit
    delete warper;
    delete scaler;
    warper = new WarpController(rows);
    scaler = new WarpController(columns);
}

inline void Render3::getDisplayXY(GridPoint_t *g, int w, int h, int *x, int *y)
{
    int xv = (int)(g->x * w + 0.5);
    int yv = (int)(g->y * h + 0.5);
    if (xv < 0)
        xv = 0;
    if (xv >= w)
        xv = w - 1;
    if (yv < 0)
        yv = 0;
    if (yv >= h)
        yv = h - 1;
    *x = xv;
    *y = yv;
}

// https://github.com/ekmett/approximate/blob/master/cbits/fast.c
float powf_fast(float a, float b)
{
    union
    {
        float d;
        int x;
    } u = {a};
    u.x = (int)(b * (u.x - 1064866805) + 1064866805);
    return u.d;
}

inline GridPoint_t Render3::applyWarp(GridPoint_t *g, float w, float s)
{
    GridPoint_t p;

    if (g->x <= 0)
        p.x = powf_fast(g->x + 1., w) - 1.;
    else
        p.x = 1. - powf_fast(1. - g->x, w);

    if (g->y <= 0)
    {
        s = (1. + g->y / 2) * s;
        p.y = powf_fast(g->y + 1., s) - 1.;
    }
    else
    {
        s = (1. - g->y / 2) * s;
        p.y = 1. - powf_fast(1 - g->y, s);
    }
    return p;
}

void Render3::render(FS_Drivers_t *drivers)
{
    // vTaskDelayUntil(&(r->lastRender), 10);

    // Serial.println("star render3");

    renderCount++;
    long now = micros();
    float fps = 1000000. / ((float)now - (float)lastRender);
    this->fps = .01 * fps + .99 * this->fps;
    lastRender = now;

    // Serial.println("line 283");

    // todo: optimize this by calculating it in freq sensor once for new frames and then decay
    // for (int i = 0; i < r->columns; i++) {
    //     float s = 0;
    //     for (int j = 0; j < r->rows; j++) {
    //         float *amps = FS_GetColumn(drivers, i);
    //         s += (amps[j]) * drivers->scales[j];
    //     }
    //     s /= r->rows;
    //     r->scales[i] = s;
    // }

    int dsize = displayWidth * displayHeight / 4;
    int currentBuffer = this->currentBuffer ^ 1;

    // Serial.println("gonna take the semmi");
    // Serial.println(currentBuffer);
    if (xSemaphoreTake(currentBufferLock[currentBuffer], WRITE_SEMI_TIMEOUT) == pdFALSE)
    {
        Serial.println("timeout waiting for current buffer sem");
        return;
    }

    Color_RGB8 *buffer = this->buffer[currentBuffer];
    this->currentBuffer = currentBuffer;
    memset(buffer, 0, sizeof(Color_RGB8) * dsize);

    float warps[rows];
    for (int i = 0; i < rows; i++)
    {
        warps[i] = drivers->diff[i]; // * params->diffGain;
    }
    warper->computeWarp(warps, params->warpP, params->warpD, params->warpI);
    float scales[columns];
    for (int i = 0; i < columns; i++)
    {
        int column = drivers->columnIdx - i;
        if (column < 0)
            column += drivers->length;
        scales[i] = drivers->ampAverage[column];
    }
    scaler->computeWarp(scales, params->scaleP, params->scaleD, params->scaleI);

    initTime = micros() - now;

    // Serial.println("about to clear group bits");
    xEventGroupClearBits(drawDisplayGroup, 7);

    xQueueSend(startDraw, &drivers, 1);
    xQueueSend(startDraw, &drivers, 1);

    // Serial.println(drawDisplayGroup);
    // Serial.println(currentBuffer);
    EventBits_t sync = xEventGroupSync(drawDisplayGroup, 1, 7, RENDER_SYNC_TIMEOUT);
    if ((sync & 7) != 7)
    {
        xSemaphoreGive(currentBufferLock[currentBuffer]);
        Serial.println("failed to sync subrenders");
        return;
    }

    drawTime = micros() - now - initTime;

    if (xQueueSend(startWrite, &currentBuffer, RENDER_TASK_TIMEOUT) != pdTRUE)
    {
        xSemaphoreGive(currentBufferLock[currentBuffer]);
        Serial.println("failed to send write semi");
    }
}

void Render3::renderInner(int start, int end, FS_Drivers_t *drivers)
{
    float warpScale = params->warpScale;
    float warpOffset = params->warpOffset;
    float scaleScale = params->scaleScale;
    float scaleOffset = params->scaleOffset;

    Color_RGB8 *buffer = getCurrentBuffer();

    float *amps[drivers->length];
    for (int i = 0; i < drivers->length; i++)
    {
        int column = drivers->columnIdx - i;
        if (column < 0)
            column += drivers->length;

        amps[i] = drivers->amp[column];
    }

    long nowl;
    for (int i = start; i < end; i++)
    {
        GridPoint_t g = points[i];

        nowl = micros();

        int wi = g.srcY; // rows - 1 - g.srcY;
        float warp = warpScale * warper->getWarp(wi) + warpOffset;
        // if (i == 0)
        // {
        //     Serial.println(warper->getWarp(wi));
        // }
        int si = g.srcX; // columns - 1 - g.srcX;
        float scale = scaleScale * scaler->getWarp(si) + scaleOffset;
        GridPoint_t p = applyWarp(&g, warp, scale);
        int x, y;
        getDisplayXY(&p, displayWidth / 2, displayHeight / 2, &x, &y);
        int bufIdx = x + displayWidth / 2 * y;

        warpTime = (micros() - nowl + warpTime) / 2;
        // Serial.println("render ln 270");

        float amp = drivers->scales[wi] * (amps[si][wi] - 1);
        float phase = drivers->energy[wi];
        float phi = 2.0 * PI / colorParams->period * si;

        if (isnan(amp) || isinf(amp) || isnan(phase) || isinf(phase))
        {
            // Serial.println("BAD!");
            // Serial.println(amp);
            // Serial.println(phase);
            // Serial.println(si);
            // Serial.println(wi);
            continue;
        }

        Color_ABGR c1 = get_hsv8(colorParams, amp, phase, phi);
        Color_RGB8 c2 = buffer[bufIdx];

        // Serial.print("rendeer ln 277 ");
        // Serial.println(i);

        // float wc1 = /*c1.a*/ +c1.b + c1.g + c1.r;
        // float wc2 = /*c2.a*/ +c2.b + c2.g + c2.r;
        // float sw = 1.0 / (wc1 + wc2);
        // float sc1 = wc1 * sw;
        // float sc2 = wc2 * sw;

        // //c2.a = c1.a * sc1 + c2.a * sc2;
        // c2.b = c1.b * sc1 + c2.b * sc2;
        // c2.g = c1.g * sc1 + c2.g * sc2;
        // c2.r = c1.r * sc1 + c2.r * sc2;

        int sc1 = c1.a;
        int sc2 = (255 - c1.a);
        c2.b = ((int)c1.b * sc1 + (int)c2.b * sc2) >> 8;
        c2.g = ((int)c1.g * sc1 + (int)c2.g * sc2) >> 8;
        c2.r = ((int)c1.r * sc1 + (int)c2.r * sc2) >> 8;

        // if (c2.a > 1) c2.a = 1;
        // if (c2.b > 255)
        //     c2.b = 255;
        // if (c2.g > 255)
        //     c2.g = 255;
        // if (c2.r > 255)
        //     c2.r = 255;

        // TODO: put pixel in a '+' shape, but requires more blending
        buffer[bufIdx] = c2;

        colorTime = (micros() - nowl - warpTime + colorTime) / 2;
    }
}

void Render3::renderSubtask(int taskNum)
{
    FS_Drivers_t *drivers;
    long now = micros();

    for (;;)
    {

        now = micros();

        if (xQueueReceive(startDraw, &drivers, RENDER_TASK_TIMEOUT) == pdFALSE)
        {
            Serial.printf("timeout waiting in subrender %d\n", taskNum);
        }
        else
        {

            if (taskNum == 0)
            {
                queueLeftTime = (micros() - now + queueLeftTime) / 2;
            }
            else
            {
                queueRightTime = (micros() - now + queueRightTime) / 2;
            }

            int rsize = rows * columns;
            int split = rsize / 2;

            renderInner((taskNum == 0 ? 0 : split), (taskNum == 0 ? split : rsize), drivers);

            if (taskNum == 0)
            {
                processLeftTime = (micros() - now - queueLeftTime + processLeftTime) / 2;
            }
            else
            {
                processRightTime = (micros() - now - queueRightTime + processRightTime) / 2;
            }
        }

        xEventGroupSync(drawDisplayGroup, 1 << (taskNum + 1), 7, RENDER_SYNC_TIMEOUT);
    }
}

void Render3::writeTask(void *arg)
{
    Render3 *r = (Render3 *)arg;
    r->write();
}

void Render3::write()
{
    int currentBuffer;
    long now;

    for (;;)
    {

        if (xQueueReceive(startWrite, &currentBuffer, WRITE_QUEUE_TIMEOUT) == pdFALSE)
        {
            xSemaphoreGive(currentBufferLock[0]);
            xSemaphoreGive(currentBufferLock[1]);
            Serial.println("timeout waiting to write");
            return;
        }

        now = micros();

        show(this);

        writeTime = micros() - now;

        xSemaphoreGive(currentBufferLock[currentBuffer]);

        vTaskDelay(1);
    }
}

Color_RGB8 *Render3::getCurrentBuffer()
{
    return buffer[currentBuffer];
}
