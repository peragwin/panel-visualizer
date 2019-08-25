#include "Render.h"
#include <FrequencySensor.h>
#include "color.h"
#include "hsluv.h"
#include <stdlib.h>
#include "Arduino.h"
#include <math.h>

RenderMode2_t* NewRender2(Render2Params_t *params, ColorParams_t *colorParams, int size, int length,
    void (*setPixel) (int x, int y, Color_ABGR c)) {

    RenderMode2_t *r = (RenderMode2_t*)malloc(sizeof(RenderMode2_t));
    r->params = params;
    r->colorParams = colorParams;
    r->size = size;
    r->length = length;
    r->_setPixel = setPixel;

    return r;
}

static float sigmoid(float x) {
    float a = x;
    if (x < 0) a = -a;
    return (1.0 + x / (1.0 + a)) / 2.0;
}

void clut(float* t, double hue, double sat, double val, double *r, double *g, double *b) {
    int ti = (int)(val/4) + 25 * (int)(hue/4);
    ti *= 3;
    *r = (double)t[ti];
    *g = (double)t[ti+1];
    *b = (double)t[ti+2];
}

static Color_ABGRf get_hsv(ColorParams_t *params, float amp, float phase, float phi) {
    float vs = params->valueScale;
    float vo = params->valueOffset;
    float ss = params->saturationScale;
    float so = params->saturationOffset;
    float as = params->alphaScale;
    float ao = params->alphaOffset;
    float cs = params->colorScale;

    float hue = fmod(180 * (cs*phi + phase) / PI, 360);
    if (hue < 0) hue += 360;

    // float sat = sigmoid(ss * amp + so);
    float val = ss * sigmoid(vs * amp + vo) + so;
    float alp = 0;//sigmoid(as * amp + ao);

    double r, g, b;
    if (params->clut == NULL) {
        hsluv2rgb((double)hue, 100, 100*(double)val, &r, &g, &b);
    } else {
        clut(params->clut, hue, 100, 100*val, &r, &g, &b);
    }
    r *= r;
    g *= g;
    b *= b;
    
    Color_ABGRf c = {alp * params->maxAlpha, (float)b, (float)g, (float)r};
    return c;
}

QueueHandle_t startDraw;// = xQueueCreate(4, sizeof(int));
// r->startDraw = startDraw;
QueueHandle_t startWrite;// = xQueueCreate(4, sizeof(int));
// r->startWrite = startWrite;
EventGroupHandle_t drawDisplayGroup;// = xEventGroupCreate();
// r->drawDisplayGroup = drawGroup;
SemaphoreHandle_t cb0;// = xSemaphoreCreateMutex();
SemaphoreHandle_t cb1;// = xSemaphoreCreateMutex();
SemaphoreHandle_t currentBufferLock[2] = {cb0, cb1};
// r->currentBufferLock[0] = cb0;
// r->currentBufferLock[1] = cb1;

RenderMode3_t *NewRender3(
    int displayWidth,
    int displayHeight,
    int rows,
    int columns,
    Render3_Params_t *params,
    ColorParams_t *colorParams,
    void (*setPixel) (int x, int y, Color_ABGR c),
    void (*show) (int currentBuffer)
) {
    RenderMode3_t *r = (RenderMode3_t *)malloc(sizeof(RenderMode3_t));
    r->displayWidth = displayWidth;
    r->displayHeight = displayHeight;
    r->rows = rows;
    r->columns = columns;
    r->params = params;
    r->colorParams = colorParams;
    r->show = show;
    r->setPixel = setPixel;
    r->renderCount = 0;

    r->queueLeftTime = 0;
    r->queueRightTime = 0;
    r->processLeftTime = 0;
    r->processRightTime = 0;
    r->lastRender = xTaskGetTickCount();

    int psize = rows * columns;

    float *scales = (float*)malloc(psize * sizeof(float));
    r->scales = scales;

    Render3_GridPoint_t *points = (Render3_GridPoint_t*)malloc(psize * sizeof(Render3_GridPoint_t));
    for (int x = 0; x < columns; x++) {
        for (int y = 0; y < rows; y++) {
            float xf = (float)x / (float)displayWidth;
            float yf = (float)y / (float)displayHeight * params->aspect;
            Render3_GridPoint_t p = {
                .x = xf,
                .y = yf,
                .srcX = x,
                .srcY = y,
            };
            points[x+y*columns] = p;
        }
    }
    r->points = points;

    if (r->colorParams->clut != NULL) {
        for (int h = 0; h < 90; h++) {
            for (int v = 0; v < 25; v++) {
                double re, g, b;
                hsluv2rgb((double)h*4, 100, (double)v*4, &re, &g, &b);

                int ti = (int)v + 25 * (int)h;
                ti *= 3;
                r->colorParams->clut[ti] = (float)re;
                r->colorParams->clut[ti+1] = (float)g;
                r->colorParams->clut[ti+2] = (float)b;
            }
        }
    }

    int dsize = displayWidth * displayHeight / 4;
    r->buffer = (Color_ABGRf**)malloc(2*sizeof(Color_ABGRf*));
    r->buffer[0] = (Color_ABGRf*)malloc(dsize*sizeof(Color_ABGRf));
    r->buffer[1] = (Color_ABGRf*)malloc(dsize*sizeof(Color_ABGRf));
    r->currentBuffer = 0;

     startDraw = xQueueCreate(4, sizeof(int));
    //r->startDraw = startDraw;
    startWrite = xQueueCreate(4, sizeof(int));
    //r->startWrite = startWrite;
    drawDisplayGroup = xEventGroupCreate();
    //r->drawDisplayGroup = drawGroup;
    currentBufferLock[0] = xSemaphoreCreateBinary();
    xSemaphoreGive(currentBufferLock[0]);
    currentBufferLock[1] = xSemaphoreCreateBinary();
    xSemaphoreGive(currentBufferLock[1]);
    //r->currentBufferLock[0] = cb0;
    //r->currentBufferLock[1] = cb1;

    return r;
}

void Render3_GetDisplayXY(Render3_GridPoint_t *g, int w, int h, int *x, int *y) {
    int xv = (int)(g->x * w + 0.5);
    int yv = (int)(g->y * h + 0.5);
    if (xv < 0) xv = 0;
    if (xv >= w) xv = w - 1;
    if (yv < 0) yv = 0;
    if (yv >= h) yv = h - 1;
    *x = xv;
    *y = yv;
}

// https://github.com/ekmett/approximate/blob/master/cbits/fast.c
float powf_fast(float a, float b) {
  union { float d; int x; } u = { a };
  u.x = (int)(b * (u.x - 1064866805) + 1064866805);
  return u.d;
}

Render3_GridPoint_t Render3_ApplyWarp(Render3_GridPoint_t *g, float w, float s) {
    Render3_GridPoint_t p;

    if (g->x <= 0) p.x = powf_fast(g->x + 1., w) - 1.;
    else p.x = 1. - powf_fast(1. - g->x, w);

    if (g->y <= 0) {
        s = (1. + g->y / 2) * s;
        p.y = powf_fast(g->y + 1., s) - 1.;
    } else {
        s = (1. - g->y / 2) * s;
        p.y = 1. - powf_fast(1 - g->y, s);
    }
    return p;
}

void Render3(RenderMode3_t *r, FS_Drivers_t *drivers) {
    // vTaskDelayUntil(&(r->lastRender), 10);

    // Serial.println("star render3");

    r->renderCount++;
    long now = micros();
    r->fps = 1000000. / ((float)now - (float)r->lastRender);
    r->lastRender = now;

    // Serial.println("line 283");

    // todo: optimize this by calculating it in freq sensor once for new frames and then decay
    for (int i = 0; i < r->columns; i++) {
        float s = 0;
        for (int j = 0; j < r->rows; j++) {
            float *amps = FS_GetColumn(drivers, i);
            s += (amps[j]) * drivers->scales[j];
        }
        s /= r->rows;
        r->scales[i] = s;
    }

    int dsize = r->displayWidth * r->displayHeight / 4;
    int currentBuffer = r->currentBuffer^1;

// Serial.println("gonna take the semmi");
    if ( xSemaphoreTake(currentBufferLock[currentBuffer], 100) == pdFALSE ) {
        Serial.println("timeout waiting for current buffer sem");
        return;
    }

    Color_ABGRf *buffer = r->buffer[currentBuffer];
    r->currentBuffer = currentBuffer;

    for (int i = 0; i < dsize; i++) {
        Color_ABGRf c = {0,0,0,0};
        buffer[i] = c;
    }

    r->initTime = micros() - now;

// Serial.println("about to clear group bits");
    xEventGroupClearBits(drawDisplayGroup, 7);

    xQueueSend(startDraw, &currentBuffer, 1);
    xQueueSend(startDraw, &currentBuffer, 1);

    EventBits_t sync = xEventGroupSync(drawDisplayGroup, 1, 7, 100);
    if ((sync & 7) != 7) {
        xSemaphoreGive(currentBufferLock[currentBuffer]);
        Serial.println("failed to sync subrenders");
        return;
    }

    r->drawTime = micros() - now - r->initTime;

    if ( xQueueSend(startWrite, &currentBuffer, 100) != pdTRUE ) {
        xSemaphoreGive(currentBufferLock[currentBuffer]);
        Serial.println("failed to send write semi");
    }
}

void Render3Inner(RenderMode3_t *r, int start, int end, int currentBuffer, FS_Drivers_t *drivers) {
    float warpScale =  r->params->warpScale;
    float warpOffset = r->params->warpOffset;
    float scaleScale = r->params->scaleScale;
    float scaleOffset = r->params->scaleOffset;

    Color_ABGRf *buffer = r->buffer[currentBuffer];

    long nowl;
    for (int i = start; i < end; i++) {
        Render3_GridPoint_t g = r->points[i];

        nowl = micros();

        int wi = g.srcY; // r->rows - 1 - g.srcY;
        float warp = warpScale * drivers->diff[wi] + warpOffset;
        int si = g.srcX; // r->columns - 1 - g.srcX;
        float scale = scaleScale * r->scales[si] + scaleOffset;
        Render3_GridPoint_t p = Render3_ApplyWarp(&g, warp, scale);
        int x, y;
        Render3_GetDisplayXY(&p, r->displayWidth/2, r->displayHeight/2, &x, &y);
        int bufIdx = x + r->displayWidth/2 * y;

        r->warpTime = (micros() - nowl + r->warpTime) / 2;
        // Serial.println("render ln 270");

        float *amps = FS_GetColumn(drivers, si);
        float amp = drivers->scales[wi] * (amps[wi] - 1);
        float phase = drivers->energy[wi];
        float phi = 2.0 * PI / r->colorParams->period * si;

        if (isnan(amp) || isinf(amp) || isnan(phase) || isinf(phase)) {
            // Serial.println("BAD!");
            // Serial.println(amp);
            // Serial.println(phase);
            // Serial.println(si);
            // Serial.println(wi);
            continue;
        }

        Color_ABGRf c1 = get_hsv(r->colorParams, amp, phase, phi);
        Color_ABGRf c2 = buffer[bufIdx];

        // Serial.print("rendeer ln 277 ");
        // Serial.println(i);

        float wc1 = c1.a + c1.b + c1.g + c1.r;
        wc1 /= 4;
        float wc2 = c2.a + c2.b + c2.g + c2.r;
        wc2 /= 4;
        float sw = 1 / (wc1 + wc2);
        float sc1 = wc1 * sw;
        float sc2 = wc2 * sw;

        c2.a = c1.a * sc1 + c2.a * sc2;
        c2.b = c1.b * sc1 + c2.b * sc2;
        c2.g = c1.g * sc1 + c2.g * sc2;
        c2.r = c1.r * sc1 + c2.r * sc2;

        if (c2.a > 1) c2.a = 1;
        if (c2.b > 1) c2.b = 1;
        if (c2.g > 1) c2.g = 1;
        if (c2.r > 1) c2.r = 1;

        buffer[bufIdx] = c2;

        r->colorTime = (micros() - nowl - r->warpTime + r->colorTime) / 2;
    }
}

void Render3Left(RenderMode3_t *r, FS_Drivers_t *drivers) {
    long now = micros();

    int currentBuffer;
    if ( xQueueReceive(startDraw, &currentBuffer, 100) == pdFALSE ) {
        //xEventGroupSync(r->drawDisplayGroup, 2, 0, 100);
        xEventGroupSetBits(drawDisplayGroup, 7);
        Serial.println("timeout waiting for subrender left");
        return;
    }

    r->queueLeftTime = (micros() - now + r->queueLeftTime) / 2;

    int rsize = r->rows * r->columns;
    int end = rsize / 2;
    Render3Inner(r, 0, end, currentBuffer, drivers);

    r->processLeftTime = (micros() - now - r->queueLeftTime + r->processLeftTime) / 2;

    xEventGroupSync(drawDisplayGroup, 2, 7, 100);
}

void Render3Right(RenderMode3_t *r, FS_Drivers_t *drivers) {
    long now = micros();

    int currentBuffer;
    if ( xQueueReceive(startDraw, &currentBuffer, 100) == pdFALSE ) {
        // xEventGroupSync(r->drawDisplayGroup, 4, 0, 100);
        xEventGroupSetBits(drawDisplayGroup, 7);
        Serial.println("timeout waiting for subrender right");
        return;
    }

    r->queueRightTime = (micros() - now + r->queueRightTime) / 2;

    int rsize = r->rows * r->columns;
    int start = rsize / 2;
    Render3Inner(r, start, rsize, currentBuffer, drivers);

    r->processRightTime = (micros() - now - r->queueRightTime + r->processRightTime) / 2;

    xEventGroupSync(drawDisplayGroup, 4, 7, 100);
}

void Render3Write(RenderMode3_t *r) {
    int currentBuffer;

    if ( xQueueReceive(startWrite, &currentBuffer, 100) == pdFALSE ) {
        xSemaphoreGive(currentBufferLock[0]);
        xSemaphoreGive(currentBufferLock[1]);
        Serial.println("timeout waiting to write");
        return;
    }

    long now = micros();

    r->show(currentBuffer);

    r->writeTime = micros() - now;

    xSemaphoreGive(currentBufferLock[currentBuffer]);
}