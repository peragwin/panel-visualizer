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
    float alp = sigmoid(as * amp + ao);

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

void Render2(RenderMode2_t *r, FS_Drivers_t *drivers) {
    int width = r->size / r->length;
    float spacing = (float)width / (float)(drivers->size + 1);

    Color_ABGRf frame[r->size];
    for (int i = 0; i < r->size; i++) {
        Color_ABGRf c = {0,0,0,0};
        frame[i] = c;
    }
    float *amp = FS_GetColumn(drivers, 0);
    float *phase = drivers->energy;
    float *scales = drivers->scales;

    float ph = r->params->pHeight;
    float phs = r->params->pHScale;
    float pho = r->params->pHOffset;
    float pvho = r->params->pVHOffset;

    float pw = r->params->pWidth;
    float pws = r->params->pWScale;
    float pwo = r->params->pWOffset;
    float pvwo = r->params->pVWOffset;

    for (int i = 0; i < drivers->size; i++) {
        float center = (i+1) * spacing;
        int y = i % r->length;
        int yoffset = y * width;

        float val = amp[i] * scales[i];
        float phi = 2*PI / r->colorParams->period * center;
        Color_ABGRf color = get_hsv(r->colorParams, val, phase[i], phi);

        float pheight = ph * sigmoid(phs * val + pvho) + pho;
        float pwidth = pw * sigmoid(pws * val + pvwo) + pwo;

        int start = (int)ceilf(center - pwidth);
        if (start < 0) start = 0;
        int end = (int)floorf(center + pwidth);
        if (end >= width) end = width - 1;

        for (int x = start; x < end; x++) {
            float xc = (x - center) / pwidth;
            float p = pheight * (1 - xc*xc);

            int idx = x + yoffset;
            frame[idx].a += p * color.a;
            frame[idx].b += p * color.b;
            frame[idx].g += p * color.g;
            frame[idx].r += p * color.r;
        }
    }

    

    for (int y = 0; y < r->length; y++) {
        int yoffset = y * width;
        for (int x = 0; x < width; x++) {
            int i = x + yoffset;
            int a = (int)(255 * frame[i].a);
            int b = (int)(255 * frame[i].b);
            int g = (int)(255 * frame[i].g);
            int d = (int)(255 * frame[i].r);
            if (a < 0) a = 0;
            if (a > 255) a = 255;
            if (b < 0) b = 0;
            if (b > 255) b = 255;
            if (g < 0) g = 0;
            if (g > 255) g = 255;
            if (d < 0) d = 0;
            if (d > 255) d = 255;

            Color_ABGR c = {a, b, g, d};

            r->_setPixel(x, y, c);
        }
    }
}

RenderMode3_t *NewRender3(
    int displayWidth,
    int displayHeight,
    int rows,
    int columns,
    Render3_Params_t *params,
    ColorParams_t *colorParams,
    void (*setPixel) (int x, int y, Color_ABGR c),
    void (*show) (void)
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

    int psize = rows * columns;
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

    QueueHandle_t bufferReady = xQueueCreate(4, sizeof(int));
    r->bufferReady = bufferReady;

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
    r->renderCount++;
    long now = micros();
    r->fps = 1000000. / ((float)now - (float)r->lastRender);
    r->lastRender = now;

    // todo: optimize this by calculating it in freq sensor once for new frames and then decay
    float scales[r->columns];
    for (int i = 0; i < r->columns; i++) {
        float s = 0;
        for (int j = 0; j < r->rows; j++) {
            float *amps = FS_GetColumn(drivers, i);
            s += (amps[j]) * drivers->scales[j];
        }
        s /= r->rows;
        scales[i] = s;
    }

    int dsize = r->displayWidth * r->displayHeight / 4;
    Color_ABGRf *buffer = r->buffer[r->currentBuffer^=1];
    for (int i = 0; i < dsize; i++) {
        Color_ABGRf c = {0,0,0,0};
        buffer[i] = c;
    }

    float warpScale =  r->params->warpScale;
    float warpOffset = r->params->warpOffset;
    float scaleScale = r->params->scaleScale;
    float scaleOffset = r->params->scaleOffset;

    int rsize = r->rows * r->columns;

    // Serial.println("render ln 256");

    r->initTime = micros() - now;

    long nowl;
    for (int i = 0; i < rsize; i++) {
        Render3_GridPoint_t g = r->points[i];

        nowl = micros();

        int wi = g.srcY; // r->rows - 1 - g.srcY;
        float warp = warpScale * drivers->diff[wi] + warpOffset;
        int si = g.srcX; // r->columns - 1 - g.srcX;
        float scale = scaleScale * scales[si] + scaleOffset;
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

    r->drawTime = micros() - now - r->initTime;
    xQueueSend(r->bufferReady, &r->currentBuffer, 1);

    // Serial.println("render ln 298");
}

void Render3Write(RenderMode3_t *r) {
    int currentBuffer;
    bool rx = xQueueReceive(r->bufferReady, &currentBuffer, 100);
    if (!rx) return;

    long now = micros();

    Color_ABGRf *buffer = r->buffer[currentBuffer];

    int yo = r->displayHeight/2;
    int xo = r->displayWidth/2;
    for (int x = 0; x < r->displayWidth/2; x++) {
        for (int y = 0; y < r->displayHeight/2; y++) {
            int bidx = x + y * r->displayWidth/2;
            Color_ABGRf cf = buffer[bidx];
            Color_ABGR c;

            c.a = (char)(255 * cf.a);
            c.b = (char)(255 * cf.b);
            c.g = (char)(255 * cf.g);
            c.r = (char)(255 * cf.r);

            // Serial.print("pixel ");
            // Serial.print(x);
            // Serial.print(", ");
            // Serial.print(x);
            // Serial.print(", ");
            // Serial.print(cf.r);
            // Serial.print(", ");
            // Serial.print(cf.g);
            // Serial.print(", ");
            // Serial.println(cf.b);

            r->setPixel(xo+x, yo+y, c);
            r->setPixel(xo-1-x, yo+y, c);
            r->setPixel(xo+x, yo-1-y, c);
            r->setPixel(xo-1-x, yo-1-y, c);
        }
    }

    r->writeTime = micros() - now;

    // Serial.println("render ln 318");

    r->show();
}