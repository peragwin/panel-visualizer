#pragma once

#include <FrequencySensor.h>
#include "color.h"

typedef struct {
    float valueScale;
    float valueOffset;
    float saturationScale;
    float saturationOffset;
    float alphaScale;
    float alphaOffset;
    float maxAlpha;
    float period;
    float colorScale;
    ColorGamut_t gamut;
    float *clut;
} ColorParams_t;

typedef struct {
    float pHeight;
    float pHScale;
    float pHOffset;
    float pVHOffset;
    float pWidth;
    float pWScale;
    float pWOffset;
    float pVWOffset;
} Render2Params_t;

// RenderMode2 displays a single row of LEDs
typedef struct {
    int size;
    int length;
    ColorParams_t *colorParams;
    Render2Params_t *params;
    void (*_setPixel) (int x, int y, Color_ABGR c);
} RenderMode2_t;

RenderMode2_t* NewRender2(Render2Params_t *params, ColorParams_t *colorParams, int size, int length,
    void (*setPixel) (int x, int y, Color_ABGR c));
void Render2(RenderMode2_t *r, FS_Drivers_t *drivers);


typedef struct {
    float warpScale;
    float warpOffset;
    float scaleScale;
    float scaleOffset;
    float aspect;
} Render3_Params_t;

typedef struct {
    float x;
    float y;
    int srcX;
    int srcY;
} Render3_GridPoint_t;

typedef struct {
    Render3_GridPoint_t *points;
    int columns;
    int rows;
    int displayWidth;
    int displayHeight;
    Render3_Params_t *params;
    ColorParams_t *colorParams;
    
    int renderCount;
    long lastRender;
    float fps;
    long initTime;
    long warpTime;
    long colorTime;
    long drawTime;
    long writeTime;

    Color_ABGRf **buffer;
    int currentBuffer;
    QueueHandle_t bufferReady;

    void (*setPixel) (int x, int y, Color_ABGR c);
    void (*show) (void);
} RenderMode3_t;

RenderMode3_t *NewRender3(
    int displayWidth,
    int displayHeight,
    int rows,
    int columns,
    Render3_Params_t *params,
    ColorParams_t *colorParams,
    void (*setPixel) (int x, int y, Color_ABGR c),
    void (*show) (void)
);

void Render3(RenderMode3_t *r, FS_Drivers_t *drivers);
void Render3Write(RenderMode3_t *r);

// #endif