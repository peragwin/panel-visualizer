#pragma once

#include <FrequencySensor.h>
#include "color.h"
#include <FreeRTOS.h>

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
    float aspectX;
    float aspectY;
} Render3_Params_t;

typedef struct {
    float x;
    float y;
    int srcX;
    int srcY;
} GridPoint_t;

class Render3 {
  private:
    int columns;
    int rows;
    int displayWidth;
    int displayHeight;

    int renderCount;
    TickType_t lastRender;

    GridPoint_t *points;

    Color_RGB **buffer;
    int currentBuffer;

    TaskHandle_t renderSubtasks[2];
    TaskHandle_t writeTaskHandle;

    void (*setPixel) (int x, int y, Color_ABGR c);
    void (*show) (Render3 *r);

    void createRenderSubtask(int taskNum, const char* name);
    static void renderSubtask0(void *arg);
    static void renderSubtask1(void *arg);
    void renderSubtask(int taskNum);
    void renderInner(int start, int end, FS_Drivers_t *drivers);
    static void writeTask(void *arg);
    void createWriteSubtask();
    void write();
    void initGridPoints(int rows, int columns);

    static GridPoint_t applyWarp(GridPoint_t *g, float w, float s);
    static void getDisplayXY(GridPoint_t *g, int w, int h, int *x, int *y);


  public:
    Render3_Params_t *params;
    ColorParams_t *colorParams;

    float fps;
    long initTime;
    long warpTime;
    long colorTime;
    long drawTime;
    long writeTime;
    long queueLeftTime;
    long queueRightTime;
    long processLeftTime;
    long processRightTime;

    QueueHandle_t startDraw;
    QueueHandle_t startWrite;
    EventGroupHandle_t drawDisplayGroup;
    SemaphoreHandle_t currentBufferLock[2];

    Render3(
        int displayWidth,
        int displayHeight,
        int rows,
        int columns,
        Render3_Params_t *params,
        ColorParams_t *colorParams,
        void (*setPixel) (int x, int y, Color_ABGR c),
        void (*show) (Render3 *r)
    );

    void render(FS_Drivers_t *drivers);
    Color_RGB* getCurrentBuffer();
    void setSize(int rows, int columns);
    int getRows();
    int getColumns();
};

// #endif