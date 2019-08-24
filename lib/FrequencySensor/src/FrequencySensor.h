// #ifndef __FREQUENCY_SENSOR_H
// #define __FREQUENCY_SENSOR_H

#pragma once

#include "Arduino.h"
#include "bucketer.h"
#include "fft.h"

typedef struct {
    float *values;
    float *params;
    int size;
} Filter_t;

typedef struct {
    float gain;
    float offset;
    float diffGain;
    float diffAbs;
    float sync;
    float preemph;

    int mode;
    int columnDivider;
} FS_Config_t;

typedef struct {
    int size;
    Filter_t *filter;
    float *gain;
    float *err;
    float kp;
    float kd;
} FS_GainController_t;

typedef struct {
    int size;
    float *valueHistory;
    float *scales;
    float *offsets;
} FS_ValueScaler_t;

typedef struct {
    float **amp;
    float *diff;
    float *energy;
    float bass;
    float *scales;
    int columnIdx;

    int size;
    int length;
} FS_Drivers_t;

typedef struct {
    int size;
    int columns;
    int columnIdx;
    FS_Config_t *config;

    FS_GainController_t *gc;

    Filter_t *gainFilter;
    Filter_t *gainFeedback;
    Filter_t *diffFilter;
    Filter_t *diffFeedback;
    Filter_t *scaleFilter;

    FS_Drivers_t *drivers;

    int renderLock;
} FS_Module_t;

typedef struct {
    int size;
    fft_config_t *fft;
    float *window;
    Bucketer_t *bucketer;
    FS_Module_t *fs;

    short *dacOutput;
    float *buckets;
} Audio_Processor_t;

Audio_Processor_t* NewAudioProcessor(int size, int buckets, int columns, short* dacBuffer);
void Audio_Process(Audio_Processor_t *a, float *input);
void FS_Process(FS_Module_t *f, float *input);
float* FS_GetColumn(FS_Drivers_t *d, int column);

// #endif