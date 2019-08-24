
// #ifndef bucketer_h
// #define bucketer_h

#pragma once

#include "Arduino.h"
#include <stdlib.h>

typedef struct {
    int size;
    int buckets;
    int *indices;
} Bucketer_t;

Bucketer_t* NewBucketer(int size, int buckets, float f_min, float f_max);
void Bucket(Bucketer_t *b, float *frame, float *output);


// #endif
