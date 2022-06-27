#pragma once

#include "Arduino.h"
#include <stdlib.h>

typedef struct
{
    int size;
    int buckets;
    int *indices;
    float f_min;
    float f_max;
} Bucketer_t;

Bucketer_t *NewBucketer(int size, int buckets, float f_min, float f_max);
void Bucket(Bucketer_t *b, float *frame, float *output);
void Bucketer_SetBuckets(Bucketer_t *b, int buckets);
