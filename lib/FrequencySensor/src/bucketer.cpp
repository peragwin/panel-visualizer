#include "bucketer.h"

static float to_log_scale(float val) {
    return (log2(1 + val));
}

static float from_log_scale(float val) {
    return (exp2(val)) - 1;
}

Bucketer_t* NewBucketer(int size, int buckets, float f_min, float f_max) {
    int *indices = (int*)malloc((buckets-1) * sizeof(int));
    float s_min = to_log_scale(f_min);
    float s_max = to_log_scale(f_max);
    int last_index = 0;

    float space = (s_max - s_min) / (float)buckets;
    float adj_space;
    // in scale space, how far is a unit of index
    float offset_delta = (s_max - s_min) / (float)size;
    float offset = 1;

    float v;
    int idx;
    for (int i = 0; i < buckets - 1; i++) {
        // the bucket space needs adjustment if we've accumulated offset
        adj_space = space - offset_delta * offset / (float)buckets;

        v = from_log_scale((float)(i+1)*adj_space + s_min + offset_delta*offset);
        idx = (int)(ceil((float)size * v / f_max));

        // increment the offset if the next index hasn't incremented by at least 1;
        if (idx <= last_index) {
            idx = last_index + 1;
            offset++;
        }

        if (idx >= size) {
            idx = size - 1;
        }

        indices[i] = idx;
        last_index = idx;
    }

    Bucketer_t *b = (Bucketer_t*)malloc(sizeof(Bucketer_t));
    b->size = size;
    b->buckets = buckets;
    b->indices = indices;
    return b;
}

void Bucket(Bucketer_t *b, float *frame, float *output) {
    int start, stop;
    float sum;
    for (int i = 0; i < b->buckets; i++) {
        start = (i == 0) ? 0 : b->indices[i-1];
        stop = (i == b->buckets - 1) ? b->size : b->indices[i];
        sum = 0;
        for (int j = start; j < stop; j++) {
            sum += frame[j]; 
        }
        output[i] = sum / (float)(stop - start);
    }
}
