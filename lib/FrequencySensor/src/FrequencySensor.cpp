#include "FrequencySensor.h"
#include <stdlib.h>
#include "bucketer.h"
#include "fast_log.h"
#include "fft.h"

#define MAX_SIZE 16
#define MAX_LENGTH 32

Filter_t* NewFilter(int size, float *params, int nparams) {
    float *values = (float*)calloc(MAX_SIZE, sizeof(float));
    float *fparams = (float*)malloc(nparams*sizeof(float));
    memcpy(fparams, params, nparams*sizeof(float));

    Filter_t *f = (Filter_t*)malloc(sizeof(Filter_t));
    f->params = fparams;
    f->size = size;
    f->values = values;
    
    return f;
}

void filter_setSize(Filter_t *f, int size) {
    f->size = size;
}

void apply_filter(Filter_t *f, float *frame) {
    for (int i = 0; i < f->size; i++) {
        f->values[i] = f->params[0] * frame[i] + f->params[1] * f->values[i];
    }
}

FS_Drivers_t* NewDrivers(int size, int columns) {
    float **amp = (float**)malloc(MAX_LENGTH * sizeof(float*));
    for (int i = 0; i < MAX_LENGTH; i++) {
        float *amp_vals = (float*)calloc(MAX_SIZE, sizeof(float));
        amp[i] = amp_vals;
    }
    float *ampAvg = (float*)calloc(MAX_LENGTH, sizeof(float));
    float *diff = (float*)calloc(MAX_SIZE, sizeof(float));
    float *energy = (float*)calloc(MAX_SIZE, sizeof(float));
    float *scales = (float*)calloc(MAX_LENGTH, sizeof(float));
    for (int i = 0; i < MAX_SIZE; i++) scales[i] = 1;
    
    FS_Drivers_t *d = (FS_Drivers_t*)malloc(sizeof(FS_Drivers_t));
    d->amp = amp;
    d->ampAverage = ampAvg;
    d->diff = diff;
    d->energy = energy;
    d->scales = scales;
    d->bass = 0;
    d->size = size;
    d->length = columns;
    d->columnIdx = 0;
    
    return d;
}

FS_GainController_t* NewGainController(int size, float *filterParams, float kp, float kd) {
    Filter_t *filter = NewFilter(size, filterParams, 2); 

    float *gain = (float*)calloc(MAX_SIZE, sizeof(float));
    float *err = (float*)calloc(MAX_SIZE, sizeof(float));
    
    FS_GainController_t* gc = (FS_GainController_t*)malloc(sizeof(FS_GainController_t));
    gc->size = size;
    gc->filter = filter;
    gc->gain = gain;
    gc->err = err;
    gc->kp = kp;
    gc->kd = kd;

    return gc;
}

FS_Module_t* NewFrequencySensor(int size, int columns) {

    FS_Config_t *config = (FS_Config_t*)malloc(sizeof(FS_Config_t));
    config->offset = 0.25;
    config->gain = 8;
    config->diffGain = 0.1;
    config->diffAbs = 0.5;
    config->sync = 5e-3;
    config->mode = 1;
    config->preemph = 2;
    config->columnDivider = 1;

    float gainParams[2] = {
        0.20, 0.80,
    };
    Filter_t *gainFilter = NewFilter(size, gainParams, 2);

    float gainFeedbackP[2] = {
        -0.001, 0.999,
    };
    Filter_t *gainFeedback = NewFilter(size, gainFeedbackP, 2);

    float diffParams[2] = {
        0.15, .85,
    };
    Filter_t *diffFilter = NewFilter(size, diffParams, 2);

    float diffFeedbackP[2] = {
        -0.0028, 0.2272,
    };
    Filter_t *diffFeedback = NewFilter(size, diffFeedbackP, 2);

    float scaleParams[4] = {
        0.01, 0.99,
        0.001, 0.999,
    };
    Filter_t *scaleFilter = NewFilter(size, scaleParams, 4);
    for (int i = 0; i < size; i++) {
        scaleFilter->values[i] = 1;
    }

    float gainControllerParams[2] = {
        0.001, 0.999,
    };
    float kp = 0.001;
    float kd = 0.005;
    FS_GainController_t *gc = NewGainController(size, gainControllerParams, kp, kd);
    
    FS_Drivers_t *drivers = NewDrivers(size, columns);

    FS_Module_t *fs = (FS_Module_t*)malloc(sizeof(FS_Module_t));
    fs->size = size;
    fs->columns = columns;
    fs->columnIdx = 0;
    fs->config = config;
    fs->gainFilter = gainFilter;
    fs->gainFeedback = gainFeedback;
    fs->diffFilter = diffFilter;
    fs->diffFeedback = diffFeedback;
    fs->scaleFilter = scaleFilter;
    fs->gc = gc;
    fs->drivers = drivers;
    fs->renderLock = 0;

    return fs;
}

void FS_SetSize(FS_Module_t *f, int size, int columns) {
    filter_setSize(f->gainFilter, size);
    filter_setSize(f->gainFeedback, size);
    filter_setSize(f->diffFilter, size);
    filter_setSize(f->diffFeedback, size);
    filter_setSize(f->scaleFilter, size);
    filter_setSize(f->gc->filter, size);
    f->size = size;
    f->columns = columns;
}

static void init_blackman_window(float *window, int size) {
  for (int i = 0; i < size; i++) {
    // blackman-harris window
    float c1 = cosf(2.0 * PI * (float)i / (float)(size - 1));
    float c2 = cosf(4.0 * PI * (float)i / (float)(size - 1));
    float c3 = cosf(6.0 * PI * (float)i / (float)(size - 1));
    window[i] = 0.35875 - 0.48829 * c1 + 0.14128 * c2 - 0.01168 * c3;
    window[i] /= 0x800000; //32768; // 2**15 to get a range of [-1, 1]
  }
}

Audio_Processor_t* NewAudioProcessor(int size, int buckets, int columns, short *dacBuffer) {
    Audio_Processor_t *ap = (Audio_Processor_t*)malloc(sizeof(Audio_Processor_t));
    ap->size = size;

    float *window = (float*)malloc(size*sizeof(float));
    init_blackman_window(window, size);
    ap->window = window;
    
    int bucketSize = (size - (4 * size / 24)) / 2;
    ap->bucketer = NewBucketer(bucketSize, buckets, 32, 12000);
    float *bucketArray = (float*)malloc(MAX_SIZE*sizeof(float));
    ap->buckets = bucketArray;

    fft_config_t *fft = fft_init(size, FFT_REAL, FFT_FORWARD, (float*)-1, (float*)-1);
    ap->fft = fft;

    ap->fs = NewFrequencySensor(buckets, columns);

    return ap;
}

void AP_SetSize(Audio_Processor_t *a, int size, int columns) {
    if (size > MAX_SIZE) size = MAX_SIZE;
    if (columns > MAX_LENGTH) columns = MAX_LENGTH;
    FS_SetSize(a->fs, size, columns);
    Bucketer_SetBuckets(a->bucketer, size);
}

// converts 24bit right shifted 2s complement into signed ints
void convert_to_signed(int *input, int *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = (input[i] << 8) >> 8;
    }
}

// static long avg = 0;

// removes the dc component by subtracting the average
void remove_dc_component(float *input, float *output, int size) {
    long sum = 0;
    for (int i = 0; i < size; i++) {
        sum += input[i];
    }
    sum /= size;

    // avg = (5 * sum + 99995 * avg) / 100000;

    for (int i = 0; i < size; i++) {
        output[i] = input[i] - sum;
    }
}

void convert_to_float(int *input, float *output, int size) {
    for (int i = 0; i < size; i++) {
        output[i] = (float)input[i];
    }
}

void apply_window(float *input, float *window, int size) {
    for (int i = 0; i < size; i++){
        input[i] = window[i] * input[i];
    }
}

// fast_log just returns the floating point's exponent!
// float fast_log(float x) {
//     unsigned int v = *((int*)&x);
//     return (float)(((v >> 23) & 0xFF) - 127);
// }

void power_spectrum(fft_config_t *fft, float *input, float *output) {
    int size = fft->size;
    fft->input = input;
    fft->output = output;
    
    fft_execute(fft);

    for (int i = 0; i < size; i+=2) {
        float r = output[i];
        float c = output[i+1];
        float x = log2_2521(1 + r*r + c*c);
        output[i/2] = x;
    }
}

int audio_process_count = 0;

static float fftSum = 0;

void Audio_Process(Audio_Processor_t *a, float *input) {

    remove_dc_component(input, input, a->size);
    
    float fftFrame[a->size];
    float *bucketFrame = a->buckets;

    apply_window(input, a->window, a->size);

    // Serial.println("FRrame:");
    // for (int i = 0; i < a->size; i+=(a->size/16)) {
    //     Serial.print(input[i]);
    //     Serial.print(", ");
    // }

    power_spectrum(a->fft, input, fftFrame);

    float sum = 0;
    for (int i = 1; i < a->size/2; i++) {
        sum += fftFrame[i];
    }
    int badFrame = 0;
    if (audio_process_count++ > 1000 && sum > (32 * fftSum)) badFrame = 1; //Error_Handler();
    fftSum = .01 * sum + .99 * fftSum;
    if (badFrame) {
        Serial.println("BAD FRAME!");
        return;
    }

    Bucket(a->bucketer, fftFrame, bucketFrame);

    // Serial.println("Buckets:");
    for (int i = 0; i < a->bucketer->buckets; i++) {
        // Serial.print(bucketFrame[i]);
        // Serial.print(", ");
        if (isnan(bucketFrame[i])) {
            Serial.println("NAN FRAME!");
            return;
        }
    }

    FS_Process(a->fs, bucketFrame);
}

void apply_preemphasis(FS_Module_t *f, float *frame) {
    float incr = (f->config->preemph - 1) / (float)f->size;
    for (int i = 0; i < f->size; i++) {
        frame[i] *= 1 + (float)i*incr;
    }
}

static float log_error(float x) {
    x = 1.000001 - x;
    float sign = (x < 0) ? 1.0 : -1.0;
    float a = (x < 0) ? -x : x;
    return sign * log2_2521(a);
}

void apply_gain_control(FS_GainController_t *g, float *frame) {
    // apply gain
    for (int i = 0; i < g->size; i++) {
        frame[i] = g->gain[i] * frame[i];
        // if (i==0 && isnan(frame[i])) {
        //     Serial.println("bad");
        //     // Serial.println(f->params[0]);
        //     Serial.println(g->gain[i]);
        //     Serial.println(frame[i]);
        // }
    }

    // apply filter
    apply_filter(g->filter, frame);

    // calculate error
    float e[g->size];
    for (int i = 0; i < g->size; i++) {
        e[i] = log_error(1 - g->filter->values[i]);
        if (i==0 && isnan(e[i])) {
            Serial.println("bad");
            // Serial.println(f->params[0]);
            Serial.println(g->filter->values[i]);
            Serial.println(e[i]);
            e[i] = 0;
        }
    }
    
    // apply pd controller
    float u;
    float kp = g->kp;
    float kd = g->kd;
    for (int i = 0; i < g->size; i++) {
        float gain = g->gain[i];
        float err = g->err[i];
        u = kp * e[i] + kd * (e[i] - err);
        gain += u;
        if (gain > 1e8) gain = 1e8;
        if (gain < 1e-8) gain = 1e-8;
        if (isnan(gain)) {
            gain = 1;
        }
        g->gain[i] = gain;
        g->err[i] = e[i];
    }
}

void apply_filters(FS_Module_t *f, float *frame) {
    float diff_input[f->size];
    memcpy(diff_input, f->gainFilter->values, f->size * sizeof(float));

    apply_filter(f->gainFilter, frame);
    apply_filter(f->gainFeedback, frame);

    // calculate differential of this update
    for (int i = 0; i < f->size; i++) {
        diff_input[i] = f->gainFilter->values[i] - diff_input[i];
    }
    apply_filter(f->diffFilter, diff_input);
    apply_filter(f->diffFeedback, diff_input);
}

static int effectCounter = 0;

void apply_effects(FS_Module_t *f) {
    float dg = f->config->diffGain;
    float ag = f->config->gain;
    float ao = f->config->offset;

    // Serial.print("s ");
    // Serial.println(f->size);
    // Serial.print("l ");
    // Serial.println(f->columns);

    // apply column animation
    if (f->config->mode == 1 || f->config->mode == 2) {
        effectCounter++;
        effectCounter %= f->config->columnDivider;
        if (effectCounter == 0) {
            f->columnIdx++;
            f->columnIdx %= f->columns;
            if (f->config->mode == 1) { // columns decay
                // TODO: decimate updates to fit into output size
                float decay = 1.0 - (2.0 / (float)f->columns);
                for (int i = 0; i < f->columns; i++) {
                    for (int j = 0; j < f->size; j++) {
                        f->drivers->amp[i][j] *= decay;
                    }
                    f->drivers->ampAverage[i] *= decay;
                }
            }
        }
    }

    f->drivers->columnIdx = f->columnIdx;
    float *amp = f->drivers->amp[f->columnIdx];

    for (int i = 0; i < f->size; i++) {
        amp[i] = f->gainFilter->values[i] + f->gainFeedback->values[i];
    }

    float *diff = f->drivers->diff;
    for (int i = 0; i < f->size; i++) {
        diff[i] = f->diffFilter->values[i] + f->diffFeedback->values[i];
    }

    float *energy = f->drivers->energy;
    float ph;
    for (int i = 0; i < f->size; i++) {
        amp[i] = ao + ag * amp[i];
        ph = energy[i] + .001;
        ph -= dg * ((1 - f->config->diffAbs) * diff[i] 
                    + f->config->diffAbs * (diff[i] < 0 ? -diff[i] : diff[i]));
        energy[i] = ph;
    }
}


void apply_sync(FS_Module_t *f) {
    float *diffA = f->drivers->diff;
    for (int i = 1; i < f->size-1; i++) {
        float dl = diffA[i-1];
        float d = diffA[i];
        float dh = diffA[i+1];
        diffA[i] = (dl + d + dh) / 3;
    }
    
    float *energy = f->drivers->energy;
    
    float mean = 0;
    for (int i = 0; i < f->size; i++) {
        mean += energy[i];
    }
    mean /= (float)f->size;

    float diff;
    float sign;
    for (int i = 0; i < f->size; i++) {
        if (i != 0) {
            diff = energy[i-1] - energy[i];
            sign = (diff < 0) ? -1.0 : 1.0;
            diff = sign * diff * diff;
            energy[i] += f->config->sync * diff;
        }
        if (i != f->size-1) {
            diff = energy[i+1] - energy[i];
            sign = (diff < 0) ? -1.0 : 1.0;
            diff = sign * diff * diff;
            energy[i] += f->config->sync * diff;
        }
        diff = mean - energy[i];
        sign = (diff < 0) ? -1.0 : 1.0;
        diff = sign * diff * diff;
        energy[i] += f->config->sync * diff;
    }

    mean = 0;
    for (int i = 0; i < f->size; i++) {
        mean += energy[i];
    }
    mean /= (float)f->size;

    if (mean < -2*PI) {
        // wait until all elements go past the mark so theres no sign flips
        for (int i = 0; i < f->size; i++) {
            if (energy[i] >= -2*PI) return;
        }
        for (int i = 0; i < f->size; i++) {
            energy[i] = 2*PI + fmod(energy[i], 2*PI);
        }
        mean = 2*PI + fmod(mean, 2*PI);
    }
    if (mean > 2*PI) {
        for (int i = 0; i < f->size; i++) {
            if (energy[i] <= 2*PI) return;
        }
        for (int i = 0; i < f->size; i++) {
            energy[i] = fmod(energy[i], 2*PI);
        }
        mean = fmod(mean, 2*PI);
    }
}

void apply_scaling(FS_Module_t *f, float *frame) {
    for (int i = 0; i < f->size; i++) {
        float vsh = f->scaleFilter->values[i];
        float vs = f->drivers->scales[i];

        float sval = vs * (frame[i] - 1);
        if (sval < 0) sval = -sval;
        
        float *params = f->scaleFilter->params;
        if (sval < vsh) {
            vsh = params[0] * sval + params[1] * vsh;
        } else {
            vsh = params[2] * sval + params[3] * vsh;
        }

        if (vsh < .001) vsh = .001;
        vs = 1. / vsh;

        f->scaleFilter->values[i] = vsh;
        f->drivers->scales[i] = vs;
    }
    float ampAvg = 0;
    float *amp = FS_GetColumn(f->drivers, 0);
    for (int i = 0; i < f->size; i++) {
        ampAvg += f->drivers->scales[i] * amp[i];
    }
    ampAvg /= f->size;
    f->drivers->ampAverage[f->columnIdx] = ampAvg;
}

void FS_Process(FS_Module_t *f, float *frame) {
    f->renderLock = 1;
    apply_preemphasis(f, frame);
    apply_gain_control(f->gc, frame);
    apply_filters(f, frame);
    apply_effects(f);
    apply_sync(f);
    apply_scaling(f, f->drivers->amp[f->columnIdx]);
    f->renderLock = 0;
}

