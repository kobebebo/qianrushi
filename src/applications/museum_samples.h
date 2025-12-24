#ifndef MUSEUM_SAMPLES_H
#define MUSEUM_SAMPLES_H

#include <rtdef.h>

typedef struct
{
    float temp_c;
    float humi_pct;
    float light_lux;
    float cap_raw;
} museum_sample_t;

#define MUSEUM_SAMPLE_COUNT 500

extern const museum_sample_t museum_samples[MUSEUM_SAMPLE_COUNT];
extern const rt_size_t museum_sample_count;

#endif
