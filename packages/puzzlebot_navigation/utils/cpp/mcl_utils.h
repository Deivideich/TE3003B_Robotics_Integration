#pragma once

#ifdef _WIN32
#define EXPORT extern "C" __declspec(dllexport)
#else
#define EXPORT extern "C"
#endif

#ifndef MCL_UTILS_H_
#define MCL_UTILS_H_

EXPORT bool resample_particles(
    int num_particles, int num_dimensions, float theta_noise, float trans_noise,
    float* weights, float* particles,
    float* resampled_particles);

EXPORT bool weight_particles(
    int* map_array, float* map_origin, int* map_shape, float map_resolution,
    float* scan_angles, float* scan_ranges, int scan_size, float max_range,
    int num_particles, int num_dimensions, float* particles, 
    float* max_particle, float* weights);


#endif /* MCL_UTILS_H_ */