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

EXPORT bool ts_map_update(int x1, int y1, int slam_points, float* scan_ranges, int max_range,
                   int* map_array, float TS_MAP_SCALE, double* x_slam, double* y_slam,
                   int TS_HOLE_WIDTH, double origin_x, double origin_y, double x, double y, float theta, int quality,
                   int TS_NO_OBSTACLE, int TS_OBSTACLE, int TS_MAP_SIZE);


void ts_map_laser_ray(int* map_array, int x1, int y1, int x2, int y2, int xp, int yp,
                      int value, int alpha, int TS_MAP_SIZE, int TS_NO_OBSTACLE);

#endif /* MCL_UTILS_H_ */