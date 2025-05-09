#include <iostream>
#include <stdio.h>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_set>
#include <unordered_map>
#include <memory>


#ifndef CORE_SLAM_H_
#define CORE_SLAM_H_

typedef std::tuple<float, float, float> Particle;

namespace puzzlebot_navigation
{
    namespace SLAM
    {
        class core_slam
        {
            private:
                // PARTICLE PARAMS
                std::shared_ptr<std::vector<Particle>> particles;
                std::shared_ptr<std::vector<float>> weights;
                int num_particles;
                int num_dimensions;
                float theta_noise;
                float trans_noise;

                // MAP PARAMS

                

            public:
                core_slam(/* args */);
                ~core_slam();

                    
                bool weight_slam_particles(
                    int* map_array, float* map_origin, int* map_shape, float map_resolution,
                    float* scan_angles, float* scan_ranges, int scan_size, float max_range,
                    int num_particles, int num_dimensions, float* particles,
                    float* max_particle, float* weights,
                    int* new_map_array, float* new_map_origin, int* new_map_shape);
                
                bool resample_particles(
                    float* weights, float* particles,
                    float* resampled_particles);
        };
    }        
}

#endif /* CORE_SLAM_H_ */