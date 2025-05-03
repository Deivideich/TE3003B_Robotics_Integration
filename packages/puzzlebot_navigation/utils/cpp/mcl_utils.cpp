// #define DEBUG_RESAMPLE
// #define DEBUG_WEIGHT

#include <iostream>
#include <stdio.h>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "mcl_utils.h"

bool resample_particles(
    int num_particles, int num_dimensions, float theta_noise, float trans_noise,
    float* weights, float* particles,
    float* resampled_particles) {
    try {
        std::vector<float> particle_scores(num_particles);
        std::vector<std::vector<float>> particles_sampled;
        float score_base = 0.0;

        // Build the CDF (cumulative distribution function) from weights
        for (int i = 0; i < num_particles; i++) {
            score_base += weights[i];
            particle_scores[i] = score_base;
        }

        std::mt19937 gen(std::random_device{}());
        std::normal_distribution<float> dart(0, score_base);

        // Perform resampling wheel (low variance resampling)
        for (int i = 0; i < num_particles; i++) {
            float random_value = dart(gen);
            int index = std::lower_bound(particle_scores.begin(), particle_scores.end(), random_value) - particle_scores.begin();

            std::vector<float> dump_particle;
            for (int j = 0; j < num_dimensions; j++) {
                dump_particle.push_back(particles[index * num_dimensions + j]);
            }
            particles_sampled.push_back(dump_particle);
        }

        std::uniform_int_distribution<int> particle_index_distribution(0, num_particles - 1);
        std::normal_distribution<float> trans_noise_distribution(0, trans_noise);
        std::normal_distribution<float> theta_noise_distribution(-theta_noise, theta_noise);

        // Add noise and write to resampled_particles
        for (int i = 0; i < num_particles; i++) {
            std::size_t number = particle_index_distribution(gen);
            std::vector<float>& dump_particle = particles_sampled[number];
            float x = dump_particle[0] + trans_noise_distribution(gen);
            float y = dump_particle[1] + trans_noise_distribution(gen);
            float theta = dump_particle[2] + theta_noise_distribution(gen);
            
            resampled_particles[i * num_dimensions + 0] = x;
            resampled_particles[i * num_dimensions + 1] = y;
            resampled_particles[i * num_dimensions + 2] = theta;
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
        return false;
    }
}

bool weight_particles(
    int* map_array, float* map_origin, int* map_shape, float map_resolution, 
    float* scan_angles, float* scan_ranges, int scan_size, float max_range,
    int num_particles, int num_dimensions, float* particles, 
    float* max_particle, float* weights){
    try{
        float origin_x = map_origin[0];
        float origin_y = map_origin[1];

        int map_height = map_shape[0];
        int map_width = map_shape[1];
        float max_score = 0.0;
        
        for (size_t i = 0; i < num_particles; i++){
            float x = particles[i * num_dimensions + 0];
            float y = particles[i * num_dimensions + 1];
            float theta = particles[i * num_dimensions + 2];
            float particle_weight = 0.0;
            
            for (size_t j = 0; j < scan_size; j++){
                // std::cout << "Debug: " << i << " " << j << std::endl;
                float angle = scan_angles[j];
                float range = scan_ranges[j];
                if (range >= max_range || range < 0.0) continue;

                float beam_x = x + range * std::cos(theta + angle);
                float beam_y = y + range * std::sin(theta + angle);

                int map_x = int((beam_x - origin_x) / map_resolution);
                int map_y = int((beam_y - origin_y) / map_resolution);
                
                if (map_x < 0 or map_x >= map_width or map_y < 0 or map_y >= map_height) continue;
                int cell_value = map_array[map_y * map_width + map_x];  // Fixed indexing
                
                
                particle_weight += cell_value >= 100 ? float(cell_value / 100.0) : 0.0;
            }
    
            weights[i] = particle_weight / scan_size;
            
            if (particle_weight > max_score){
                max_score = particle_weight;

                max_particle[0] = x;
                max_particle[1] = y;
                max_particle[2] = theta;
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
        return false;
    }
}

#ifdef DEBUG_RESAMPLE
int main(int argc, char** argv) {
    std::cout << "START" << std::endl;

    int num_particles = 300;
    int num_dimensions = 3;
    float theta_noise = float(M_PI/16.0);   // radians
    float trans_noise = 0.05f;  // meters

    float weights[num_particles];
    float particles[num_particles * num_dimensions];
    float* resampled_particles = new float[num_particles * num_dimensions];

    // Initialize weights and particles
    for (int i = 0; i < num_particles; i++) {
        weights[i] = 1.0f;
        for (int j = 0; j < num_dimensions; j++) {
            particles[i * num_dimensions + j] = static_cast<float>(i + j);
        }
    }

    std::cout << "Resampling..." << std::endl;
    bool result = resample_particles(num_particles, num_dimensions, theta_noise, trans_noise,
                                     weights, particles, resampled_particles);

    if (result) {
        std::cout << "Resampling successful!" << std::endl;
        for (int i = 0; i < num_particles; i++) {
            std::cout << "Resampled Particle " << i << ": ";
            for (int j = 0; j < num_dimensions; j++) {
                std::cout << resampled_particles[i * num_dimensions + j] << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "Resampling failed!" << std::endl;
    }

    // Only delete memory that was dynamically allocated
    delete[] resampled_particles;

    std::cout << "END" << std::endl;
    return 0;
}
#endif

#ifdef DEBUG_WEIGHT
int main(int argc, char** argv) {
    std::cout << "START" << std::endl;

    int map_width = 100;
    int map_height = 100;
    float map_resolution = 0.1f;
    int map_array[map_width * map_height];
    float map_origin[2] = {0.0f, 0.0f};
    int map_shape[2] = {int(map_height), int(map_width)};

    // Initialize a simple map (all cells set to 255)
    for (int i = 0; i < map_width * map_height; i++) {
        map_array[i] = 255;
    }

    int num_particles = 300;
    int num_dimensions = 3;
    float particles[num_particles * num_dimensions];
    float weights[num_particles];
    float max_particle[num_dimensions];

    // Initialize particles
    for (int i = 0; i < num_particles; i++) {
        particles[i * num_dimensions + 0] = static_cast<float>(i % map_width) * map_resolution;
        particles[i * num_dimensions + 1] = static_cast<float>(i / map_width) * map_resolution;
        particles[i * num_dimensions + 2] = 0.0f; // Orientation
        weights[i] = 0.0f;
    }

    int num_scans = 10;
    float scan_angles[num_scans];
    float scan_ranges[num_scans];
    float max_range = 5.0f;

    // Initialize scan angles and ranges
    for (int i = 0; i < num_scans; i++) {
        scan_angles[i] = -M_PI / 4 + i * (M_PI / 2 / (num_scans - 1));
        scan_ranges[i] = max_range / 2.0f; // Fixed range for simplicity
    }

    std::cout << "Calculating weights..." << std::endl;
    bool result = weight_particles(map_array, map_origin, map_shape, map_resolution,
                                   scan_angles, scan_ranges, map_width * map_height, max_range,
                                   num_particles, num_dimensions, particles,
                                   max_particle, weights);

    if (result) {
        std::cout << "Weight calculation successful!" << std::endl;
        std::cout << "Max Particle: ";
        for (int i = 0; i < num_dimensions; i++) {
            std::cout << max_particle[i] << " ";
        }
        std::cout << std::endl;

        for (int i = 0; i < num_particles; i++) {
            std::cout << "Particle " << i << " Weight: " << weights[i] << std::endl;
        }
    } else {
        std::cout << "Weight calculation failed!" << std::endl;
    }

    std::cout << "END" << std::endl;

    return 0;
}
#endif