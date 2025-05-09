// #define DEBUG_RESAMPLE
// #define DEBUG_WEIGHT
// #define DEBUG_TS_MAP

#include <iostream>
#include <stdio.h>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "mcl_utils.h"


// This function currently uses a CDF resample strategy in which a vector of scores
// is built from the weights and then a random number is generated using a uniform
// distribution. The index of the score that is greater than the random number is
// used to select the particle. The selected particle is then added to a vector of
// sampled particles. Finally, a random number is generated to select a particle
// from the sampled particles and noise is added to the selected particle to create
// all the resampled particles
// TODO: Change the resampling strategy to a more efficient one
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
        std::uniform_real_distribution<float> dart(0, score_base);

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

        // Add gaussian noise and write to resampled_particles using uniform distribution to select the particle
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

// Function to calculate the weight of each particle based on the laser scan and map
// This function uses the occupancy grid map and for ach of the particles it models 
// the laser scan rays and checks if they hit an obstacle or not, depending on this
// the weight of the particle is calculated to finally normalize these weights
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
            
            // For each particle calculate the weight based on the laser scan hits and map occupancy grid
            for (size_t j = 0; j < scan_size; j++){
                float angle = scan_angles[j];
                float range = scan_ranges[j];

                if (range >= max_range || range < 0.0) continue;
                float ray_angle = theta + angle;

                // Normalize angle to [-π, π]
                while (ray_angle > M_PI)
                ray_angle -= 2.0f * M_PI;
                while (ray_angle < -M_PI)
                ray_angle += 2.0f * M_PI;

                // Calculate the beam endpoint accding to the particle pose
                float beam_x = x + range * std::cos(ray_angle);
                float beam_y = y + range * std::sin(ray_angle);

                // Check if the beam is within the map bounds
                int map_x = int((beam_x - origin_x) / map_resolution);
                int map_y = int((beam_y - origin_y) / map_resolution);
                if (map_x < 0 or map_x >= map_width or map_y < 0 or map_y >= map_height) continue;
                
                // Check value of the map cell 
                int cell_value = map_array[map_y * map_width + map_x];
                particle_weight += cell_value >= 100 ? 1.0 : 0.0;
            }
    
            weights[i] = particle_weight;
        }

        // Normalize weights
        float sum = 0.0;
        for (size_t i = 0; i < num_particles; i++){
            sum += weights[i];
        }

        if (sum > 0.0){
            for (size_t i = 0; i < num_particles; i++){
                weights[i] /= sum;

                // Find the maximum weight and corresponding particle
                if (weights[i] > max_score){
                    max_score = weights[i];
    
                    max_particle[0] = particles[i * num_dimensions + 0];
                    max_particle[1] = particles[i * num_dimensions + 1];
                    max_particle[2] = particles[i * num_dimensions + 2];
                }
            }
        } else {
            std::cout << "Warning: All weights are zero!" << std::endl;
            for (size_t i = 0; i < num_particles; i++){
                weights[i] = 1.0 / num_particles;
            }
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
        return false;
    }
}

bool ts_map_update(int x1, int y1, int slam_points, float* scan_ranges, int max_range,
                   int* map_array, float TS_MAP_SCALE, double* x_slam, double* y_slam,
                   double TS_HOLE_WIDTH, double origin_x, double origin_y, double x, double y, float theta, int quality,
                   int TS_NO_OBSTACLE, int TS_OBSTACLE, int TS_MAP_SIZE) {
    double c , s , q;
    double x2p , y2p;
    int i, x2, y2, xp, yp, value;
    
    double add, dist;
    // check all variables with print
    // printf("x1: %d y1: %d\n", x1, y1);
    // fflush(stdout);
    // printf("slam_points: %d\n", slam_points);
    // printf("scan_ranges: ");
    // for (i = 0; i < slam_points; i++) {
    //     printf("%f ", scan_ranges[i]);
    // }
    // printf("\n");
    // printf("max_range: %d\n", max_range);
    // printf("TS_MAP_SCALE: %f\n", TS_MAP_SCALE);
    // printf("x_slam: ");
    // for (i = 0; i < slam_points; i++) {
    //     printf("%f ", x_slam[i]);
    // }
    // printf("\n");
    // printf("y_slam: ");
    // for (i = 0; i < slam_points; i++) {
    //     printf("%f ", y_slam[i]);
    // }
    // printf("\n");
    // printf("TS_HOLE_WIDTH: %d\n", TS_HOLE_WIDTH);
    // printf("origin_x: %f\n", origin_x);
    // printf("origin_y: %f\n", origin_y);
    // printf("x: %f\n", x);
    // printf("y: %f\n", y);
    // printf("theta: %f\n", theta);
    // printf("quality: %d\n", quality);
    // printf("TS_NO_OBSTACLE: %d\n", TS_NO_OBSTACLE);
    // fflush(stdout);
    // printf("TS_OBSTACLE: %d\n", TS_OBSTACLE);
    // fflush(stdout);
    // printf("TS_MAP_SIZE: %d\n", TS_MAP_SIZE);
    // fflush(stdout);
    // printf("map_array: ");
    // for (i = 0; i < TS_MAP_SIZE * TS_MAP_SIZE; i++) {
    //     printf("%d ", map_array[i]);
    // }
    // printf("len map_array: %ld\n", sizeof(map_array));
    // fflush(stdout);


    c = cos(theta);
    s = sin(theta);
    try{
        for(int i = 0; i != slam_points; i++) {
            printf("flag 1\n");
            x2p = c * x_slam[i] - s * y_slam[i];
            y2p = s * x_slam[i] + c * y_slam[i];
            xp = (int)floor(origin_x + (x + x2p) * TS_MAP_SCALE + 0.5);
            yp = (int)floor(origin_y + (y + y2p) * TS_MAP_SCALE + 0.5);
            dist = sqrt(x2p * x2p + y2p * y2p);
            add = TS_HOLE_WIDTH / 2 / dist;
            printf("Index %d: x2p=%.4f y2p=%.4f dist=%.4f\n", i, x2p, y2p, dist);
            fflush(stdout);
            x2p *= TS_MAP_SCALE * (1 + add);
            y2p *= TS_MAP_SCALE * (1 + add);
            x2 = (int)floor(origin_x + x * TS_MAP_SCALE + x2p + 0.5);
            y2 = (int)floor(origin_y + y * TS_MAP_SCALE + y2p + 0.5);
            if((int)scan_ranges[i] == max_range){
                q = quality / 2;
                value = TS_NO_OBSTACLE;
            } else {
                q = quality;
                value = TS_OBSTACLE;
            }
            printf ("flag 3\n");
            ts_map_laser_ray(map_array, x1, y1, x2, y2, xp, yp, value, q, TS_MAP_SIZE, TS_NO_OBSTACLE);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception in ts_map_update: " << e.what() << std::endl;
        return false;
    }

}

void ts_map_laser_ray(int* map_array, int x1, int y1, int x2, int y2, int xp, int yp,
                      int value, int alpha, int TS_MAP_SIZE, int TS_NO_OBSTACLE) {
    int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
    int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
    int* ptr;
    printf("flag 1\n");

    if(x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE) return;

    x2c = x2;
    y2c = y2;

    if(x2c < 0){
        if(x2c == x1) return;
        y2c += (y2c - y1) * (0 - x2c) / (x2c - x1);
        x2c = 0;
    }
    if(x2c >= TS_MAP_SIZE){
        if(x2c == x1) return;
        y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
        x2c = TS_MAP_SIZE - 1;
    }
    if(y2c < 0){
        if(y2c == y1) return;
        x2c += (x1 - x2c) * (0 - y2c) / (y1 - y2c);
        y2c = 0;
    }
    if(y2c >= TS_MAP_SIZE){
        if(y2c == y1) return;
        x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
        y2c = TS_MAP_SIZE - 1;
    }

    printf("flag 2\n");
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    dxc = abs(x2c - x1);
    dyc = abs(y2c - y1);
    incptrx = x2 > x1 ? 1 : -1;
    incptry = y2 > y1 ? TS_MAP_SIZE : -TS_MAP_SIZE;
    sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;
    if(dx > dy) derrorv = abs(xp - x2);
    else {
        std::swap(dx, dy);
        std::swap(dxc, dyc);
        std::swap(incptrx, incptry);
        derrorv = abs(yp - y2);
    }
    printf("derrorv: %d\n", derrorv);
    error = 2 * dyc - dxc;
    horiz = 2 * dyc;
    diago = 2 * (dyc - dxc);
    errorv = derrorv / 2;
    incv = (value - TS_NO_OBSTACLE) / derrorv;
    printf("flag 4\n");
    incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;
    ptr = map_array + y1 * TS_MAP_SIZE + x1;
    pixval = TS_NO_OBSTACLE;
    for (x = 0; x <= dxc; x++, ptr += incptrx) {
        if (x > dx - 2 * derrorv) {
            if (x <= dx - derrorv) {
                pixval += incv;
                errorv += incerrorv;
                if (errorv > derrorv) {
                    pixval += sincv;
                    errorv -= derrorv;
                }
            } else {
                pixval -= incv;
                errorv -= incerrorv;
                if (errorv < 0) {
                    pixval -= sincv;
                    errorv += derrorv;
                }
            }
        }
        // Integration into the map
        //check if the pointer is within the map bounds
        if (ptr < map_array || ptr >= map_array + TS_MAP_SIZE * TS_MAP_SIZE) {
            std::cerr << "Pointer out of bounds!" << std::endl;
            return;
        }
        *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) / 256;
        if (error > 0) {
            ptr += incptry;
            error += diago;
        } else error += horiz;
    }

    printf("flag 3\n");
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

    // normalize weights
    float sum = 0.0f;
    for (int i = 0; i < num_particles; i++) {
        sum += weights[i];
    }
    if (sum > 0.0f) {
        for (int i = 0; i < num_particles; i++) {
            weights[i] /= sum;
        }
    } else {
        std::cout << "Warning: All weights are zero!" << std::endl;
        for (int i = 0; i < num_particles; i++) {
            weights[i] = 1.0f / num_particles;
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

#ifdef DEBUG_TS_MAP
int main(int argc, char** argv) {
    std::cout << "START" << std::endl;

    int x1 = 50;
    int y1 = 50;
    int slam_points = 5;
    float scan_ranges[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    int max_range = 10;
    unsigned char map_array[10000]; // Example map size
    float TS_MAP_SCALE = 1.0f;
    float x_slam[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float y_slam[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    int TS_HOLE_WIDTH = 600;
    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float x = 1.0f;
    float y = 1.0f;
    float theta = M_PI / 4; // Example angle
    int quality = 100;
    int TS_NO_OBSTACLE = 50;
    int TS_OBSTACLE = 200;
    int TS_MAP_SIZE = 100;

    // Initialize map array
    for (int i = 0; i < TS_MAP_SIZE * TS_MAP_SIZE; i++) {
        map_array[i] = TS_NO_OBSTACLE; // Example initialization
    }

    std::cout << "Updating TS map..." << std::endl;
    bool result = ts_map_update(x1, y1, slam_points, scan_ranges, max_range,
                                map_array, TS_MAP_SCALE, x_slam, y_slam,
                                TS_HOLE_WIDTH, origin_x, origin_y,
                                x, y, theta, quality,
                                TS_NO_OBSTACLE, TS_OBSTACLE, TS_MAP_SIZE);

    if (result) {
        std::cout << "TS map update successful!" << std::endl;
        // Print updated map array
        for (int i = 0; i < TS_MAP_SIZE; i++) {
            for (int j = 0; j < TS_MAP_SIZE; j++) {
                std::cout << (int)map_array[i * TS_MAP_SIZE + j] << " ";
            }
        }
    }
}
#endif