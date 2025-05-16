// #define DEBUG_RESAMPLE
// #define DEBUG_WEIGHT

#include "puzzlebot_navigation/slam/core_slam.h"
#include <iostream>
#include <algorithm>  // For std::clamp, std::partial_sort_copy
#include <numeric>    // For std::accumulate
#include <stdio.h>
#include <random>
#include <cmath>
#include <vector>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <unordered_set>
#include <unordered_map>
#include <memory>



namespace puzzlebot_navigation
{
    namespace SLAM
    {
        CoreSLAM::CoreSLAM(
            int num_particles, int num_dimensions,
            float theta_noise, float trans_noise) :
            num_particles_(num_particles),num_dimensions_(num_dimensions),
            theta_noise_(theta_noise), trans_noise_(trans_noise)
        {
            map_resolution_ = 0.1; // Default value
            particles_ = std::make_shared<std::vector<Particle>>(num_particles);
            weights_ = std::make_shared<std::vector<float>>(num_particles);
            main_map_ = std::make_shared<uset_pair>();
            particle_map_ = std::unordered_map<int, std::shared_ptr<uset_pair>>();

            std::cout << "CoreSLAM initialized with parameters: "
                << "num_particles=" << num_particles_ 
                << ", num_dimensions=" << num_dimensions_ 
                << ", theta_noise=" << theta_noise_ << std::endl;

             // Initialize with reasonable default size centered around (0,0)
                map_origin_ = {0.0f, 0.0f};  // 5m buffer in each direction
                map_shape_ = {0, 0};         // 10x10m initial size
                recorded_data = {INFINITY, INFINITY, -INFINITY, -INFINITY};
                
                particles_ = std::make_shared<std::vector<Particle>>(num_particles);
                weights_ = std::make_shared<std::vector<float>>(num_particles);
                main_map_ = std::make_shared<uset_pair>();

                initialize_particles();
        }

        CoreSLAM::CoreSLAM() :
            num_particles_(1000), num_dimensions_(3),
            theta_noise_(0.01), trans_noise_(0.01)
        {
            particles_ = std::make_shared<std::vector<Particle>>(num_particles_);
            weights_ = std::make_shared<std::vector<float>>(num_particles_);
            main_map_ = std::make_shared<uset_pair>();
            particle_map_ = std::unordered_map<int, std::shared_ptr<uset_pair>>();

            initialize_particles();
        } 

        void CoreSLAM::initialize_particles()
        {
            for (int i = 0; i < num_particles_; i++) {
                (*particles_)[i] = std::make_tuple(0,0,0); // Initialize particles to (0, 0, 0)
                (*weights_)[i] = 1.0f / num_particles_; // Initialize weights uniformly
            }
        }


        void CoreSLAM::updateMapParams() {
            if (recorded_data.empty() || recorded_data.size() != 4) {
                std::cerr << "Error: recorded_data does not comply with specifications!" << std::endl;
                return;
            } 
        
            float xmin = recorded_data[0];
            float ymin = recorded_data[1];
            float xmax = recorded_data[2];
            float ymax = recorded_data[3];
        
            // Set map origin with margin
            float margin = 0.5f; // 1 meter margin
                
            map_shape_ = {
                static_cast<int>((recorded_data[3] - recorded_data[1] + 2*margin) / map_resolution_),
                static_cast<int>((recorded_data[2] - recorded_data[0] + 2*margin) / map_resolution_)
            };

            map_origin_ = {(recorded_data[1] + 2*margin),
                (recorded_data[0] + 2*margin)};
 
            
        
            std::cout << "Map shape updated to: ("
                      << map_shape_[0] << ", " << map_shape_[1] << ")"
                      << " | origin remains at: (" 
                      << map_origin_[0] << ", " << map_origin_[1] << ")"
                      << std::endl;
        }

        void CoreSLAM::motion_update(float dx, float dy, float dtheta)
        {
            float delta_trans = std::sqrt(dx * dx + dy * dy);
            float delta_rot = std::atan2(dy, dx);

            float trans_noise_coeff = trans_noise_ * 0.2 + theta_noise_ * 0.2;
            float rot_noise_coeff = theta_noise_ * 0.2 + trans_noise_ * 0.2;

            std::mt19937 gen(std::random_device{}());
            std::normal_distribution<float> trans_noise_dist(0, trans_noise_coeff);
            std::normal_distribution<float> rot_noise_dist(0, rot_noise_coeff);

            std::cout << "Motion update: dx=" << dx << ", dy=" << dy << ", dtheta=" << dtheta << std::endl;
            for (auto& particle : *particles_) {
                auto [x, y, theta] = particle;

                float delta_rot1 = std::atan2(dy, dx) - theta;
                float delta_rot2 = dtheta - delta_rot1;

                float delta_trans_noisy = delta_trans + trans_noise_dist(gen);
                float delta_rot1_noisy = delta_rot1 + rot_noise_dist(gen);
                float delta_rot2_noisy = delta_rot2 + rot_noise_dist(gen);

                float x_new = x + delta_trans_noisy * std::cos(theta + delta_rot1_noisy);
                float y_new = y + delta_trans_noisy * std::sin(theta + delta_rot1_noisy);
                float theta_new = theta + delta_rot1_noisy + delta_rot2_noisy;

                // Normalize theta to [-π, π]
                while (theta_new > M_PI) theta_new -= 2.0f * M_PI;
                while (theta_new < -M_PI) theta_new += 2.0f * M_PI;

                particle = std::make_tuple(x_new, y_new, theta_new);
            }
        }


        bool CoreSLAM::initial_guess(
            std::shared_ptr<std::vector<float>> scan_angles,
            std::shared_ptr<std::vector<float>> scan_ranges,
            int scan_size, float max_range, 
            float robot_x, float robot_y, float robot_theta) 
        {
            try {
                if (!particles_) return false;
        
                // Initialize particles with small noise around (0,0)
                std::mt19937 gen(std::random_device{}());
                std::normal_distribution<float> trans_dist(0.0f, 0.001f);
                std::normal_distribution<float> rot_dist(0.0f, 0.001f);
        
                // First pass: determine map bounds
                float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
                for (int j = 0; j < scan_size; ++j) {
                    float range = (*scan_ranges)[j];
                    if (range >= max_range || range < 0.0f) continue;
        
                    float beam_x = range * std::cos((*scan_angles)[j]);
                    float beam_y = range * std::sin((*scan_angles)[j]);
                    
                    recorded_data[0] = std::min(recorded_data[0], beam_x);
                    recorded_data[1] = std::min(recorded_data[1], beam_y);
                    recorded_data[2] = std::max(recorded_data[2], beam_x);
                    recorded_data[3] = std::max(recorded_data[3], beam_y);

                    // auto [map_x, map_y] = worldToMap(beam_x, beam_y);
                    // main_map_->insert({{map_y, map_x}, {beam_y, beam_x}});
                }
        
                // Set map origin with margin
                float margin = 1.0f; // 1 meter margin
                
                map_shape_ = {
                    static_cast<int>((recorded_data[3] - recorded_data[1] + 2*margin) / map_resolution_),
                    static_cast<int>((recorded_data[2] - recorded_data[0] + 2*margin) / map_resolution_)
                };
                map_origin_ = {(recorded_data[1] + 2*margin),
                    (recorded_data[0] + 2*margin)};
     
        
                // Second pass: process scans with proper coordinates
                for (int i = 0; i < num_particles_; i++) {
                    float x_new = robot_x + trans_dist(gen);
                    float y_new = robot_y + trans_dist(gen);
                    float theta_new = robot_theta + rot_dist(gen);
                    (*particles_)[i] = {x_new, y_new, theta_new};
        
                    for (int j = 0; j < scan_size; ++j) {
                        float range = (*scan_ranges)[j];
                        if (range >= max_range || range < 0.0f) continue;
        
                        float beam_x = x_new + range * std::cos(theta_new + (*scan_angles)[j]);
                        float beam_y = y_new + range * std::sin(theta_new + (*scan_angles)[j]);
                        // expandMap(beam_x, beam_y);  // This updates origin and shape as needed
                        auto [map_x, map_y] = worldToMap(beam_x, beam_y);
                        if (map_x < 0 or map_x >= map_shape_[0] or map_y < 0 or map_y >= map_shape_[1]) continue;

                        main_map_->insert({{map_y, map_x}, {beam_y, beam_x}});
                    }
                }
        
                return true;
            } catch (...) {
                return false;
            }
        }

        // This function currently uses a CDF resample strategy in which a vector of scores
        // is built from the weights and then a random number is generated using a uniform
        // distribution. The index of the score that is greater than the random number is
        // used to select the particle. The selected particle is then added to a vector of
        // sampled particles. Finally, a random number is generated to select a particle
        // from the sampled particles and noise is added to the selected particle to create
        // all the resampled particles
        // TODO: Change the resampling strategy to a more efficient one
        bool CoreSLAM::resample_particles() 
        {
            try {
                std::vector<float> particle_scores(num_particles_);
                std::vector<Particle> particles_sampled;
                float score_base = 0.0;
                float sum_squared_weights = 0.0;

                // Build the CDF (cumulative distribution function) from weights
                for (int i = 0; i < num_particles_; i++) {
                    score_base += (*weights_)[i];
                    sum_squared_weights += std::pow((*weights_)[i], 2);
                    particle_scores[i] = score_base;
                }

                std::mt19937 gen(std::random_device{}());
                std::uniform_real_distribution<float> dart(0, score_base);
                std::unordered_set<int> selected_particle_ids;

                // Perform resampling wheel (low variance resampling)
                for (int i = 0; i < num_particles_; i++) {
                    float random_value = dart(gen);
                    int index = std::lower_bound(particle_scores.begin(), particle_scores.end(), random_value) - particle_scores.begin();
                    selected_particle_ids.insert(index);
                    
                    const auto& particle = (*particles_)[index];
                    auto [x, y, theta] = particle;
                    particles_sampled.push_back({x,y,theta});
                }

                // After resampling, update main_map_
                std::cout << "Resampling done. Main map size: " << main_map_->size() << std::endl;  // Check size
                
                std::vector<Particle> resampled_particles(num_particles_);
                std::uniform_int_distribution<int> particle_index_distribution(0, num_particles_ - 1);
                std::normal_distribution<float> trans_noise_distribution(0, trans_noise_);
                std::normal_distribution<float> theta_noise_distribution(-theta_noise_, theta_noise_);

                // Add gaussian noise and write to resampled_particles using uniform distribution to select the particle
                for (int i = 0; i < num_particles_; i++) {
                    std::size_t number = particle_index_distribution(gen);
                    const auto& selected = particles_sampled[number];

                    auto [dump_x, dump_y, dump_theta] = selected;
                    float x = dump_x + trans_noise_distribution(gen);
                    float y = dump_y + trans_noise_distribution(gen);
                    float theta = dump_theta + theta_noise_distribution(gen);
                    
                    resampled_particles[i] = std::make_tuple(x, y, theta);
                }

                // Update the particles_ with the resampled particles
                swap(*particles_, resampled_particles);
                // Clear the resampled_particles to avoid memory leaks
                resampled_particles.clear();

                // // Append data into main_map_ if the cell pair does not exist
                // for (const auto& index : selected_particle_ids) {
                //     const auto& observations = particle_map_[index];
                //     for (const auto& [cell, coords] : *observations) {
                //         auto [map_x, map_y] = worldToMap(coords.second, coords.first);
                //         if (main_map_->find({map_y, map_x}) == main_map_->end()) {
                //             main_map_->insert({{map_y, map_x}, coords});
                //         }
                //     }
                // }

                // particle_map_.clear();

                return true;
            } catch (const std::exception& e) {
                std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
                return false;
            }
        }
        
        
        bool CoreSLAM::weight_slam_particles(
            std::shared_ptr<std::vector<float>> scan_angles,
            std::shared_ptr<std::vector<float>> scan_ranges,
            int scan_size, float max_range,
            std::shared_ptr<Particle> max_particle)
        {
            try {
                if (!main_map_ || main_map_->empty()) return false;
          
                float max_score = -1.0f;
                int best_particle_idx = 0;
                float origin_x = map_origin_[1];
                float origin_y = map_origin_[0];
        
                // First pass: calculate weights and collect observations
                for (size_t i = 0; i < num_particles_; i++) {
                    const auto& [x, y, theta] = (*particles_)[i];
                    float particle_weight = 0.0f;
                    particle_map_[i] = std::make_shared<uset_pair>();
        
                    for (size_t j = 0; j < scan_size; j++) {
                        float angle = (*scan_angles)[j];
                        float range = (*scan_ranges)[j];
        
                        if (range >= max_range || range < 0.0f) continue;
        
                        float ray_angle = theta + angle;
                        // Normalize angle
                        ray_angle = std::fmod(ray_angle + M_PI, 2*M_PI) - M_PI;
        
                        // Calculate beam endpoint
                        float beam_x = x + range * std::cos(ray_angle);
                        float beam_y = y + range * std::sin(ray_angle);

                        // expandMap(beam_x, beam_y);  // This updates origin and shape as needed
                        // Map coordinates
                        // expandMap(beam_x, beam_y);  // This updates origin and shape as needed
                        auto [map_x, map_y] = worldToMap(beam_x, beam_y);
                        
                        if (map_x >= 0 && map_x < map_shape_[0] && map_y >= 0 && map_y < map_shape_[1])
                            particle_weight += (main_map_->find({map_y, map_x}) != main_map_->end()) ? 1.0 : 0.0;
        
                        // Store observation for map update
                        particle_map_[i]->insert({{map_y, map_x}, {beam_y, beam_x}});
                    }
        
                    (*weights_)[i] = particle_weight;
                    if (particle_weight > max_score) {
                        max_score = particle_weight;
                        best_particle_idx = i;
                        *max_particle = (*particles_)[i];
                    }
                }
        
                // // Second pass: update probabilistic map using best particles
                const int top_n = std::max(5, num_particles_/10); // Use top 10% particles
                // Create index array [0, 1, 2, ..., num_particles_-1]
                std::vector<size_t> indices(num_particles_);
                std::iota(indices.begin(), indices.end(), 0);

                // Partial sort the indices based on corresponding weights
                std::partial_sort(
                    indices.begin(), 
                    indices.begin() + std::min(top_n, num_particles_),
                    indices.end(),
                    [this](size_t a, size_t b) { return (*weights_)[a] > (*weights_)[b]; }
                );
                            
                for (size_t i = 0; i < std::min(top_n, num_particles_); ++i) {
                    size_t idx = indices[i];
                    const auto& observations = particle_map_[idx];
                    for (const auto& [cell, coords] : *observations) {
                        // Update recorded data with world coordinates
                        recorded_data[0] = std::min(recorded_data[0], coords.second);
                        recorded_data[1] = std::min(recorded_data[1], coords.first);
                        recorded_data[2] = std::max(recorded_data[2], coords.second);
                        recorded_data[3] = std::max(recorded_data[3], coords.first);
                        updateMapParams();
                        auto[new_map_x, new_map_y] = worldToMap(coords.second, coords.first);
                        if (main_map_->find({new_map_y, new_map_x}) == main_map_->end()) main_map_->insert({{new_map_y, new_map_x}, coords});
                    }
                }        
        
                // Normalize weights
                float sum = std::accumulate(weights_->begin(), weights_->end(), 0.0f);
                if (sum > 0.0f) {
                    for (auto& w : *weights_) w /= sum;
                } else {
                    std::fill(weights_->begin(), weights_->end(), 1.0f/num_particles_);
                }
                
        
                return true;
            } catch (...) {
                return false;
            }
        }


        std::pair<int, int> CoreSLAM::worldToMap(float x, float y) const {
            return {
                static_cast<int>((x - map_origin_[1]) / map_resolution_),
                static_cast<int>((y - map_origin_[0]) / map_resolution_)
            };
        }
        
        std::pair<float, float> CoreSLAM::mapToWorld(int x, int y) const {
            return {
                map_origin_[1] + x * map_resolution_,
                map_origin_[0] + y * map_resolution_
            };
        }
        
        void CoreSLAM::expandMap(float world_x, float world_y) {
            auto [map_x, map_y] = worldToMap(world_x, world_y);
            
            // Check each boundary
            if (map_x < 0) {
                float expand = -map_x * map_resolution_;
                map_origin_[0] -= expand;
                map_shape_[1] += -map_x;
            }
            else if (map_x >= map_shape_[1]) {
                int expand = map_x - map_shape_[1] + 1;
                map_shape_[1] += expand;
            }
        
            if (map_y < 0) {
                float expand = -map_y * map_resolution_;
                map_origin_[1] -= expand;
                map_shape_[0] += -map_y;
            }
            else if (map_y >= map_shape_[0]) {
                int expand = map_y - map_shape_[0] + 1;
                map_shape_[0] += expand;
            }
        }

        std::pair<float, float> CoreSLAM::get_map_center() const {
            return {
                map_origin_[0] + (map_shape_[0] * map_resolution_ / 2.0f),
                map_origin_[1] + (map_shape_[1] * map_resolution_ / 2.0f)
            };
        }
    } // namespace SLAM
} // namespace puzzlebot_navigation

