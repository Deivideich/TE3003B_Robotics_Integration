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
                map_origin_ = {-5.0f, -5.0f};  // 5m buffer in each direction
                map_shape_ = {10, 10};         // 10x10m initial size
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


        void CoreSLAM::updateMapParams(){
            if (recorded_data.empty() || recorded_data.size() != 4) {
                std::cerr << "Error: recorded_data does not comply especifications!" << std::endl;
                return;
            } 
            float xmin, ymin, xmax, ymax;
            xmin = recorded_data[0];
            ymin = recorded_data[1];
            xmax = recorded_data[2];
            ymax = recorded_data[3];

            map_origin_ = {xmin * map_resolution_, ymin * map_resolution_};
            map_shape_ = {int((ymax - ymin) / map_resolution_) + 1, int((xmax - xmin) / map_resolution_) + 1};

            std::cout << "Map parameters updated: "
                << "origin=(" << map_origin_[0] << ", " << map_origin_[1] << "), "
                << "shape=(" << map_shape_[0] << ", " << map_shape_[1] << ")" << std::endl;
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

                // float neff = 1.0 / sum_squared_weights;
                // if (neff > num_particles_ / 2.0) {
                //     std::cout << "Effective sample size is low, resampling..." << std::endl;
                // } else {
                //     std::cout << "Effective sample size is sufficient, no resampling needed." << std::endl;
                //     return false;
                // }

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
                // main_map_->clear();
                // for (const int id : selected_particle_ids) {
                //     const auto& particle_cells = particle_map_[id];
                //     if (!particle_cells) continue;

                //     for (const auto& coord : *particle_cells) {
                //         main_map_->insert(coord);
                //         recorded_data[0] = std::min(recorded_data[0], coord.second.second);
                //         recorded_data[1] = std::min(recorded_data[1], coord.second.first);
                //         recorded_data[2] = std::max(recorded_data[2], coord.second.second);
                //         recorded_data[3] = std::max(recorded_data[3], coord.second.first);
                //     }
                // }

                // // Clear the particle_map_ to avoid memory leaks
                // for (const auto& pair : particle_map_) {
                //     pair.second->clear();
                // }
                // particle_map_.clear();
                
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

                return true;
            } catch (const std::exception& e) {
                std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
                return false;
            }
        }

        // bool CoreSLAM::initial_guess(
        //     std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
        //     int scan_size, float max_range)
        // {
        //     try
        //     {
        //         if (!particles_) return false;

        //         // Assume initial pose is (0, 0, 0)
        //         auto& particle = (*particles_)[0];
        //         particle = std::make_tuple(0.0f, 0.0f, 0.0f);

        //         for (int j = 0; j < scan_size; ++j){
        //             float ray_angle = (*scan_angles)[j];
        //             float range = (*scan_ranges)[j];
        //             std::cout << "Range: " << range << " Angle: " <<ray_angle << std::endl;
        //             if (range >= max_range || range < 0.0f) continue;

        //             float beam_x = range * std::cos(ray_angle);
        //             float beam_y = range * std::sin(ray_angle);

        //             int map_x = static_cast<int>(beam_x / map_resolution_);
        //             int map_y = static_cast<int>(beam_y / map_resolution_);
        //             main_map_->insert({{map_x, map_y}, {beam_x, beam_y}});
        //             std::cout << "Inserted map: " << map_x << ", " << map_y << " beam: " << beam_x << ", " << beam_y << std::endl;

        //             recorded_data[0] = std::min(recorded_data[0], beam_x);
        //             recorded_data[1] = std::min(recorded_data[1], beam_y);
        //             recorded_data[2] = std::max(recorded_data[2], beam_x);
        //             recorded_data[3] = std::max(recorded_data[3], beam_y);
        //         }
        //         std::cout << "Initial guess done. Main map size: " << main_map_->size() << std::endl;  // Check size

        //         return true;
        //     }
        //     catch(const std::exception& e)
        //     {
        //         std::cerr << e.what() << '\n';
        //         return false;
        //     }
        // }

        std::pair<int, int> CoreSLAM::worldToMap(float x, float y) const {
            return {
                static_cast<int>((x - map_origin_[0]) / map_resolution_),
                static_cast<int>((y - map_origin_[1]) / map_resolution_)
            };
        }
        
        std::pair<float, float> CoreSLAM::mapToWorld(int x, int y) const {
            return {
                map_origin_[0] + x * map_resolution_,
                map_origin_[1] + y * map_resolution_
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
                map_origin_[0] + (map_shape_[1] * map_resolution_ / 2.0f),
                map_origin_[1] + (map_shape_[0] * map_resolution_ / 2.0f)
            };
        }

        // bool CoreSLAM::initial_guess(
        //     std::shared_ptr<std::vector<float>> scan_angles, 
        //     std::shared_ptr<std::vector<float>> scan_ranges,
        //     int scan_size, float max_range)
        // {
        //     try {
        //         if (!particles_) return false;
                
        //         // Initialize particles with some uncertainty around (0,0,0)
        //         std::mt19937 gen(std::random_device{}());
        //         std::normal_distribution<float> trans_dist(0.0f, 0.1f); // 10cm std dev
        //         std::normal_distribution<float> rot_dist(0.0f, 0.05f);  // ~3deg std dev
        
        //         // Temporary map to accumulate observations from all particles
        //         std::unordered_map<std::pair<int,int>, int, hashFunction> temp_map;

        //         // Initialize particles around (0,0)
        //         for (int i = 0; i < num_particles_; i++) {
        //             float x = trans_dist(gen);
        //             float y = trans_dist(gen);
        //             float theta = rot_dist(gen);
        //             (*particles_)[i] = {x, y, theta};
        //         }

        //         // Calculate proper map bounds including negative coordinates
        //         float min_x = 0, max_x = 0, min_y = 0, max_y = 0;
        //         for (int j = 0; j < scan_size; ++j) {
        //             float range = (*scan_ranges)[j];
        //             if (range >= max_range || range < 0.0f) continue;
                    
        //             float beam_x = range * std::cos((*scan_angles)[j]);
        //             float beam_y = range * std::sin((*scan_angles)[j]);
                    
        //             min_x = std::min(min_x, beam_x);
        //             max_x = std::max(max_x, beam_x);
        //             min_y = std::min(min_y, beam_y);
        //             max_y = std::max(max_y, beam_y);
        //         }

        //         // Set map origin to encompass all points with some margin
        //         map_origin_ = {min_x - 1.0f, min_y - 1.0f};  // 1 meter margin
        //         map_shape_ = {
        //             static_cast<int>((max_y - min_y + 2.0f) / map_resolution_),
        //             static_cast<int>((max_x - min_x + 2.0f) / map_resolution_)
        //         };
                
                
        //         // Transform scan to map frame for this particle
        //         for (int j = 0; j < scan_size; ++j) {
        //             float ray_angle = (*scan_angles)[j];
        //             float range = (*scan_ranges)[j];
                    
        //             if (range >= max_range || range < 0.0f) continue;
    
        //             float beam_x = x + range * std::cos(theta + ray_angle);
        //             float beam_y = y + range * std::sin(theta + ray_angle);
    
        //             auto [map_x, map_y] = worldToMap(beam_x, beam_y);
                    
        //             // Vote for this cell
        //             temp_map[{map_x, map_y}]++;
        //         }
        
        //         // Only keep cells observed by multiple particles
        //         const int vote_threshold = num_particles_ / 5; // 20% agreement
        //         for (const auto& [cell, votes] : temp_map) {
        //             if (votes > vote_threshold) {
        //                 float beam_x = cell.first * map_resolution_;
        //                 float beam_y = cell.second * map_resolution_;
        //                 main_map_->insert({cell, {beam_x, beam_y}});
                        
        //                 // Update map bounds
        //                 recorded_data[0] = std::min(recorded_data[0], beam_x);
        //                 recorded_data[1] = std::min(recorded_data[1], beam_y);
        //                 recorded_data[2] = std::max(recorded_data[2], beam_x);
        //                 recorded_data[3] = std::max(recorded_data[3], beam_y);
        //             }
        //         }
        
        //         return true;
        //     } catch(const std::exception& e){
        //         std::cerr << e.what() << '\n';
        //     }
        // }

        bool CoreSLAM::initial_guess(
            std::shared_ptr<std::vector<float>> scan_angles,
            std::shared_ptr<std::vector<float>> scan_ranges,
            int scan_size, float max_range) 
        {
            try {
                if (!particles_) return false;
        
                // Initialize particles with small noise around (0,0)
                std::mt19937 gen(std::random_device{}());
                std::normal_distribution<float> trans_dist(0.0f, 0.1f);
                std::normal_distribution<float> rot_dist(0.0f, 0.05f);
        
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
                }
        
                // Set map origin with margin
                float margin = 1.0f; // 1 meter margin
                map_origin_ = {
                    recorded_data[0] - margin,
                    recorded_data[1] - margin
                };
                map_shape_ = {
                    static_cast<int>((recorded_data[3] - recorded_data[1] + 2*margin) / map_resolution_),
                    static_cast<int>((recorded_data[2] - recorded_data[0] + 2*margin) / map_resolution_)
                };
        
                // Second pass: process scans with proper coordinates
                for (int i = 0; i < num_particles_; i++) {
                    float x = trans_dist(gen);
                    float y = trans_dist(gen);
                    float theta = rot_dist(gen);
                    (*particles_)[i] = {x, y, theta};
        
                    for (int j = 0; j < scan_size; ++j) {
                        float range = (*scan_ranges)[j];
                        if (range >= max_range || range < 0.0f) continue;
        
                        float beam_x = x + range * std::cos(theta + (*scan_angles)[j]);
                        float beam_y = y + range * std::sin(theta + (*scan_angles)[j]);
        
                        auto [map_x, map_y] = worldToMap(beam_x, beam_y);
                        main_map_->insert({{map_x, map_y}, {beam_x, beam_y}});
                    }
                }
        
                return true;
            } catch (...) {
                return false;
            }
        }
        
        
        // bool CoreSLAM::weight_slam_particles(
        //     std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
        //     int scan_size, float max_range, std::shared_ptr<Particle> max_particle){
        //     try{
        //         if (!main_map_){ 
        //             std::cerr << "Main map is not initialized!" << std::endl;
        //             return false;
        //         } 
        //         if (main_map_->empty()){
        //             std::cerr << "Main map is empty!" << std::endl;
        //             return false;
        //         }

        //         float origin_x = map_origin_[0];
        //         float origin_y = map_origin_[1];

        //         int map_height = map_shape_[0];
        //         int map_width = map_shape_[1];
        //         float max_score = 0.0;


        //         for (size_t i = 0; i < num_particles_; i++){
        //             const auto& particle = (*particles_)[i];
        //             auto [x, y, theta] = particle;
        //             std::cout << "Particle: " << i << " X: " << x << " Y: " << y << " Theta: " << theta << std::endl;
        //             float particle_weight = 0.0;
        //             particle_map_[i] = std::make_shared<uset_pair>();
        //             // For each particle calculate the weight based on the laser scan hits and map occupancy grid
        //             for (size_t j = 0; j < scan_size; j++){
        //                 float angle = (*scan_angles)[j];
        //                 float range = (*scan_ranges)[j];

        //                 if (range >= max_range || range < 0.0) continue;
        //                 float ray_angle = theta + angle;
        //                 // Normalize angle to [-π, π]
        //                 while (ray_angle > M_PI)
        //                 ray_angle -= 2.0f * M_PI;
        //                 while (ray_angle < -M_PI)
        //                 ray_angle += 2.0f * M_PI;
        //                 // Calculate the beam endpoint accding to the particle pose
        //                 float beam_x = x + range * std::cos(ray_angle);
        //                 float beam_y = y + range * std::sin(ray_angle);

        //                 // Check if the beam is within the map bounds
        //                 int map_x = static_cast<int>((beam_x - origin_x) / map_resolution_);
        //                 int map_y = static_cast<int>((beam_y - origin_y) / map_resolution_);                        
        //                 // Check value of the map cell 
        //                 particle_weight += main_map_->find({map_y, map_x}) != main_map_->end() ? 1.0 : 0.0;
        //                 particle_map_[i]->insert({{map_x, map_y}, {beam_x, beam_y}});
        //             }
                    
        //             // std::cout << "Particle: " << i << " Weight: " << particle_weight << std::endl;
        //             (*weights_)[i] = particle_weight;
        //         }

        //         // Normalize weights_
        //         float sum = 0.0;
        //         for (size_t i = 0; i < num_particles_; i++){
        //             sum += (*weights_)[i];
        //         }

        //         int max_index = 0;
        //         if (sum > 0.0){
        //             for (size_t i = 0; i < num_particles_; i++){
        //                 (*weights_)[i] /= sum;

        //                 // Find the maximum weight and corresponding particle
        //                 if ((*weights_)[i] > max_score){
        //                     max_score = (*weights_)[i];
        //                     max_index = i;
        //                     const auto& particle = (*particles_)[i];
        //                     auto [x, y, theta] = particle;
        //                     max_particle = std::make_shared<Particle>(x, y, theta);
        //                 }
        //             }

        //             main_map_->clear();
        //             const auto& particle_cells = particle_map_[max_index];
        //             if (!particle_cells) return false;

        //             for (const auto& coord : *particle_cells) {
        //                 main_map_->insert(coord);
        //                 recorded_data[0] = std::min(recorded_data[0], coord.second.second);
        //                 recorded_data[1] = std::min(recorded_data[1], coord.second.first);
        //                 recorded_data[2] = std::max(recorded_data[2], coord.second.second);
        //                 recorded_data[3] = std::max(recorded_data[3], coord.second.first);
        //             }
        //             std::cout << "{" << recorded_data[0] << ", " << recorded_data[1] << ", " 
        //                 << recorded_data[2] << ", " << recorded_data[3] << "}" << std::endl;

        //             // Clear the particle_map_ to avoid memory leaks
        //             for (const auto& pair : particle_map_) {
        //                 pair.second->clear();
        //             }
        //             particle_map_.clear();

                    
        //         } else {
        //             std::cout << "Warning: All weights are zero!" << std::endl;
        //             for (size_t i = 0; i < num_particles_; i++){
        //                 (*weights_)[i] = 1.0 / num_particles_;
        //             }
        //         }

        //         return true;
        //     } catch (const std::exception& e) {
        //         std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
        //         return false;
        //     }
        // }
        bool CoreSLAM::weight_slam_particles(
            std::shared_ptr<std::vector<float>> scan_angles,
            std::shared_ptr<std::vector<float>> scan_ranges,
            int scan_size, float max_range,
            std::shared_ptr<Particle> max_particle)
        {
            try {
                if (!main_map_ || main_map_->empty()) return false;
        
                // Probabilistic map representation
                struct MapCell {
                    float occupancy = 0.5f; // 0.5 = unknown
                    int observations = 0;
                };
                static std::unordered_map<std::pair<int,int>, MapCell, hashFunction> prob_map;
        
                // Parameters for inverse sensor model
                const float hit_prob = 0.7f;
                const float miss_prob = 0.4f;
                const float clamp_min = 0.1f;
                const float clamp_max = 0.9f;
        
                float max_score = -1.0f;
                int best_particle_idx = 0;
                float origin_x = map_origin_[0];
                float origin_y = map_origin_[1];
        
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
        
                        // Map coordinates
                        auto [map_x, map_y] = worldToMap(beam_x, beam_y);
        
                        // Score based on probabilistic map if available
                        if (prob_map.find({map_y, map_x}) != prob_map.end()) {
                            particle_weight += prob_map[{map_y, map_x}].occupancy;
                        } else {
                            // Fallback to binary occupancy
                            particle_weight += (main_map_->find({map_y, map_x}) != main_map_->end()) ? 1.0 : 0.0;
                        }
        
                        // Store observation for map update
                        particle_map_[i]->insert({{map_x, map_y}, {beam_x, beam_y}});
                    }
        
                    (*weights_)[i] = particle_weight;
                    if (particle_weight > max_score) {
                        max_score = particle_weight;
                        best_particle_idx = i;
                        *max_particle = (*particles_)[i];
                    }
                }
        
                // Second pass: update probabilistic map using best particles
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
                
                // Update map with observations from good particles
                // Now use the first top_n indices
                for (size_t i = 0; i < std::min(top_n, num_particles_); ++i) {
                    size_t idx = indices[i];
                    // Safe to access particle_map_[idx] now
                    const auto& observations = particle_map_[idx];
                    for (const auto& [cell, coords] : *observations) {
                        auto& map_cell = prob_map[cell];
                        map_cell.occupancy = (map_cell.occupancy * map_cell.observations + hit_prob) / 
                                            (map_cell.observations + 1);
                        map_cell.observations++;
                        map_cell.occupancy = std::max(clamp_min, std::min(map_cell.occupancy, clamp_max));
                    }
                }
                
                std::cout << "Debug: "<< std::endl;  // Check size
                // Update main_map_ from probabilistic map
                if (!main_map_) {
                    main_map_ = std::make_shared<uset_pair>();
                } else {
                    main_map_->clear();
                }

                auto new_main_map = std::make_shared<uset_pair>();
                std::vector<float> new_recorded_data = {
                    INFINITY, INFINITY, -INFINITY, -INFINITY
                };
            
                for (size_t i = 0; i < std::min(top_n, num_particles_); ++i) {
                    size_t idx = indices[i];
                    const auto& observations = particle_map_[idx];
                    for (const auto& [cell, coords] : *observations) {
                        new_main_map->insert({cell, coords});
                        
                        // Update recorded data with world coordinates
                        new_recorded_data[0] = std::min(new_recorded_data[0], coords.first);
                        new_recorded_data[1] = std::min(new_recorded_data[1], coords.second);
                        new_recorded_data[2] = std::max(new_recorded_data[2], coords.first);
                        new_recorded_data[3] = std::max(new_recorded_data[3], coords.second);
                    }
                }
            
                // Check if we need to expand the map
                float expand_threshold = 0.5f; // meters
                bool need_expand = false;
                
                // Check all boundaries
                if (new_recorded_data[0] < map_origin_[0] + expand_threshold) {
                    need_expand = true;
                    float expand = map_origin_[0] - new_recorded_data[0] + expand_threshold;
                    map_origin_[0] -= expand;
                    map_shape_[1] += static_cast<int>(expand / map_resolution_);
                }
                // Similar checks for other boundaries...
            
                // Apply updates
                main_map_ = new_main_map;
                recorded_data = new_recorded_data;
        
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
    } // namespace SLAM
} // namespace puzzlebot_navigation

