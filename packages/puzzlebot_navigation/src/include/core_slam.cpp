// #define DEBUG_RESAMPLE
// #define DEBUG_WEIGHT

#include "puzzlebot_navigation/slam/core_slam.h"
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

            map_origin_y_ = ymin;
            map_origin_x_ = xmin;
            map_height_ = int((ymax - ymin) / map_resolution_) + 1;
            map_width_ = int((xmax - xmin) / map_resolution_) + 1;


            std::cout << "Map parameters updated: "
                << "origin=(" << map_origin_y_ << ", " << map_origin_x_ << "), "
                << "shape=(" << map_height_ << ", " << map_width_ << ")" << std::endl;
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
                // std::cout << "Resampling done. Main map size: " << main_map_->size() << std::endl;  // Check size
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

        bool CoreSLAM::initial_guess(
            std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
            int scan_size, float max_range)
        {
            try
            {
                if (!particles_) return false;

                // Assume initial pose is (0, 0, 0)
                auto& particle = (*particles_)[0];
                particle = std::make_tuple(0.0f, 0.0f, 0.0f);

                for (int j = 0; j < scan_size; ++j){
                    float ray_angle = (*scan_angles)[j];
                    float range = (*scan_ranges)[j];
                    // std::cout << "Range: " << range << " Angle: " <<ray_angle << std::endl;
                    if (range >= max_range || range < 0.0f) continue;

                    float beam_x = range * std::cos(ray_angle);
                    float beam_y = range * std::sin(ray_angle);

                    recorded_data[0] = std::min(recorded_data[0], beam_x);
                    recorded_data[1] = std::min(recorded_data[1], beam_y);
                    recorded_data[2] = std::max(recorded_data[2], beam_x);
                    recorded_data[3] = std::max(recorded_data[3], beam_y);
                }

                updateMapParams();


                for (int j = 0; j < scan_size; ++j){
                    float ray_angle = (*scan_angles)[j];
                    float range = (*scan_ranges)[j];
                    // std::cout << "Range: " << range << " Angle: " <<ray_angle << std::endl;
                    if (range >= max_range || range < 0.0f) continue;

                    float beam_x = range * std::cos(ray_angle);
                    float beam_y = range * std::sin(ray_angle);

                    auto [map_y, map_x] = worldToMap(beam_y, beam_x);

                    if (map_y >= map_height_ || map_y < 0 || map_x >= map_width_ || map_x < 0) continue;

                    main_map_->insert({{map_y, map_x},{beam_y, beam_x}});
                }

                std::cout << "Initial guess done. Main map size: " << main_map_->size() << std::endl;  // Check size

                return true;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                return false;
            }
        }
        
        
        bool CoreSLAM::weight_slam_particles(
            std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
            int scan_size, float max_range, std::shared_ptr<Particle> max_particle){
            try{
                if (!main_map_){ 
                    std::cerr << "Main map is not initialized!" << std::endl;
                    return false;
                } 
                if (main_map_->empty()){
                    std::cerr << "Main map is empty!" << std::endl;
                    return false;
                }

                float max_score = 0.0;

                for (size_t i = 0; i < num_particles_; i++){
                    const auto& particle = (*particles_)[i];
                    auto [x, y, theta] = particle;
                    // std::cout << "Particle: " << i << " X: " << x << " Y: " << y << " Theta: " << theta << std::endl;
                    float particle_weight = 0.0;
                    particle_map_[i] = std::make_shared<uset_pair>();
                    // For each particle calculate the weight based on the laser scan hits and map occupancy grid
                    for (size_t j = 0; j < scan_size; j++){
                        float angle = (*scan_angles)[j];
                        float range = (*scan_ranges)[j];

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
                        auto [map_y, map_x] = worldToMap(beam_y, beam_x);
                        // Check value of the map cell 
                        particle_weight += main_map_->find({map_y, map_x}) != main_map_->end() ? 1.0 : 0.0;
                        particle_map_[i]->insert({{map_y, map_x}, {beam_y, beam_x}});
                    }
                    
                    // std::cout << "Particle: " << i << " Weight: " << particle_weight << std::endl;
                    (*weights_)[i] = particle_weight;
                }

                // Normalize weights
                float sum = std::accumulate(weights_->begin(), weights_->end(), 0.0f);
                if (sum > 0.0f) {
                    for (auto& w : *weights_) w /= sum;
                } else {
                    std::fill(weights_->begin(), weights_->end(), 1.0f / num_particles_);
                }

                // Get top N%
                const int top_n = std::max(5, num_particles_ / 10);
                std::vector<size_t> indices(num_particles_);
                std::iota(indices.begin(), indices.end(), 0);
                std::partial_sort(
                    indices.begin(),
                    indices.begin() + top_n,
                    indices.end(),
                    [this](size_t a, size_t b) { return (*weights_)[a] > (*weights_)[b]; }
                );

                // Update main_map_ using the top N% particles
                for (int i = 0; i < top_n; ++i) {
                    auto idx = indices[i];
                    const auto& particle_cells = particle_map_[idx];
                    if (!particle_cells) continue;

                    for (const auto& [cell, coords] : *particle_cells) {
                        recorded_data[0] = std::min(recorded_data[0], coords.second);
                        recorded_data[1] = std::min(recorded_data[1], coords.first);
                        recorded_data[2] = std::max(recorded_data[2], coords.second);
                        recorded_data[3] = std::max(recorded_data[3], coords.first);
                    }
                }

                // STEP 4: Update map parameters
                updateMapParams();

                // STEP 5: Backup and realign old main_map_ entries
                uset_pair new_main_map;
                for (const auto& [cell, coords] : *main_map_) {
                    auto [new_y, new_x] = worldToMap(coords.first, coords.second);
                    new_main_map[{new_y, new_x}] = coords;
                }
                main_map_->clear();
                *main_map_ = std::move(new_main_map);

                // STEP 6: Second pass — insert new cells
                for (int i = 0; i < top_n; ++i) {
                    const auto& cells = particle_map_[i];
                    if (!cells) continue;
                    for (const auto& [cell, coords] : *cells) {
                        auto [map_y, map_x] = worldToMap(coords.first, coords.second);
                        if (main_map_->find({map_y, map_x}) == main_map_->end()) {
                            (*main_map_)[{map_y, map_x}] = coords;
                        }
                    }
                }
                return true;
            } catch (const std::exception& e) {
                std::cerr << "Exception in resample_particles: " << e.what() << std::endl;
                return false;
            }
        }

        std::pair<int,int> CoreSLAM::worldToMap(float world_y, float world_x){
            return {
                static_cast<int>((world_y - map_origin_y_) / map_resolution_),
                static_cast<int>((world_x - map_origin_x_) / map_resolution_)
            };
        }
    } // namespace SLAM
} // namespace puzzlebot_navigation

