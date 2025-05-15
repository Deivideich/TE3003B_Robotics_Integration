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
        struct hashFunction
        {
            size_t operator()(const std::pair<int,int> &key) const
            {
                auto [x, y] = key;
                return std::hash<int>()(x) ^ std::hash<int>()(y);
            }
        };

        typedef std::unordered_map<std::pair<int,int>, std::pair<float,float>, hashFunction> uset_pair;
        
        class CoreSLAM
        {
            private:
                // PARTICLEs
                std::shared_ptr<std::vector<Particle>> particles_;
                std::shared_ptr<std::vector<float>> weights_;
                int num_particles_;
                int num_dimensions_;
                float theta_noise_;
                float trans_noise_;
                float map_resolution_;

                // MAPs
                std::shared_ptr<uset_pair> main_map_;
                std::unordered_map<int, std::shared_ptr<uset_pair>> particle_map_;
                std::vector<float> map_origin_;
                std::vector<int> map_shape_;
                std::vector<float> recorded_data = {
                    std::numeric_limits<float>::max(), 
                    std::numeric_limits<float>::max(), 
                    std::numeric_limits<float>::lowest(), 
                    std::numeric_limits<float>::lowest()
                }; // xmin, ymin, xmax, ymax

            public:
                CoreSLAM(
                    int num_particles,
                    int num_dimensions,
                    float theta_noise,
                    float trans_noise);

                CoreSLAM();

                void initialize_particles();
                
                void updateMapParams();

                std::pair<int,int> worldToMap(float x, float y) const;

                std::pair<float,float> mapToWorld(int x, int y) const;

                std::pair<float, float> get_map_center() const;

                void expandMap(float world_x, float world_y);

                void motion_update(float dx, float dy, float dtheta);

                bool initial_guess(
                    std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
                    int scan_size, float max_range,
                    float robot_x, float robot_y, float robot_theta); 
                    
                bool weight_slam_particles(
                    std::shared_ptr<std::vector<float>> scan_angles, std::shared_ptr<std::vector<float>> scan_ranges, 
                    int scan_size, float max_range, std::shared_ptr<Particle> max_particle);
                
                bool resample_particles();

                std::shared_ptr<std::vector<Particle>> get_particles() const {
                    return particles_;
                }
        
                std::shared_ptr<uset_pair> get_main_map() const {
                    return main_map_;
                }
        
                std::vector<float> get_map_origin() const {
                    return map_origin_;
                }
        
                std::vector<int> get_map_shape() const {
                    return map_shape_;
                }
        
                float get_map_resolution() const {
                    return map_resolution_;
                }

                std::vector<float> get_recorded_data() const {
                    return recorded_data;
                }

                void set_map_origin(const std::vector<float>& origin) {
                    map_origin_ = origin;
                }
        };
    }        
}

#endif /* CORE_SLAM_H_ */