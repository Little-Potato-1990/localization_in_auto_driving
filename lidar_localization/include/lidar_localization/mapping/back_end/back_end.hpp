/*
 * @Description: back end 具体实现
 * @Author: Ren Qian
 * @Date: 2020-02-28 01:01:00
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_BACK_END_HPP_

#include <string>
#include <deque>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/sensor_data/pose_data.hpp"
#include "lidar_localization/sensor_data/key_frame.hpp"

#include "lidar_localization/models/graph_optimizer/g2o/g2o_graph_optimizer.hpp"

namespace lidar_localization {
class BackEnd {
  public:
    BackEnd();

    bool Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose);

    bool ForceOptimize();
    void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);
    bool HasNewKeyFrame();
    bool HasNewOptimized();
    void GetLatestKeyFrame(KeyFrame& key_frame);
  
  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitGraphOptimizer(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);

    void ResetParam();
    bool SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose);
    bool AddNodeAndEdge(const PoseData& gnss_data);
    bool MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom);
    bool MaybeOptimized();

  private:
    std::string key_frames_path_ = "";
    std::string trajectory_path_ = "";

    std::ofstream ground_truth_ofs_;
    std::ofstream laser_odom_ofs_;

    float key_frame_distance_ = 2.0;

    bool has_new_key_frame_ = false;
    bool has_new_optimized_ = false;

    KeyFrame current_key_frame_;
    std::deque<KeyFrame> key_frames_deque_;

    // 优化器
    std::shared_ptr<InterfaceGraphOptimizer> graph_optimizer_ptr_;
    
    class GraphOptimizerConfig {
      public:
        GraphOptimizerConfig() {
          odom_edge_noise.resize(6);
          close_loop_noise.resize(6);
          gnss_noise.resize(3);
        }
      
      public:
        bool use_gnss = true;
        bool use_loop_close = false;

        Eigen::VectorXd odom_edge_noise;
        Eigen::VectorXd close_loop_noise;
        Eigen::VectorXd gnss_noise;

        int optimize_step_with_key_frame = 100;
        int optimize_step_with_gnss = 100;
        int optimize_step_with_loop = 10;
    };
    GraphOptimizerConfig graph_optimizer_config_;

    int new_gnss_cnt_ = 0;
    int new_loop_cnt_ = 0;
    int new_key_frame_cnt_ = 0;
};
}

#endif