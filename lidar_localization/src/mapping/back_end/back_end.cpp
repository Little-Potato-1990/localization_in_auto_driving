/*
 * @Description: back end 具体实现
 * @Author: Ren Qian
 * @Date: 2020-02-28 01:02:51
 */
#include "lidar_localization/mapping/back_end/back_end.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tools/file_manager.hpp"

namespace lidar_localization {
BackEnd::BackEnd() {
    InitWithConfig();
}

bool BackEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/back_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    InitParam(config_node);
    InitDataPath(config_node);

    return true;
}

bool BackEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    optimize_step_with_none_ = config_node["optimize_step_with_none"].as<int>();
    optimize_step_with_gnss_ = config_node["optimize_step_with_gnss"].as<int>();
    optimize_step_with_loop_ = config_node["optimize_step_with_loop"].as<int>();

    return true;
}

bool BackEnd::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    key_frames_path_ = data_path + "/slam_data/key_frames";
    trajectory_path_ = data_path + "/slam_data/trajectory";

    if (!FileManager::InitDirectory(key_frames_path_, "关键帧点云"))
        return false;
    if (!FileManager::InitDirectory(trajectory_path_, "轨迹文件"))
        return false;

    if (!FileManager::CreateFile(ground_truth_ofs_, trajectory_path_ + "/ground_truth.txt"))
        return false;
    if (!FileManager::CreateFile(laser_odom_ofs_, trajectory_path_ + "/laser_odom.txt"))
        return false;

    return true;
}

bool BackEnd::Update(const CloudData& cloud_data, const PoseData& laser_odom, const PoseData& gnss_pose) {
    ResetParam();

    SaveTrajectory(laser_odom, gnss_pose);

    if (MaybeNewKeyFrame(cloud_data, laser_odom)) {
        MaybeOptimized();
    }

    return true;
}

void BackEnd::ResetParam() {
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool BackEnd::SaveTrajectory(const PoseData& laser_odom, const PoseData& gnss_pose) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {

            ground_truth_ofs_ << gnss_pose.pose(i, j);
            laser_odom_ofs_ << laser_odom.pose(i, j);
            
            if (i == 2 && j == 3) {
                ground_truth_ofs_ << std::endl;
                laser_odom_ofs_ << std::endl;
            } else {
                ground_truth_ofs_ << " ";
                laser_odom_ofs_ << " ";
            }
        }
    }

    return true;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData& cloud_data, const PoseData& laser_odom) {
    if (key_frames_deque_.size() == 0) {
        has_new_key_frame_ = true;
        last_key_pose_ = laser_odom.pose;
    }

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(laser_odom.pose(0,3) - last_key_pose_(0,3)) + 
        fabs(laser_odom.pose(1,3) - last_key_pose_(1,3)) +
        fabs(laser_odom.pose(2,3) - last_key_pose_(2,3)) > key_frame_distance_) {

        has_new_key_frame_ = true;
        last_key_pose_ = laser_odom.pose;
    }

    if (has_new_key_frame_) {
        // 把关键帧点云存储到硬盘里
        std::string file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames_deque_.size()) + ".pcd";
        pcl::io::savePCDFileBinary(file_path, *cloud_data.cloud_ptr);

        KeyFrame key_frame;
        key_frame.time = laser_odom.time;
        key_frame.index = (unsigned int)key_frames_deque_.size();
        key_frame.pose = laser_odom.pose;
        key_frames_deque_.push_back(key_frame);

        latest_key_frame_ = key_frame;

    }

    return has_new_key_frame_;
}

bool BackEnd::MaybeOptimized() {
    static int unoptimized_cnt = 0;

    if (++unoptimized_cnt > optimize_step_with_none_) {
        unoptimized_cnt = 0;
        has_new_optimized_ = true;
    }

    return true;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque) {
    key_frames_deque = key_frames_deque_;
}

bool BackEnd::HasNewKeyFrame() {
    return has_new_key_frame_;
}

bool BackEnd::HasNewOptimized() {
    return has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrame& key_frame) {
    key_frame = latest_key_frame_;
}
}