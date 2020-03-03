/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:52:45
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_HPP_

#include <deque>
#include <Eigen/Dense>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/models/registration/registration_interface.hpp"
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class FrontEnd {
  public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

  public:
    FrontEnd();

    bool Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);

  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame> local_map_frames_;

    CloudData::CLOUD_PTR local_map_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};
}

#endif