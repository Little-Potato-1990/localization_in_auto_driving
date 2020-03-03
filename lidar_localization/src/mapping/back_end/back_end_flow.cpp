/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/mapping/back_end/back_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string odom_topic) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 100000);
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, odom_topic, 100000);
    loop_pose_sub_ptr_ = std::make_shared<LoopPoseSubscriber>(nh, "/loop_pose", 100000);

    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/transformed_odom", "/map", "/lidar", 100);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);
    key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "/optimized_key_frames", "/map", 100);

    back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
    if (!ReadData())
        return false;
    
    MaybeInsertLoopPose();

    while(HasData()) {
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool BackEndFlow::ForceOptimize() {
    back_end_ptr_->ForceOptimize();
    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }
    return true;
}

bool BackEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);
    loop_pose_sub_ptr_->ParseData(loop_pose_data_buff_);

    return true;
}

bool BackEndFlow::MaybeInsertLoopPose() {
    while (loop_pose_data_buff_.size() > 0) {
        back_end_ptr_->InsertLoopPose(loop_pose_data_buff_.front());
        loop_pose_data_buff_.pop_front();
    }
    return true;
}

bool BackEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (gnss_pose_data_buff_.size() == 0)
        return false;
    if (laser_odom_data_buff_.size() == 0)
        return false;

    return true;
}

bool BackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();

    double diff_gnss_time = current_cloud_data_.time - current_gnss_pose_data_.time;
    double diff_laser_time = current_cloud_data_.time - current_laser_odom_data_.time;

    if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if (!odometry_inited) {
        odometry_inited = true;
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();
    }
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
}

bool BackEndFlow::PublishData() {
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        KeyFrame key_frame;

        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);

        back_end_ptr_->GetLatestKeyGNSS(key_frame);
        key_gnss_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}
}