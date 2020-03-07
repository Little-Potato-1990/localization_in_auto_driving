/*
 * @Description: 订阅 key frame 数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "lidar_localization/subscriber/key_frames_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_localization{
KeyFramesSubscriber::KeyFramesSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &KeyFramesSubscriber::msg_callback, this);
}

void KeyFramesSubscriber::msg_callback(const nav_msgs::Path::ConstPtr& key_frames_msg_ptr) {
    buff_mutex_.lock();
    new_key_frames_.clear();

    for (size_t i = 0; i < key_frames_msg_ptr->poses.size(); i++) {
        KeyFrame key_frame;
        key_frame.time = key_frames_msg_ptr->poses.at(i).header.stamp.toSec();
        key_frame.index = (unsigned int)i;

        key_frame.pose(0,3) = key_frames_msg_ptr->poses.at(i).pose.position.x;
        key_frame.pose(1,3) = key_frames_msg_ptr->poses.at(i).pose.position.y;
        key_frame.pose(2,3) = key_frames_msg_ptr->poses.at(i).pose.position.z;

        Eigen::Quaternionf q;
        q.x() = key_frames_msg_ptr->poses.at(i).pose.orientation.x;
        q.y() = key_frames_msg_ptr->poses.at(i).pose.orientation.y;
        q.z() = key_frames_msg_ptr->poses.at(i).pose.orientation.z;
        q.w() = key_frames_msg_ptr->poses.at(i).pose.orientation.w;
        key_frame.pose.block<3,3>(0,0) = q.matrix();

        new_key_frames_.push_back(key_frame);
    }
    buff_mutex_.unlock();
}

void KeyFramesSubscriber::ParseData(std::deque<KeyFrame>& key_frames_buff) {
    buff_mutex_.lock();
    if (new_key_frames_.size() > 0) {
        key_frames_buff = new_key_frames_;
        new_key_frames_.clear();
    }
    buff_mutex_.unlock();
}
}