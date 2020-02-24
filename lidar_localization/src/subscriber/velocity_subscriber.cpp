/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-06-14 16:44:18
 */
#include "lidar_localization/subscriber/velocity_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization{
VelocitySubscriber::VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &VelocitySubscriber::msg_callback, this);
}

void VelocitySubscriber::msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr) {
    VelocityData velocity_data;
    velocity_data.time = twist_msg_ptr->header.stamp.toSec();

    velocity_data.linear_velocity.x = twist_msg_ptr->twist.linear.x;
    velocity_data.linear_velocity.y = twist_msg_ptr->twist.linear.y;
    velocity_data.linear_velocity.z = twist_msg_ptr->twist.linear.z;

    velocity_data.angular_velocity.x = twist_msg_ptr->twist.angular.x;
    velocity_data.angular_velocity.y = twist_msg_ptr->twist.angular.y;
    velocity_data.angular_velocity.z = twist_msg_ptr->twist.angular.z;

    new_velocity_data_.push_back(velocity_data);
}

void VelocitySubscriber::ParseData(std::deque<VelocityData>& velocity_data_buff) {
    if (new_velocity_data_.size() > 0) {
        velocity_data_buff.insert(velocity_data_buff.end(), new_velocity_data_.begin(), new_velocity_data_.end());
        new_velocity_data_.clear();
    }
}
}