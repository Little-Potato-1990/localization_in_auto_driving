/*
 * @Description: 发送闭环检测的相对位姿
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/loop_pose_publisher.hpp"

#include <Eigen/Dense>
#include "glog/logging.h"

namespace lidar_localization {
LoopPosePublisher::LoopPosePublisher(ros::NodeHandle& nh, 
                                     std::string topic_name, 
                                     std::string frame_id,
                                     int buff_size)
    :nh_(nh), frame_id_(frame_id) {

    publisher_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_name, buff_size);
}

void LoopPosePublisher::Publish(LoopPose& loop_pose) {
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;

    ros::Time ros_time((float)loop_pose.time);
    pose_stamped.header.stamp = ros_time;
    pose_stamped.header.frame_id = frame_id_;

    pose_stamped.pose.pose.position.x = loop_pose.pose(0,3);
    pose_stamped.pose.pose.position.y = loop_pose.pose(1,3);
    pose_stamped.pose.pose.position.z = loop_pose.pose(2,3);

    Eigen::Quaternionf q = loop_pose.GetQuaternion();
    pose_stamped.pose.pose.orientation.x = q.x();
    pose_stamped.pose.pose.orientation.y = q.y();
    pose_stamped.pose.pose.orientation.z = q.z();
    pose_stamped.pose.pose.orientation.w = q.w();

    pose_stamped.pose.covariance[0] = (double)loop_pose.index0;
    pose_stamped.pose.covariance[1] = (double)loop_pose.index1;

    publisher_.publish(pose_stamped);
}

bool LoopPosePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
}