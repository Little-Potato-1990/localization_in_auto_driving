/*
 * @Description: back end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:31:22
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "lidar_localization/subscriber/loop_pose_subscriber.hpp"

#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/key_frame_publisher.hpp"
#include "lidar_localization/publisher/key_frames_publisher.hpp"

#include "lidar_localization/mapping/back_end/back_end.hpp"

namespace lidar_localization {
class BackEndFlow {
  public:
    BackEndFlow(ros::NodeHandle& nh);

    bool Run();

    bool ForceOptimize();

  private:
    bool ReadData();
    bool MaybeInsertLoopPose();
    bool HasData();
    bool ValidData();
    bool UpdateBackEnd();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;
    std::shared_ptr<LoopPoseSubscriber> loop_pose_sub_ptr_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<BackEnd> back_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> gnss_pose_data_buff_;
    std::deque<PoseData> laser_odom_data_buff_;
    std::deque<LoopPose> loop_pose_data_buff_;

    PoseData current_gnss_pose_data_;
    PoseData current_laser_odom_data_;
    CloudData current_cloud_data_;
};
}

#endif