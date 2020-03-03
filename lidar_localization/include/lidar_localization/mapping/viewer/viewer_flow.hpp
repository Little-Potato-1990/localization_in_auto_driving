/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-29 03:32:14
 */
#ifndef LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/odometry_subscriber.hpp"
#include "lidar_localization/subscriber/key_frame_subscriber.hpp"
#include "lidar_localization/subscriber/key_frames_subscriber.hpp"
// publisher
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
// viewer
#include "lidar_localization/mapping/viewer/viewer.hpp"

namespace lidar_localization {
class ViewerFlow {
  public:
    ViewerFlow(ros::NodeHandle& nh, std::string cloud_topic);

    bool Run();
    bool SaveMap();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishGlobalData();
    bool PublishLocalData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
    // publisher
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    // viewer
    std::shared_ptr<Viewer> viewer_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<PoseData> transformed_odom_buff_;
    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> optimized_key_frames_;
    std::deque<KeyFrame> all_key_frames_;

    CloudData current_cloud_data_;
    PoseData current_transformed_odom_;
};
}

#endif