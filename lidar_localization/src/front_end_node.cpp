/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/subscriber/imu_subscriber.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/odometry_publisher.hpp"
#include "lidar_localization/front_end/front_end.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    std::shared_ptr<CloudPublisher> local_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    std::shared_ptr<CloudPublisher> global_map_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

    std::shared_ptr<FrontEnd> front_end_ptr = std::make_shared<FrontEnd>();

    std::deque<CloudData> cloud_data_buff;
    std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool gnss_origin_position_inited = false;
    bool front_end_pose_inited = false;
    CloudData::CLOUD_PTR local_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR current_scan_ptr(new CloudData::CLOUD());
    
    double run_time = 0.0;
    double init_time = 0.0;
    bool time_inited = false;
    bool has_global_map_published = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
            }
        } else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                if (!time_inited) {
                    time_inited = true;
                    init_time = cloud_data.time;
                } else {
                    run_time = cloud_data.time - init_time;
                }

                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ();
                    odometry_matrix(0,3) = gnss_data.local_E;
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
                    odometry_matrix *= lidar_to_imu;
                    gnss_pub_ptr->Publish(odometry_matrix);

                    if (!front_end_pose_inited) {
                        front_end_pose_inited = true;
                        front_end_ptr->SetInitPose(odometry_matrix);
                    }
                    front_end_ptr->SetPredictPose(odometry_matrix);
                    Eigen::Matrix4f laser_matrix = front_end_ptr->Update(cloud_data);
                    laser_odom_pub_ptr->Publish(laser_matrix);

                    front_end_ptr->GetCurrentScan(current_scan_ptr);
                    cloud_pub_ptr->Publish(current_scan_ptr);
                    if (front_end_ptr->GetNewLocalMap(local_map_ptr))
                        local_map_pub_ptr->Publish(local_map_ptr);
                }
                if (run_time > 460.0 && !has_global_map_published) {
                    if (front_end_ptr->GetNewGlobalMap(global_map_ptr)) {
                        global_map_pub_ptr->Publish(global_map_ptr);
                        has_global_map_published = true;
                    }
                }
            }
        }

        rate.sleep();
    }

    return 0;
}