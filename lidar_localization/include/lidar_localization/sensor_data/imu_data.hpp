/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:27:40
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>

namespace lidar_localization {
class IMUData {
  public:
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
    
    struct Orientation {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double w = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
  
  public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix() {
      Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
      Eigen::Matrix3f matrix = q.matrix().cast<float>();

      return matrix;
    }
};
}
#endif