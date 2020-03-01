/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:35:19
 */

#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_INTERFACE_GRAPH_OPTIMIZER_HPP_

#include <string>
#include <deque>
#include <Eigen/Dense>

namespace lidar_localization {
class InterfaceGraphOptimizer {
  public:
    virtual ~InterfaceGraphOptimizer() {}
    // 优化
    virtual bool Optimize() = 0;
    // 输入、输出数据
    virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f>& optimized_pose) = 0;
    virtual int GetNodeNum() = 0;
    // 添加节点、边、鲁棒核
    virtual void SetEdgeRobustKernel(std::string robust_kernel_name, double robust_kernel_size) = 0;
    virtual void AddSe3Node(const Eigen::Isometry3d &pose, bool need_fix) = 0;
    virtual void AddSe3Edge(int vertex_index1,
                            int vertex_index2,
                            const Eigen::Isometry3d &relative_pose,
                            const Eigen::VectorXd noise) = 0;
    virtual void AddSe3PriorXYZEdge(int se3_vertex_index,
                                    const Eigen::Vector3d &xyz,
                                    Eigen::VectorXd noise) = 0;
    virtual void AddSe3PriorQuaternionEdge(int se3_vertex_index,
                                           const Eigen::Quaterniond &quat,
                                           Eigen::VectorXd noise) = 0;
    // 设置优化参数
    void SetMaxIterationsNum(int max_iterations_num);
  
  protected:
    int max_iterations_num_ = 512;
};
} // namespace lidar_localization
#endif