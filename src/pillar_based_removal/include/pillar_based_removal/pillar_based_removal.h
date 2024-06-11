#ifndef PILLAR_BASED_REMOVAL_H
#define PILLAR_BASED_REMOVAL_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <spconvlib/spconv/csrc/sparse/all/ops3d/Point2Voxel.h>
#include <spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU.h>

class PillarBasedRemoval : public rclcpp::Node {

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;

    sensor_msgs::msg::PointCloud2 received_point_cloud_;
    sensor_msgs::msg::PointCloud2 target_point_cloud_;

    std::string device_;
    float resolution_;
    int max_num_pillars_;
    int num_features_;
    std::vector<float> lidar_ranges_;
    float environment_radius_;
    std::vector<float> rebuild_radiuses_;
    std::vector<float> range_split_;
    

    void declare_param(const std::string &name, const std::string &default_value, const std::string &description);
    void set_params();

public:
    PillarBasedRemoval();
    ~PillarBasedRemoval();

    void msgToTensor(const sensor_msgs::msg::PointCloud2 &point_cloud);

}

#endif