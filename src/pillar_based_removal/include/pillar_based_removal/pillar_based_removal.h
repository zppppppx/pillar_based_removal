#ifndef PILLAR_BASED_REMOVAL_H
#define PILLAR_BASED_REMOVAL_H

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// voxelization lib
#include <spconvlib/spconv/csrc/sparse/all/ops3d/Point2Voxel.h>
#include <spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU.h>

// customized point cloud type
#include "pillar_based_removal/point_cloud_type.h"

// tensor view
#include <spconvlib/cumm/common/TensorView.h>

class PillarBasedRemoval : public rclcpp::Node {

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    std::string subscription_name_;

    // sensor_msgs::msg::PointCloud2 received_point_cloud_;
    // data variables
    PointCloudT::Ptr received_point_cloud_;
    sensor_msgs::msg::PointCloud2 target_point_cloud_;
    tv::Tensor point_cloud_tensor_;
    size_t num_received_points_;
    size_t num_send_points_;

    // parameters
    std::string device_;
    int device_num_ = -1;
    float resolution_;
    int max_num_pillars_;
    int num_features_;
    std::vector<float> lidar_ranges_;
    float environment_radius_;
    std::vector<float> rebuild_radiuses_;
    std::vector<float> range_split_;

    
    // A helper function to declare parameters.
    void declare_param(const std::string &name, const std::string &default_value, const std::string &description);
    // Set necessary parameters.
    void set_params();
    // Convert the PointCloud2 message to tensorview tensor.
    void msgToTensor(const sensor_msgs::msg::PointCloud2 &point_cloud);

public:
    PillarBasedRemoval();
    ~PillarBasedRemoval();

    void Callback(const sensor_msgs::msg::PointCloud2);
}

#endif