#ifndef PILLAR_BASED_REMOVAL_H
#define PILLAR_BASED_REMOVAL_H

#include <chrono>
#include <cmath>
#include <unordered_set>

// ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// voxelization lib
#include <spconvlib/spconv/csrc/sparse/all/ops3d/Point2Voxel.h>
#include <spconvlib/spconv/csrc/sparse/all/ops_cpu3d/Point2VoxelCPU.h>
#include <spconvlib/spconv/csrc/sparse/all/SpconvOps.h>
#include <spconvlib/spconv/csrc/sparse/alloc/StaticAllocator.h>
#include <spconvlib/spconv/csrc/sparse/maxpool/IndiceMaxPoolCPU.h>
#include <spconvlib/spconv/csrc/sparse/convops/gemmops/GemmTunerSimple.h>
#include <spconvlib/spconv/csrc/sparse/convops/spops/ConvGemmOps.h>
#include <spconvlib/cumm/gemm/main/GemmMainUnitTest.h>
#include <spconvlib/spconv/csrc/sparse/convops/SimpleExternalSpconvMatmul.h>

#include <tensorview/io/jsonarray.h>
#include <tensorview/parallel/map.h>
#include <tensorview/contexts/core.h>

// #include <spconvlib/spconv/csrc/sparse/maxpool/IndiceMaxPool.h>

// customized point cloud type
#include "pillar_based_removal/point_cloud_type.h"

// tensor view
#include <spconvlib/cumm/common/TensorView.h>

class PillarBasedRemoval : public rclcpp::Node {
    // Time types
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::milliseconds ms;
    typedef std::chrono::duration<float> fsec;
    using PillarizeGPU3D =
        spconvlib::spconv::csrc::sparse::all::ops3d::Point2Voxel;
    using PillarizeCPU3D =
        spconvlib::spconv::csrc::sparse::all::ops_cpu3d::Point2VoxelCPU;
    using SpconvOps = spconvlib::spconv::csrc::sparse::all::SpconvOps;
    using StaticAllocator = spconvlib::spconv::csrc::sparse::alloc::StaticAllocator;
    using IndiceMaxPoolCPU = spconvlib::spconv::csrc::sparse::maxpool::IndiceMaxPoolCPU;
    using GemmTunerSimple =
        spconvlib::spconv::csrc::sparse::convops::spops::GemmTuner;
    using GemmMain = spconvlib::cumm::gemm::main::GemmMainUnitTest;
    using SimpleExternalSpconvMatmul =
        spconvlib::spconv::csrc::sparse::convops::SimpleExternalSpconvMatmul;
    using ConvGemmOps =
        spconvlib::spconv::csrc::sparse::convops::spops::ConvGemmOps;
private:
    bool verbose_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;
    std::string subscription_name_;
    std::string publisher_name_;

    // sensor_msgs::msg::PointCloud2 received_point_cloud_;
    // data variables
    PointCloudT received_point_cloud_;
    sensor_msgs::msg::PointCloud2 target_point_cloud_msg_;
    size_t num_received_points_;
    size_t num_send_points_;

    tv::Tensor point_cloud_tensor_;
    tv::Tensor kept_pillars_;
    tv::Tensor pillars_;
    tv::Tensor pillars_lowest_;
    tv::Tensor pillars_highest_;
    tv::Tensor environment_lowest_;
    tv::Tensor point_pillar_idx_;
    tv::Tensor pillar_indices_;

    std::vector<int32_t> grid_size_;


    // parameters
    std::string device_;
    int device_num_ = -1;
    double resolution_;
    int max_num_pillars_;
    int num_features_;
    std::vector<double> lidar_ranges_;
    double environment_radius_;
    double env_min_threshold_;
    double max_min_threshold_;
    std::vector<double> rebuild_radiuses_;
    std::vector<double> range_split_;

    
    // A helper function to declare parameters.
    template<class T>
    void declare_param(const std::string &name, const T &default_value, const std::string &description);
    
    // Set necessary parameters.
    void set_params();
    // Convert the PointCloud2 message to tensorview tensor.
    void msgToTensor(const sensor_msgs::msg::PointCloud2 &point_cloud);
    void tensorToMsg();

    /*
    Algorithms implemention
    */
    // Pillarize the point cloud and save the pillars into private variables
    void pillarize();

    void removal_stage();

    void rebuild_stage();

public:
    PillarBasedRemoval();
    ~PillarBasedRemoval();

    void callback(const sensor_msgs::msg::PointCloud2 &received_point_cloud_msg);
};

#endif