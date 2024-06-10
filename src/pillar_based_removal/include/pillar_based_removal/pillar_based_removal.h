#ifndef PILLAR_BASED_REMOVAL_H
#define PILLAR_BASED_REMOVAL_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PillarBasedRemoval : public rclcpp:Node {

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;

    sensor_msgs::msg::PointCloud2 received_point_cloud_;
    sensor_msgs::msg::PointCloud2 target_point_cloud_;

public:
    PillarBasedRemoval();
    ~PillarBasedRemoval();

    

}

#endif