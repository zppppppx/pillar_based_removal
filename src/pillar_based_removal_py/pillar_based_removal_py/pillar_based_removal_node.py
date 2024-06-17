import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import torch
import numpy as np
from sensor_msgs.msg import PointCloud2


class PillarBasedRemoval(Node):
    def __init__(self):
        super().__init__("pillar_based_removal_node")
        self.set_params()

        self.subscriber = self.create_subscription(
            PointCloud2,
            self.subscription_name_,
            self.callback,
            1
        )

    
    def declare_param(self, param_name: str, default, description: str):
        descriptor = ParameterDescriptor(description=description)
        self.declare_parameter(param_name, default, descriptor)

    def set_params(self):
        self.declare_param("verbose", False, "If set true, the description of each module will be verbosely printed in the terminal")
        self.verbose_ = self.get_parameter("verbose").get_parameter_value().bool_value

        self.declare_param("subscription_name", "kitti/velo", "This parameter decides the name of the node subscribed")
        self.subscription_name_ = self.get_parameter("subscription_name").get_parameter_value().string_value
        self.declare_param("publisher_name", "reduced_point_cloud", "This parameter decides the name of the publisher")
        self.publisher_name_ = self.get_parameter("publisher_name").get_parameter_value().string_value

        self.declare_param("device", "cpu", "This parameter decides which device you are going to use, 1. cpu (default), 2. cuda")
        self.declare_param("resolution", 0.4, "This parameter decides the side length of the square pillar, the unit is [meter]")
        self.declare_param("max_num_pillars", 30000, "This parameter decides how many pillars are allowed, set it to a reasonably"
                                                    "large number to cover all the pillars.")
        self.declare_param("lidar_ranges", [-90, -90, 90, 90], "This parameter accepts a vector which represents "
                                                            "[xmin, ymin, xmax, ymax] of the lidar, the unit is [meter]")
        self.declare_param("environment_radius", 1.8, "This parameter decides the radius of the environment to check in "
                                                    "the removal stage, the unit is [meter]")
        self.declare_param("env_min_threshold", 0.45, "This parameter is a threshold for comparing each pillar's lowest point with surrounding "
                                                            "environment's lowest point, the unit is [meter]")
        self.declare_param("max_min_threshold", 0.45, "This parameter is a threshold for comparing each pillar's highest point with its lowest point, "
                                                            "the unit is [meter]")
        self.declare_param("rebuild_radiuses", [1.8, 5.4], "This parameter decides how large area to rebuild the "
                                                                        "surrounding ground, multiple values mean we want to "
                                                                        "use different radius for different ranges, the unit "
                                                                        "is [meter]")
        self.declare_param("range_split", {30}, "This parameter denotes the split of different ranges when using "
                                                                "different radius. It should correspond to the number of radiuses "
                                                                "used. empty for global rebuild, the unit is [meter]")

        self.device_ = self.get_parameter("device").get_parameter_value().string_value
        self.device_num_ = -1 if self.device_ == "cpu" else 0
        self.resolution_ = self.get_parameter("resolution").get_parameter_value().double_value
        self.max_num_pillars_ = self.get_parameter("max_num_pillars").get_parameter_value().int_value
        self.lidar_ranges_ = self.get_parameter("lidar_ranges").get_parameter_value().double_array_value
        self.environment_radius_ = self.get_parameter("environment_radius").get_parameter_value().double_value
        self.env_min_threshold_ = self.get_parameter("env_min_threshold").get_parameter_value().double_value
        self.max_min_threshold_ = self.get_parameter("max_min_threshold").get_parameter_value().double_value
        self.rebuild_radiuses_ = self.get_parameter("rebuild_radiuses").get_parameter_value().double_array_value
        self.range_split_ = self.get_parameter("range_split").get_parameter_value().double_array_value

        self.num_received_points_ = 0
        self.num_send_points_ = 0

    def msgToTensor(self, msg: PointCloud2):
        dtype = np.dtype(msg.fields)
        data = np.frombuffer(msg.data, dtype=dtype)
        return data

    def callback(self, msg: PointCloud2):
        data = self.msgToTensor(msg)

def main(args=None):
    rclpy.init(args=args)

    pillar_based_removal_node = PillarBasedRemoval()

    rclpy.spin(pillar_based_removal_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pillar_based_removal_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()