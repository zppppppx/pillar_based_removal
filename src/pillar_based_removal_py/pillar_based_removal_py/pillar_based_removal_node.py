import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import array

# from .pillar_based_removal_algo import *
import sensor_msgs_py.point_cloud2 as pc2

import torch
import numpy as np
from sensor_msgs.msg import PointCloud2
from .pillar_based_removal_algo import *
import time

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

        self.publisher = self.create_publisher(
            PointCloud2,
            self.publisher_name_,
            1
        )

    
    def declare_param(self, param_name: str, default, description: str):
        descriptor = ParameterDescriptor(description=description)
        self.declare_parameter(param_name, default, descriptor)

    def set_params(self):
        self.declare_param("verbose", True, "If set true, the description of each module will be verbosely printed in the terminal")
        self.verbose_ = self.get_parameter("verbose").get_parameter_value().bool_value

        self.declare_param("subscription_name", "kitti/velo", "This parameter decides the name of the node subscribed")
        self.subscription_name_ = self.get_parameter("subscription_name").get_parameter_value().string_value
        self.declare_param("publisher_name", "reduced_point_cloud", "This parameter decides the name of the publisher")
        self.publisher_name_ = self.get_parameter("publisher_name").get_parameter_value().string_value
        self.declare_param("point_fields", ["x", "y", "z", "intensity"], "Please specify the point cloud's field names and ensure the \
                                                                            first three indicate xyz values")
        self.point_fileds_ = self.get_parameter("point_fields").get_parameter_value().string_array_value
        self.declare_param("device", "cuda", "This parameter decides which device you are going to use, 1. cpu (default), 2. cuda")
        self.declare_param("resolution", 0.45, "This parameter decides the side length of the square pillar, the unit is [meter]")
        self.declare_param("max_num_pillars", 30000, "This parameter decides how many pillars are allowed, set it to a reasonably"
                                                    "large number to cover all the pillars.")
        self.declare_param("lidar_ranges", [-90., -90., 90., 90.], "This parameter accepts a vector which represents "
                                                            "[xmin, ymin, xmax, ymax] of the lidar, the unit is [meter]")
        self.declare_param("environment_radius", 0.9, "This parameter decides the radius of the environment to check in "
                                                    "the removal stage, the unit is [meter]")
        self.declare_param("env_min_threshold", 0.6, "This parameter is a threshold for comparing each pillar's lowest point with surrounding "
                                                            "environment's lowest point, the unit is [meter]")
        self.declare_param("max_min_threshold", 0.6, "This parameter is a threshold for comparing each pillar's highest point with its lowest point, "
                                                            "the unit is [meter]")
        self.declare_param("rebuild_radiuses", [1.8, 5.4], "This parameter decides how large area to rebuild the "
                                                                        "surrounding ground, multiple values mean we want to "
                                                                        "use different radius for different ranges, the unit "
                                                                        "is [meter]")
        self.declare_param("range_split", [30.], "This parameter denotes the split of different ranges when using "
                                                                "different radius. It should correspond to the number of radiuses "
                                                                "used. empty for global rebuild, the unit is [meter]")

        self.device_ = self.get_parameter("device").get_parameter_value().string_value
        self.device_num_ = -1 if self.device_ == "cpu" else 0
        self.resolution_ = self.get_parameter("resolution").get_parameter_value().double_value
        self.max_num_pillars_ = self.get_parameter("max_num_pillars").get_parameter_value().integer_value
        self.lidar_ranges_ = self.get_parameter("lidar_ranges").get_parameter_value().double_array_value
        self.environment_radius_ = self.get_parameter("environment_radius").get_parameter_value().double_value
        self.env_min_threshold_ = self.get_parameter("env_min_threshold").get_parameter_value().double_value
        self.max_min_threshold_ = self.get_parameter("max_min_threshold").get_parameter_value().double_value
        self.rebuild_radiuses_ = self.get_parameter("rebuild_radiuses").get_parameter_value().double_array_value
        self.range_split_ = self.get_parameter("range_split").get_parameter_value().double_array_value

        self.num_received_points_ = 0
        self.num_send_points_ = 0

    def msgToTensor(self, msg: PointCloud2):
        start = time.time()

        self.received_point_cloud_msg_ = msg
        
        data = pc2.read_points(msg, None, True)
        num_points = len(data)
        self.resolved_data = data
        
        self.point_cloud_tensor_ = torch.zeros((num_points, 3), device=torch.device(self.device_))

        idx = 0
        for field_name in self.point_fileds_[:3]:
            self.point_cloud_tensor_[:, idx] = torch.tensor(data[field_name], 
                                                            dtype=torch.float32, device=torch.device(self.device_))
            idx += 1

        self.point_cloud_tensor_ = self.point_cloud_tensor_.contiguous()
        self.num_received_points_ = self.point_cloud_tensor_.shape[0]

        end = time.time()
        duration = end - start
        if self.verbose_:
            self.get_logger().info("Time needed for resolving received points is %f" % duration)
    
    def tensorToMsg(self):
        start = time.time()

        valid_points_indices = torch.isin(self.point_pillar_idx_, self.kept_pillars_).cpu().numpy()
        resolved_data = self.resolved_data[valid_points_indices]

        memory_view = memoryview(resolved_data)
        casted = memory_view.cast('B')
        array_array = array.array('B')
        array_array.frombytes(casted)

        sending_point_cloud_msg = self.received_point_cloud_msg_
        sending_point_cloud_msg.data = array_array
        sending_point_cloud_msg.width = len(resolved_data)
        sending_point_cloud_msg.height = 1
        sending_point_cloud_msg.is_dense = False
        sending_point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        self.publisher.publish(sending_point_cloud_msg)

        end = time.time()
        duration = end - start
        if self.verbose_:
            self.get_logger().info("Time needed for resolving target points is %f" % duration)


    def callback(self, msg: PointCloud2):
        start = time.time()

        self.msgToTensor(msg)
        self.pillarize()
        self.removal_stage()
        self.rebuild_stage()
        self.tensorToMsg()

        end = time.time()
        duration = end - start
        self.get_logger().info("Overall time needed is %f seconds" % duration)


PillarBasedRemoval.pillarize = pillarize
PillarBasedRemoval.removal_stage = removal_stage
PillarBasedRemoval.rebuild_stage = rebuild_stage

def main(args=None):
    rclpy.init(args=args)

    pillar_based_removal_node = PillarBasedRemoval()

    rclpy.spin(pillar_based_removal_node)

    pillar_based_removal_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()