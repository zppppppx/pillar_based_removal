#include<pillar_based_removal/pillar_based_removal.h>

void PillarBasedRemoval::pillarize() {
    auto t0 = Time::now();
    std::array<float, 3> psize_xyz {resolution_, resolution_, 60};
    std::array<float, 6> coors_range_xyz {lidar_ranges_[0], lidar_ranges_[1], -30, lidar_ranges_[2], lidar_ranges_[3], 30};
    int num_point_features = 3;
    int max_num_pillars = max_num_pillars_;
    int max_num_points_per_voxel = 2;
    PillarizeCPU3D p2p {
        psize_xyz,
        coors_range_xyz,
        num_point_features,
        max_num_pillars,
        max_num_points_per_voxel
    };

    auto pillar_tuple = p2p.point_to_voxel(point_cloud_tensor_, true);

    auto t1 = Time::now();
    fsec duration = t1 - t0;
    RCLCPP_INFO(get_logger(), "Time needed for pillarization is %f", duration);

}