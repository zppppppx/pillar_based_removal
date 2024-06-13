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

    auto pillar_tuple = p2p.point_to_voxel_full(point_cloud_tensor_, true);
    auto pillars_ = std::get<0>(pillar_tuple);
    auto point_pillar_idx_ = std::get<3>(pillar_tuple);
    auto pillars_rw = pillars_.tview<float, 3>();
    pillars_highest_ = pillars_.select(1, 0).select(1, 2).unsqueeze(1);
    // auto hi_ptr = pillars_highest_.data_ptr<float>();
    // auto hi_rw = pillars_highest_.tview<float, 2>();
    pillars_lowest_ = pillars_.select(1, 1).select(1, 2).unsqueeze(1);
    // auto lo_rw = pillars_lowest_.tview<float, 2>();
    // auto lo_ptr = pillars_lowest_.data_ptr<float>();
    // tv::ssprint(pillars_.shape(), pillars_highest_.shape(), hi_rw(1, 0), lo_rw(1, 0), pillars_rw(1, 0, 2), pillars_rw(1, 1, 2));

    auto t1 = Time::now();
    fsec duration = t1 - t0;
    RCLCPP_INFO(get_logger(), "Time needed for pillarization is %f", duration);

}

void PillarBasedRemoval::remove_stage() {
    tv::Tensor diff = tv::zeros(pillars_highest_.shape(), tv::type_v<float>, device_num_);

    /*
    Calculate the height difference;
    */
    tv::dispatch<float, float>(diff.dtype(), [&](auto I) {
        using T = decltype(I);
        auto hi_rw = pillars_highest_.tview<T, 2>();
        auto lo_rw = pillars_lowest_.tview<T, 2>();
        auto diff_rw = diff.tview<T, 2>();

        for(size_t i = 0; i < hi_rw.dim(0); i++) {
        diff_rw((int)i, 0) = hi_rw((int)i, 0) - lo_rw((int)i, 0);
        }
    });

    auto hi_rw = pillars_highest_.tview<float, 2>();
    auto lo_rw = pillars_lowest_.tview<float, 2>();
    auto diff_rw = diff.tview<float, 2>();
    // tv::ssprint(lo_rw(0, 0), hi_rw(0, 0), hi_rw(0, 0) - lo_rw(0, 0), diff_rw(0, 0));

    int adjacent_pillars = (int) round(environment_radius_ / resolution_ - 0.5) * 2 + 1;
}