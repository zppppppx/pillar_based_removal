#include<pillar_based_removal/pillar_based_removal.h>

void PillarBasedRemoval::pillarize() {
    auto t0 = std::chrono::steady_clock::now();
    std::array<float, 3> psize_xyz {resolution_, resolution_, 60};
    std::array<float, 6> coors_range_xyz {lidar_ranges_[0], lidar_ranges_[1], -30, lidar_ranges_[2], lidar_ranges_[3], 30};
    int num_point_features = 1;
    int max_num_pillars = max_num_pillars_;
    int max_num_points_per_voxel = 2;
    PillarizeCPU3D p2p {
        psize_xyz,
        coors_range_xyz,
        num_point_features,
        max_num_pillars,
        max_num_points_per_voxel
    };

    auto pillar_tuple = p2p.point_to_pillar_full(point_cloud_tensor_, true);
    pillars_ = std::get<0>(pillar_tuple);
    pillar_indices_ = std::get<1>(pillar_tuple);
    point_pillar_idx_ = std::get<3>(pillar_tuple);
    auto pillars_rw = pillars_.tview<float, 3>();
    pillars_highest_ = pillars_.select(1, 0).select(1, 0).unsqueeze(1);
    // auto hi_ptr = pillars_highest_.data_ptr<float>();
    auto hi_rw = pillars_highest_.tview<float, 2>();
    pillars_lowest_ = pillars_.select(1, 1).select(1, 0).unsqueeze(1);
    kept_pillars_ = tv::zeros({pillars_.dim(0), 1}, tv::uint16, device_num_);
    auto lo_rw = pillars_lowest_.tview<float, 2>();
    // auto lo_ptr = pillars_lowest_.data_ptr<float>();
    tv::ssprint(pillars_.shape(), pillars_highest_.shape(), hi_rw(1, 0), lo_rw(1, 0), pillars_rw(1, 0, 0), pillars_rw(1, 1, 0));

    auto p2p_grid_size = p2p.get_grid_size();
    grid_size_.assign(p2p_grid_size.begin(), p2p_grid_size.end());
    // tv::ssprint("grid_size", p2p_grid_size);

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    RCLCPP_INFO(get_logger(), "Time needed for pillarization is %f", time_used_);

}

void PillarBasedRemoval::removal_stage() {
    // auto t0 = Time::now();
    auto t0 = std::chrono::steady_clock::now();

    /*
    Calculate the height difference;
    */
    tv::Tensor diff = tv::zeros(pillars_highest_.shape(), tv::type_v<float>, device_num_);
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

    int adjacent_pillars = (int) round(environment_radius_ / resolution_ - 0.5) * 2 + 1; // kernel size
    std::vector<int32_t> ksize{1, adjacent_pillars, adjacent_pillars};
    int32_t KV = std::accumulate(std::begin(ksize), std::end(ksize), 1, std::multiplies<>{});
    bool is_subm = true;
    int out_inds_num_limit = 0;
    int batch_size = 1;
    std::vector<int32_t> padding{0, 0, 0};
    std::vector<int32_t> dilation{1, 1, 1};
    std::vector<int32_t> stride{1, 1, 1};


    // Set up maxpool working space
    std::vector<int32_t> input_dims(grid_size_.begin(), grid_size_.end());
    auto out_dims = SpconvOps::get_conv_output_size(input_dims, ksize, stride,
                                                  padding, dilation);
    int workspace_size = SpconvOps::get_indice_gen_workspace_size(
            KV, pillars_.dim(0), out_inds_num_limit,
            0, is_subm, false, false);
    tv::Tensor workspace = tv::empty({workspace_size}, tv::uint8, device_num_);
    auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
            workspace.raw_data(), KV, pillars_.dim(0),
            pillars_.dim(0), 0, is_subm,
            false, false);

    tv::Tensor pair = tv::empty({2, KV, pillars_.dim(0)}, tv::int32, device_num_);
    tv::Tensor indices_kernel_num = tv::zeros({KV}, tv::int32, device_num_);
    tv::Tensor out_inds = tv::empty({pillars_.dim(0), pillars_.ndim() + 1}, tv::int32, device_num_);
    // tv::ssprint("In removal stage:", pillars_.shape());

    ws_tensors.insert({SPCONV_ALLOC_PAIR_FWD, pair});
    ws_tensors.insert(
        {SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num});
    ws_tensors.insert({SPCONV_ALLOC_OUT_INDICES, out_inds});
    StaticAllocator alloc(ws_tensors);

    // tv::ssprint(out_dims, workspace_size);
    // padding with batch
    auto pillar_indices_padded = tv::zeros({pillar_indices_.dim(0), 4}, tv::int32, device_num_);
    tv::dispatch<int, int>(pillar_indices_padded.dtype(), [&](auto I) {
        using T = decltype(I);
        auto pillar_indices_padded_rw = pillar_indices_padded.tview<T, 2>();
        auto pillar_indices_rw = pillar_indices_.tview<T, 2>();

        for(size_t i = 0; i < pillar_indices_rw.dim(0); i++) {
            pillar_indices_padded_rw((int)i, 1) = pillar_indices_rw((int)i, 0);
            pillar_indices_padded_rw((int)i, 2) = pillar_indices_rw((int)i, 1);
            pillar_indices_padded_rw((int)i, 3) = pillar_indices_rw((int)i, 2);
        }
    });

    uint* stream = 0; // Don't use cuda stream
    int num_act_out_real = SpconvOps::get_indice_pairs(
        alloc, pillar_indices_padded, batch_size, out_dims,
        static_cast<int>(tv::gemm::SparseConvAlgo::kNative), ksize, stride,
        padding, dilation, {0, 0, 0}, is_subm, false,
        reinterpret_cast<std::uintptr_t>(stream), out_inds_num_limit,
        pillar_indices_.dim(0));

    auto pair_rw = pair.tview<int, 3>();
    // tv::ssprint("input/output num_indices", pillar_indices_.dim(0), num_act_out_real, pair_rw(0, 0, 0), pair_rw(0, 0, 1));

    tv::Tensor indices_pair_num = tv::zeros({KV}, tv::int32, device_num_);
    environment_lowest_ = tv::empty({pillars_lowest_.dim(0), 1}, tv::float32, device_num_);

    tv::check_shape(environment_lowest_, {-1, pillars_lowest_.dim(1)});

    auto indices_pair_num_ptr = indices_pair_num.data_ptr<int>();
    for (int i = 0; i < indices_pair_num.dim(0); ++i) {
        int nhot = indices_pair_num_ptr[i];
        nhot = std::min(nhot, int(pair.dim(2)));
        if (nhot <= 0){
            continue;
        }
        auto inp_indices = pair[0][i].slice_first_axis(0, nhot);
        auto out_indices = pair[1][i].slice_first_axis(0, nhot);
        IndiceMaxPoolCPU::forward(environment_lowest_, pillars_lowest_, out_indices, inp_indices);
    }   

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);

    auto kept_pillars_ptr = kept_pillars_.data_ptr<uint16_t>();
    auto pillars_lowest_ptr = pillars_lowest_.data_ptr<float>();
    auto pillars_highest_ptr = pillars_highest_.data_ptr<float>();
    auto environment_lowest_ptr = environment_lowest_.data_ptr<float>();
    for(int i = 0; i < kept_pillars_.dim(0); i++) {
        if((pillars_lowest_ptr[i] - pillars_highest_ptr[i] >= max_min_threshold_) &&
            (pillars_lowest_ptr[i] - environment_lowest_ptr[i] >= env_min_threshold_)) {
            kept_pillars_ptr[i] = 1;
            num_send_points_ += 1;
        }
    }


    // auto t1 = Time::now();
    // fsec duration = t1 - t0;
    RCLCPP_INFO(get_logger(), "Time needed for removal stage is %.10f, and kept pillars are: %d", time_used_.count(), num_send_points_);
}