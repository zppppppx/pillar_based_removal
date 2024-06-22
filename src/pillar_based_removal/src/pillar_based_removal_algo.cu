#include <pillar_based_removal/pillar_based_removal.h>
#include <pillar_based_removal/kernel.h>

void PillarBasedRemoval::pillarize() {
    auto t0 = std::chrono::steady_clock::now();
    std::array<float, 3> psize_xyz {resolution_, resolution_, 60};
    std::array<float, 6> coors_range_xyz {lidar_ranges_[0], lidar_ranges_[1], -30, lidar_ranges_[2], lidar_ranges_[3], 30};
    int num_point_features = 1;
    int max_num_pillars = max_num_pillars_;
    int max_num_points_per_voxel = 2;
    Pillarize3D p2p {
        psize_xyz,
        coors_range_xyz,
        num_point_features,
        max_num_pillars,
        max_num_points_per_voxel
    };

    point_cloud_tensor_ = point_cloud_tensor_.cuda();
    auto pillar_tuple = p2p.point_to_pillar_hash(point_cloud_tensor_, true);
    pillars_ = std::get<0>(pillar_tuple);
    pillar_indices_ = std::get<1>(pillar_tuple);
    point_pillar_idx_ = std::get<3>(pillar_tuple);
    auto pillars_rw = pillars_.tview<float, 3>();
    pillars_highest_ = pillars_.select(1, 0).select(1, 0).unsqueeze(1);
    // auto hi_ptr = pillars_highest_.data_ptr<float>();
    // auto hi_rw = pillars_highest_.tview<float, 2>();
    pillars_lowest_ = pillars_.select(1, 1).select(1, 0).unsqueeze(1);
    kept_pillars_ = tv::zeros({pillars_.dim(0), 1}, tv::uint16, device_num_);
    // auto lo_rw = pillars_lowest_.tview<float, 2>();
    // auto lo_ptr = pillars_lowest_.data_ptr<float>();
    // tv::ssprint(pillars_.shape(), pillars_highest_.shape(), hi_rw(1, 0), lo_rw(1, 0), pillars_rw(1, 0, 0), pillars_rw(1, 1, 0));

    // tv::Tensor out_features; 
    // tv::Tensor features;
    // tv::Tensor indice_pairs; 
    // tv::Tensor indice_pair_num;
    // int num_activate_out; 
    // std::uintptr_t stream;
    // tv::Tensor indice_pair_mask;
    // std::vector<int> dilation;
    // std::vector<int> ksize;
    // std::vector<int> input_dims;
    // int batch_size;
    // tv::Tensor indice_num_per_loc;
    // tv::Tensor out_inds;
    // tv::Tensor hashdata_v;
    // tv::Tensor hashdata_k;
    // tv::Tensor indices;
    // SpconvOps::generate_subm_conv_inds(indices, hashdata_k, hashdata_v, indice_pairs, out_inds, indice_num_per_loc, batch_size, input_dims, ksize, dilation, indice_pair_mask, false, stream);

    auto p2p_grid_size = p2p.get_grid_size();
    grid_size_.assign(p2p_grid_size.begin(), p2p_grid_size.end());
    // tv::ssprint("grid_size", p2p_grid_size);

    auto t1 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    if(verbose_)
        RCLCPP_INFO(get_logger(), "Time needed for pillarization is %f", time_used_);
    checkCudaErrors(cudaStreamSynchronize(stream_));
}

void PillarBasedRemoval::removal_stage() {
    // auto t0 = Time::now();
    auto t0 = std::chrono::steady_clock::now();

    /*
    Calculate the height difference;
    */
    tv::Tensor diff = tv::zeros(pillars_highest_.shape(), tv::type_v<float>, device_num_);

    auto launcher = tv::cuda::Launch(diff.dim(0), stream_);
    launcher(kernel::get_diff, diff.data_ptr<float>(), pillars_highest_.data_ptr<float>(), 
            pillars_lowest_.data_ptr<float>(), diff.dim(0));


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
    tv::Tensor out_inds = tv::empty({(int)pillars_.dim(0), (int)pillars_.ndim() + 1}, tv::int32, device_num_);
    // tv::ssprint("In removal stage:", pillars_.shape());

    ws_tensors.insert({SPCONV_ALLOC_PAIR_FWD, pair});
    ws_tensors.insert(
        {SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num});
    ws_tensors.insert({SPCONV_ALLOC_OUT_INDICES, out_inds});
    StaticAllocator alloc(ws_tensors);

    // // tv::ssprint(out_dims, workspace_size);
    // // padding with batch
    auto pillar_indices_padded = tv::zeros({pillar_indices_.dim(0), 4}, tv::int32, device_num_);
    pillar_indices_padded.slice(1, 1, 4, 1, false, false).copy_2d_pitched_(pillar_indices_);
    // auto pillar_indices_padded_rw = pillar_indices_padded.tview<int32_t, 2>();
    // auto pillar_indices_rw = pillar_indices_.tview<int32_t, 2>();

    // for(size_t i = 0; i < pillar_indices_rw.dim(0); i++) {
    //     pillar_indices_padded_rw((int)i, 1) = pillar_indices_rw((int)i, 0);
    //     pillar_indices_padded_rw((int)i, 2) = pillar_indices_rw((int)i, 1);
    //     pillar_indices_padded_rw((int)i, 3) = pillar_indices_rw((int)i, 2);
    // }

    
    int num_act_out_real = SpconvOps::get_indice_pairs(
        alloc, pillar_indices_padded, batch_size, out_dims,
        static_cast<int>(tv::gemm::SparseConvAlgo::kNative), ksize, stride,
        padding, dilation, {0, 0, 0}, is_subm, false,
        reinterpret_cast<std::uintptr_t>(stream_), out_inds_num_limit,
        pillar_indices_.dim(0));

    // // auto pair_rw = pair.tview<int, 3>();
    // // tv::ssprint("input/output num_indices", pillar_indices_.dim(0), num_act_out_real, pair_rw(0, 0, 0), pair_rw(0, 0, 1));

    tv::Tensor indices_pair_num = tv::zeros({KV}, tv::int32, device_num_);
    environment_lowest_ = tv::empty({pillars_lowest_.dim(0), 1}, tv::float32, device_num_);

    // tv::Tensor out_features; 
    // tv::Tensor features;
    // tv::Tensor indice_pairs; 
    // tv::Tensor indice_pair_num;
    // int num_activate_out; 
    // std::uintptr_t stream;
    // SpconvOps::indice_maxpool(out_features, features, indice_pairs, indice_pair_num, num_activate_out, stream);


    // std::uintptr_t stream;
    // stream = 0;
    // spconvlib::spconv::csrc::sparse::all::SpconvOps::indice_maxpool(environment_lowest_, pillars_lowest_, 
    //                             pair, indices_pair_num, num_act_out_real, reinterpret_cast<std::uintptr_t>(stream_));
    // SpconvOps::maxpool_forward(environment_lowest_, pillars_lowest_, out_inds, 
    //                             pillar_indices_padded, reinterpret_cast<std::uintptr_t>(stream_));

    // tv::check_shape(environment_lowest_, {-1, pillars_lowest_.dim(1)});
    // auto indices_pair_num_cpu = indices_pair_num.cpu();
    // auto indices_pair_num_cpu_ptr = indices_pair_num_cpu.data_ptr<int>();
    // for (int i = 0; i < indices_pair_num.dim(0); ++i){
    //     int nhot = indices_pair_num_cpu_ptr[i];
    //     nhot = std::min(nhot, int(pair.dim(2)));
    //     if (nhot <= 0){
    //         continue;
    //     }
    //     auto inp_indices = pair[0][i].slice_first_axis(0, nhot);
    //     auto out_indices = pair[1][i].slice_first_axis(0, nhot);
    //     IndiceMaxPool::forward(out_features, features, out_indices, inp_indices, stream);
    // }

    

    // auto kept_pillars_ptr = kept_pillars_.data_ptr<uint16_t>();
    // auto pillars_lowest_ptr = pillars_lowest_.data_ptr<float>();
    // auto pillars_highest_ptr = pillars_highest_.data_ptr<float>();
    // auto environment_lowest_ptr = environment_lowest_.data_ptr<float>();
    // for(int i = 0; i < kept_pillars_.dim(0); i++) {
    //     if((pillars_lowest_ptr[i] - pillars_highest_ptr[i] >= max_min_threshold_) &&
    //         (pillars_lowest_ptr[i] - environment_lowest_ptr[i] >= env_min_threshold_)) {
    //         kept_pillars_ptr[i] = 1;
    //         num_send_points_ += 1;
    //     }
    // }

    // auto t1 = std::chrono::steady_clock::now();
    // std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
    // // auto t1 = Time::now();
    // // fsec duration = t1 - t0;
    // if(verbose_)
    //     RCLCPP_INFO(get_logger(), "Time needed for removal stage is %.10f, and kept pillars are: %d",
    //                                 time_used_.count(), num_send_points_);
}

// void PillarBasedRemoval::rebuild_stage() {
//     // padding with batch
//     auto pillar_indices_padded = tv::zeros({pillar_indices_.dim(0), 4}, tv::int32, device_num_);
//     tv::dispatch<int, int>(pillar_indices_padded.dtype(), [&](auto I) {
//         using T = decltype(I);
//         auto pillar_indices_padded_rw = pillar_indices_padded.tview<T, 2>();
//         auto pillar_indices_rw = pillar_indices_.tview<T, 2>();

//         for(size_t i = 0; i < pillar_indices_rw.dim(0); i++) {
//             pillar_indices_padded_rw((int)i, 1) = pillar_indices_rw((int)i, 0);
//             pillar_indices_padded_rw((int)i, 2) = pillar_indices_rw((int)i, 1);
//             pillar_indices_padded_rw((int)i, 3) = pillar_indices_rw((int)i, 2);
//         }
//     });

//     // Set up convolution working space
//     for(int i = 0; i < (int) rebuild_radiuses_.size(); i++) {
//         int rebuild_num = (int) round(rebuild_radiuses_[i] / resolution_ - 0.5) * 2 + 1; // kernel size
//         std::vector<int32_t> ksize{1, rebuild_num, rebuild_num};
//         int32_t KV = std::accumulate(std::begin(ksize), std::end(ksize), 1, std::multiplies<>{});
//         bool is_subm = true;
//         int out_inds_num_limit = 0;
//         int batch_size = 1;
//         std::vector<int32_t> padding{0, 0, 0};
//         std::vector<int32_t> dilation{1, 1, 1};
//         std::vector<int32_t> stride{1, 1, 1};

//         std::vector<int32_t> input_dims(grid_size_.begin(), grid_size_.end());
//         auto out_dims = SpconvOps::get_conv_output_size(input_dims, ksize, stride,
//                                                     padding, dilation);

//         int workspace_size = SpconvOps::get_indice_gen_workspace_size(
//                 KV, pillars_.dim(0), out_inds_num_limit,
//                 0, is_subm, false, false);
//         tv::Tensor workspace = tv::empty({workspace_size}, tv::uint8, device_num_);
//         auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
//                 workspace.raw_data(), KV, pillars_.dim(0),
//                 pillars_.dim(0), 0, is_subm,
//                 false, false);

//         tv::Tensor pair = tv::empty({2, KV, pillars_.dim(0)}, tv::int32, device_num_);
//         tv::Tensor indices_kernel_num = tv::zeros({KV}, tv::int32, device_num_);
//         tv::Tensor out_inds = tv::empty({(int)pillars_.dim(0), (int)pillars_.ndim() + 1}, tv::int32, device_num_);
//         // tv::ssprint("In removal stage:", pillars_.shape());

//         ws_tensors.insert({SPCONV_ALLOC_PAIR_FWD, pair});
//         ws_tensors.insert(
//             {SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num});
//         ws_tensors.insert({SPCONV_ALLOC_OUT_INDICES, out_inds});
//         StaticAllocator alloc(ws_tensors);


//         uint* stream = 0; // Don't use cuda stream
//         int num_act_out_real = SpconvOps::get_indice_pairs(
//             alloc, pillar_indices_padded, batch_size, out_dims,
//             static_cast<int>(tv::gemm::SparseConvAlgo::kNative), ksize, stride,
//             padding, dilation, {0, 0, 0}, is_subm, false,
//             reinterpret_cast<std::uintptr_t>(stream), out_inds_num_limit,
//             pillar_indices_.dim(0));

//         // Convolution
//         tv::Tensor weights = tv::full({1, 1, 1, 3, 3}, 1, tv::uint16, device_num_);
//         tv::Tensor bias = tv::zeros({1}, tv::uint16, device_num_);
//         tv::Tensor conv_kept_pillars =
//             tv::empty({pillar_indices_.dim(0), 1},
//                       tv::float16, device_num_);
//         GemmTunerSimple gemm_tuner(GemmMain::get_all_algo_desp());
//         std::unordered_map<std::string, tv::Tensor> tensor_dict{
//             {SPCONV_ALLOC_FEATURES, kept_pillars_},
//             {SPCONV_ALLOC_FILTERS, weights},
//             {SPCONV_ALLOC_OUT_FEATURES, conv_kept_pillars}};
//         StaticAllocator alloc2(tensor_dict);
//         // the SimpleExternalSpconvMatmul is used to perform bias operations
//         // provided by external bias library such as cublasLt. in pytorch this
//         // class use pytorch matmul.
//         // SimpleExternalSpconvMatmul ext_mm(alloc2);
//         // auto arch = ConvGemmOps::get_compute_capability();
//         // ConvGemmOps::indice_conv(
//         //     alloc2, ext_mm, gemm_tuner, true, false, kept_pillars_,
//         //     weights, pair, indices_kernel_num, arch, conv_kept_pillars.dim(0),
//         //     false, true,
//         //     static_cast<int>(tv::gemm::SparseConvAlgo::kNative),
//         //     reinterpret_cast<std::uintptr_t>(stream), bias,
//         //     1.0
//         //     /*bias alpha, only used for leaky relu*/,
//         //     0.0, tv::gemm::Activation::kNone);

//         ////////////////////////////////////////////////////////////////////////////
//         // auto max_act_out_theory = SpconvOps::get_handcrafted_max_act_out(
//         //     pillar_indices_.dim(0), ksize, stride, padding, dilation);
//         // int workspace_size = SpconvOps::get_indice_gen_workspace_size(
//         //     KV, pillar_indices_.dim(0), out_inds_num_limit, max_act_out_theory,
//         //     true, false, false);
//         // tv::Tensor workspace = tv::empty({workspace_size}, tv::uint8, device_num_);
//         // auto ws_tensors = SpconvOps::get_indice_gen_tensors_from_workspace(
//         //     workspace.raw_data(), KV, pillar_indices_.dim(0),
//         //     pillar_indices_.dim(0), max_act_out_theory, true, false, false);

//         // auto conv_algo = tv::gemm::SparseConvAlgo::kMaskImplicitGemm;
//         // tv::Tensor pair_fwd_padded =
//         //     tv::empty({KV, pillar_indices_.dim(0)}, tv::int32, device_num_);

//         // tv::Tensor pair_mask_fwd_padded =
//         //     tv::empty({1, pillar_indices_.dim(0)}, tv::int32, device_num_);
//         // tv::Tensor mask_argsort_fwd_padded =
//         //     tv::empty({1, pillar_indices_.dim(0)}, tv::int32, device_num_);
//         // tv::Tensor out_inds = tv::empty(
//         //     {pillar_indices_.dim(0), 4},
//         //     tv::int32, device_num_);

//         // out_inds.copy_cpu_(pillar_indices_padded);
//         // tv::Tensor indices_kernel_num = tv::zeros({KV}, tv::int32, device_num_);
//         // std::tuple<tv::Tensor, int> pair_res;

//         // ws_tensors.insert({SPCONV_ALLOC_PAIR_FWD, pair_fwd_padded});
//         // ws_tensors.insert({SPCONV_ALLOC_PAIR_MASK, pair_mask_fwd_padded});
//         // ws_tensors.insert(
//         //     {SPCONV_ALLOC_MASK_ARG_SORT, mask_argsort_fwd_padded});
//         // ws_tensors.insert({SPCONV_ALLOC_OUT_INDICES, out_inds});
//         // ws_tensors.insert(
//         //     {SPCONV_ALLOC_INDICE_NUM_PER_LOC, indices_kernel_num});
//         // StaticAllocator alloc(ws_tensors);

//         // uint* stream = 0; // Don't use cuda stream
//         // tv::ssprint(pillar_indices_padded.shape(), input_dims);
//         // pair_res = SpconvOps::get_indice_pairs_implicit_gemm(
//         //     alloc, pillar_indices_padded, batch_size, input_dims,
//         //     static_cast<int>(conv_algo), ksize, stride, padding, dilation,
//         //     {0, 0, 0}, is_subm, false, false /*is_train*/,
//         //     reinterpret_cast<std::uintptr_t>(nullptr), out_inds_num_limit,
//         //     tv::CUDAKernelTimer(false), false);
//     }
// }


// void test() {
//     cublasLtHandle_t handle_ = 0;
//     auto stat = cublasLtCreate(&handle_);
//     std::cout << "Creation status is " << stat << std::endl;
// }