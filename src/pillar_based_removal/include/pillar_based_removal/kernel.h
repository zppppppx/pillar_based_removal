#pragma once
#include <tensorview/hash/ops.h>
#include <spconvlib/cumm/common/TensorView.h>
#include <spconvlib/cumm/common/TensorViewHashKernel.h>

namespace kernel {

__global__ void get_diff(float* diff, float* hi, float* lo, int num_pillars) {
    for(int i : tv::KernelLoopX<int>(num_pillars)) {
        diff[i] = hi[i] - lo[i];
    }
}

__global__ void set_kept_pillars() {}

}