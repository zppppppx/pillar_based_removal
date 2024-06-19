from spconv.pytorch.utils import PointToVoxel as PillarGenerator
import time
import torch
from spconv.pytorch.pool import SparseMaxPool
import spconv.pytorch as spconv


def pillarize(self):
    start = time.time()

    self._pillar_generator = PillarGenerator(
        vsize_xyz=[self.resolution_, self.resolution_, 60],
        coors_range_xyz=[self.lidar_ranges_[0], self.lidar_ranges_[1], -30, self.lidar_ranges_[2], self.lidar_ranges_[3], 30],
        num_point_features=1,
        max_num_points_per_voxel=2,
        max_num_voxels=self.max_num_pillars_,
        device=torch.device(self.device_)
    )

    pillars, self.pillar_indices_, self.num_per_pillar_, self.point_pillar_idx_ \
        = self._pillar_generator.pillarize(self.point_cloud_tensor_)
    self.pillars_highest_, self.pillars_lowest_ = pillars[:, 0], pillars[:, 1]
    self.kept_pillars_ = torch.ones((pillars.shape[0], 1), dtype=torch.float32, device=torch.device(self.device_))
    self.origin_ = torch.tensor((int(-self.lidar_ranges_[1] / self.resolution_), 
                                int(-self.lidar_ranges_[0] / self.resolution_)), device=torch.device(self.device_))

    end = time.time()
    duration = end - start
    if self.verbose_:
        self.get_logger().info("Time needed for pillarization is %f" % duration)


def removal_stage(self):
    start = time.time()

    adjacent_pillars = round(self.environment_radius_ / self.resolution_ - 0.5) * 2 + 1
    pool_layer = SparseMaxPool(ndim=2, kernel_size=adjacent_pillars, stride=1, dilation=1, subm=True)


    env_sparse_tensor = spconv.SparseConvTensor(
        features=-self.pillars_lowest_.to(torch.device(self.device_)), 
        indices=self.pillar_indices_.int().to(torch.device(self.device_)),
        spatial_shape=self._pillar_generator.grid_size[1:], 
        batch_size=1
    )

    output = pool_layer(env_sparse_tensor)
    env_lowest_ = -output.features

    self.kept_pillars_[:, 0] = (self.pillars_highest_.squeeze() - self.pillars_lowest_.squeeze() > self.max_min_threshold_) | \
                        (self.pillars_lowest_.squeeze() - env_lowest_.squeeze() > self.env_min_threshold_)


    end = time.time()
    duration = end - start
    if self.verbose_:
        self.get_logger().info("Time needed for removal stage is %f" % duration)


def rebuild_stage(self):
    start = time.time()

    distance = self.pillar_indices_[:, 1:].clone().float()
    distance -= self.origin_
    distance = torch.norm(distance*self.resolution_, dim=1)

    range_split = [float('-inf')] + list(self.range_split_)+ [float('inf')]

    final_kept_pillars = self.kept_pillars_.clone()

    rebuild_dims = [round(rebuild_radius / self.resolution_ - 0.5) * 2 + 1 for rebuild_radius in self.rebuild_radiuses_]
    for idx, rebuild_dim in enumerate(rebuild_dims):
        conv_layer = spconv.SubMConv2d(1, 1, rebuild_dim, 1, 0, bias=False)
        if self.device_ in ['cuda']:
            conv_layer = conv_layer.cuda()
        torch.nn.init.ones_(conv_layer.weight)
        conv_layer.weight.requires_grad = False

        range_mask = (distance > range_split[idx]-2*self.rebuild_radiuses_[idx]) & \
                    (distance <= range_split[idx+1]+2*self.rebuild_radiuses_[idx])
        
        pillar_mask_all = (distance > range_split[idx]) & (distance <= range_split[idx+1])
        pillar_mask_range = pillar_mask_all[range_mask]

        kept_pillars_idx = self.kept_pillars_[range_mask]
        coordinates_idx = self.pillar_indices_[range_mask]

        kept_pillars_sparse = spconv.SparseConvTensor(
            features=kept_pillars_idx.view((-1, 1)), indices=coordinates_idx.int(),
            spatial_shape=self._pillar_generator.grid_size[1:], batch_size=1
        )
        with torch.no_grad():
            output_tensor = conv_layer(kept_pillars_sparse)
            rebuilt_pillars_idx = output_tensor.features.squeeze()
            
            final_kept_pillars[pillar_mask_all][:, 0] += rebuilt_pillars_idx[pillar_mask_range]

    self.kept_pillars_ = final_kept_pillars.squeeze()
    self.kept_pillars_ = (self.kept_pillars_ > 0)
    pillar_indices = torch.cumsum(torch.ones((len(self.kept_pillars_), ), device=self.kept_pillars_.device), dim=0) - 1
    self.kept_pillars_ = pillar_indices[self.kept_pillars_]

    end = time.time()
    duration = end - start
    if self.verbose_:
        self.get_logger().info("Time needed for rebuild stage is %f" % duration)