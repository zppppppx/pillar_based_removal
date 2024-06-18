from pillar_based_removal_node import PillarBasedRemoval
from spconv.pytorch.utils import PointToVoxel as VoxelGenerator


def pillarize(self):
    num_point_features = self.point_cloud_tensor_.shape[1]
    self._pillar_generator = VoxelGenerator(
        vsize_xyz=[self.resolution_, self.resolution_, 60],
        coors_range_xyz=[self.lidar_ranges_[0], self.lidar_ranges_[1], -30, self.lidar_ranges_[2], self.lidar_ranges_[3], 30],
        num_point_features=num_point_features,
        max_num_points_per_voxel=2,
        max_num_voxels=self.max_num_pillars_,
        device=self.device_num_
    )


PillarBasedRemoval.pillarize = pillarize