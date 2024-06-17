Pillar_based_removal


### Package Requirements
1. spconv c++ pure build
2. ros2, in the docker container, the distro is humble
3. pcl, >= 1.10
4. vtk, >= 8.x

### Build
Under pillar_based_removal directory, run $colcon build --symlink-install --packages-select pillar_based_removal$ or $colcon build --symlink-install --packages-select pillar_based_removal_py$