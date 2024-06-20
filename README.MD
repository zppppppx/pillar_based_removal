# Pillar-based-removal

Pillar-based-removal is a point cloud pre-processing simple, effective and robust algorithm to remove irrelevant points for 3D object detection tasks.

https://github.com/zppppppx/pillar_based_removal/assets/43416275/25636da8-ff69-4cc3-bc17-e41c52eb9c18

### News
1. Python/CUDA version is finished. Be sure to run the algorithm on a device with CUDA.

### Package Requirements
#### c++ version
1. spconv c++ pure build 
2. ros2, in the docker container, the distro is humble
3. pcl, >= 1.10
4. vtk, >= 8.x

#### python version
1. cumm >= 4.8
2. spconv (modified version, see the link below)

### Before Building
1. (**for python version**) Install cumm and spconv in editable mode. Please refer to [Pillar branch of spconv](https://github.com/zppppppx/spconv/tree/pillarize) and follow the instructions in **Build from source for development (JIT, recommend)** to install required cumm and spconv modules. When the algorithm starts running, it could first trigger the build of spconv since the source codes have been updated. The prebuilt version could be published to pypi later.
2. (**for c++ version**) cmake build the spconv c++ code base. **C++** version is not completely finished due to some CUDA memory access issues.
3. Make sure you have read all the parameters.
4. The dataset is based on KITTI, and if you want to convert the raw data to ros2 version bag data, you could refer to the submodule under src directory.

### Build
1. (**for python version**)Under pillar_based_removal directory, run  `colcon build --symlink-install --packages-select pillar_based_removal_py`
2. (**for c++ version**)Under pillar_based_removal directory, run  `colcon build --symlink-install --packages-select pillar_based_removal`

### Resources
For quickly setting up the environment, you could use the docker image [zppppppx/rosspconv:latest](https://hub.docker.com/r/zppppppx/rosspconv)
