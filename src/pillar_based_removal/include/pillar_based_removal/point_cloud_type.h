// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

typedef struct 
{
    PCL_ADD_POINT4D         // XYZ, PCL macro
    PCL_ADD_INTENSI         // Intensity, PCL macro
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
} PointT;

PointXYZ x;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
)

typedef pcl::PointCloud<PointT> PointCloudT;