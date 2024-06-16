#include<pillar_based_removal/pillar_based_removal.h>



template<class T>
void PillarBasedRemoval::declare_param(const std::string &name, 
                                       const T &default_value, 
                                       const std::string &description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
}

void PillarBasedRemoval::set_params()
{
  declare_param<bool>("verbose", false, "If set true, the description of each module will be verbosely printed in the terminal");
  verbose_ = get_parameter("verbose").as_bool();

  declare_param<std::string>("subscription_name", "point_cloud", "This parameter decides the name of the node subscribed");
  subscription_name_ = get_parameter("subscription_name").as_string();
  declare_param<std::string>("publisher_name", "reduced_point_cloud", "This parameter decides the name of the publisher");
  publisher_name_ = get_parameter("publisher_name").as_string();

  declare_param<std::string>("device", "cpu", "This parameter decides which device you are going to use, 1. cpu (default), 2. cuda");
  declare_param<double>("resolution", 0.4, "This parameter decides the side length of the square pillar, the unit is [meter]");
  declare_param<int>("max_num_pillars", 30000, "This parameter decides how many pillars are allowed, set it to a reasonably"
                                            "large number to cover all the pillars.");
  declare_param<std::vector<double>>("lidar_ranges", std::vector<double>{-90, -90, 90, 90}, "This parameter accepts a vector which represents "
                                                    "[xmin, ymin, xmax, ymax] of the lidar, the unit is [meter]");
  declare_param<double>("environment_radius", 1.8, "This parameter decides the radius of the environment to check in "
                                             "the removal stage, the unit is [meter]");
  declare_param<double>("env_min_threshold", 0.45, "This parameter is a threshold for comparing each pillar's lowest point with surrounding "
                                                    "environment's lowest point, the unit is [meter]");
  declare_param<double>("max_min_threshold", 0.45, "This parameter is a threshold for comparing each pillar's highest point with its lowest point, "
                                                    "the unit is [meter]");
  declare_param<std::vector<double>>("rebuild_radiuses", std::vector<double>{1.8, 5.4}, "This parameter decides how large area to rebuild the "
                                                                   "surrounding ground, multiple values mean we want to "
                                                                   "use different radius for different ranges, the unit "
                                                                   "is [meter]");
  declare_param<std::vector<double>>("range_split", std::vector<double>{30}, "This parameter denotes the split of different ranges when using "
                                                         "different radius. It should correspond to the number of radiuses "
                                                         "used. empty for global rebuild, the unit is [meter]");

  device_ = get_parameter("device").as_string();
  device_num_ = device_ == "cpu" ? -1 : 0;
  resolution_ = get_parameter("resolution").as_double();
  max_num_pillars_ = get_parameter("max_num_pillars").as_int();
  lidar_ranges_ = get_parameter("lidar_ranges").as_double_array();
  environment_radius_ = get_parameter("environment_radius").as_double();
  env_min_threshold_ = get_parameter("env_min_threshold").as_double();
  max_min_threshold_ = get_parameter("max_min_threshold").as_double();
  rebuild_radiuses_ = get_parameter("rebuild_radiuses").as_double_array();
  range_split_ = get_parameter("range_split").as_double_array();

  num_received_points_ = 0;
  num_send_points_ = 0;
  
}


void PillarBasedRemoval::msgToTensor(const sensor_msgs::msg::PointCloud2 &received_point_cloud_msg) {
  auto t0 = std::chrono::steady_clock::now();

  pcl::fromROSMsg<PointT>(received_point_cloud_msg, received_point_cloud_);
  num_received_points_ = received_point_cloud_.points.size();
  
  point_cloud_tensor_ = tv::zeros({num_received_points_, 3}, tv::type_v<float>, device_num_);
  auto point_cloud_tensor_ptr = point_cloud_tensor_.data_ptr<float>();

  tv::dispatch<float, float>(point_cloud_tensor_.dtype(), [&](auto I) {
    using T = decltype(I);
    auto point_cloud_tensor_rw = point_cloud_tensor_.tview<T, 2>();
    for(size_t i = 0; i < num_received_points_; i++) {
      point_cloud_tensor_rw((int)i, 0) = received_point_cloud_.points[i].x;
      point_cloud_tensor_rw((int)i, 1) = received_point_cloud_.points[i].y;
      point_cloud_tensor_rw((int)i, 2) = received_point_cloud_.points[i].z;
    }
  });

  auto t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  
  if(verbose_)
    RCLCPP_INFO(get_logger(), "The time needed for converting %d points to tensor is %f.",
                              (int) num_received_points_, time_used_.count());
}

void PillarBasedRemoval::tensorToMsg() {
  // Save all the valid indexes
  std::unordered_set<int> valid_voxel_indices;
  auto kept_pillars_ptr = kept_pillars_.data_ptr<uint16_t>();
  for(int i = 0; i < kept_pillars_.dim(0); i++) {
    if(kept_pillars_ptr[i] > 0) {
      valid_voxel_indices.insert(i);
    }
  }

  PointCloudT target_point_cloud = PointCloudT();
  // target_point_cloud.resize(num_send_points_);
  // int idx = 0;
  num_send_points_ = 0;
  auto point_pillar_idx_ptr = point_pillar_idx_.data_ptr<int64_t>();
  for(int i = 0; i < num_received_points_; i++) {
    if(valid_voxel_indices.count((int) point_pillar_idx_ptr[i])) {
      // target_point_cloud[idx] = received_point_cloud_[i];
      target_point_cloud.push_back(received_point_cloud_[i]);
      num_send_points_++;
      // idx++;
    }
  }

  pcl::toROSMsg(target_point_cloud, target_point_cloud_msg_);

  // Publishing the point cloud message.
  if(verbose_)
    RCLCPP_INFO(get_logger(), "Publishing point cloud, the number of sent points is %zu", num_send_points_);
  pointcloud_publisher_->publish(target_point_cloud_msg_);
}


PillarBasedRemoval::PillarBasedRemoval() : Node("pillar_based_removal_node") {
  set_params();
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    subscription_name_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
             std::bind(&PillarBasedRemoval::callback, this, std::placeholders::_1)
  );
  pointcloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    publisher_name_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()
  );
}

PillarBasedRemoval::~PillarBasedRemoval() {}


void PillarBasedRemoval::callback(const sensor_msgs::msg::PointCloud2 &received_point_cloud_msg) {
  auto t0 = std::chrono::steady_clock::now();

  msgToTensor(received_point_cloud_msg);
  pillarize();
  removal_stage();
  rebuild_stage();
  tensorToMsg();

  auto t1 = std::chrono::steady_clock::now();
  std::chrono::duration<double> time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
  RCLCPP_INFO(get_logger(), "Overall time needed for the algorithm is %f sec", time_used_.count());
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PillarBasedRemoval>());
  rclcpp::shutdown();

  return 0;
}
