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
  declare_param<std::string>("subscription_name", "point_cloud", "This parameter decides the name of the node subscribed");
  subscription_name_ = get_parameter("subscription_name").as_string();

  declare_param<std::string>("device", "cpu", "This parameter decides which device you are going to use, 1. cpu (default), 2. cuda");
  declare_param<double>("resolution", 0.4, "This parameter decides the side length of the square pillar, the unit is [meter]");
  declare_param<int>("max_num_pillars", 30000, "This parameter decides how many pillars are allowed, set it to a reasonably"
                                            "large number to cover all the pillars.");
  declare_param<std::vector<double>>("lidar_ranges", std::vector<double>{-90, -90, 90, 90}, "This parameter accepts a vector which represents "
                                                    "[xmin, ymin, xmax, ymax] of the lidar, the unit is [meter]");
  declare_param<double>("environment_radius", 1.8, "This paramter decides the radius of the environment to check in "
                                             "the removal stage, the unit is [meter]");
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
  rebuild_radiuses_ = get_parameter("rebuild_radiuses").as_double_array();
  range_split_ = get_parameter("range_split").as_double_array();
  
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
  
  RCLCPP_INFO(get_logger(), "The time needed for converting %d points to tensor is %f." 
                            "\nThe size of the point tensor is %d, and the first point's x is %f", 
                            (int) num_received_points_, time_used_.count(), 
                            (int)point_cloud_tensor_.size(), point_cloud_tensor_ptr[0]);
}


PillarBasedRemoval::PillarBasedRemoval() : Node("pillar_based_removal_node") {
  set_params();
  pointcloud_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    subscription_name_, rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile(),
             std::bind(&PillarBasedRemoval::callback, this, std::placeholders::_1)
  );
}

PillarBasedRemoval::~PillarBasedRemoval() {}


void PillarBasedRemoval::callback(const sensor_msgs::msg::PointCloud2 &received_point_cloud_msg) {
  msgToTensor(received_point_cloud_msg);
  pillarize();
  removal_stage();
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PillarBasedRemoval>());
  rclcpp::shutdown();

  return 0;
}
