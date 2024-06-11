#include<pillar_based_removal/pillar_based_removal.h>


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world pillar_based_removal package\n");
  return 0;
}

void PillarBasedRemoval::declare_param(const std::string& name, 
                                       const std::string& default_value, 
                                       const std::string& description)
{
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
}

void PillarBasedRemoval::set_params()
{
  declare_param("device", "cpu", "This parameter decides which device you are going to use, 1. cpu (default), 2. cuda");
  declare_param("resolution", 0.4, "This parameter decides the side length of the square pillar, the unit is [meter]");
  declare_param("max_num_pillars", 30000, "This parameter decides how many pillars are allowed, set it to a reasonably 
                                            large number to cover all the pillars.");
  declare_param("lidar_ranges", std::vector<float>{-90, -90, 90, 90}, "This parameter accepts a vector which represents 
                                                    [xmin, ymin, xmax, ymax] of the lidar, the unit is [meter]");
  declare_param("environment_radius", 1.8, "This paramter decides the radius of the environment to check in
                                             the removal stage, the unit is [meter]");
  declare_param("rebuild_radiuses", std::vector<float>{1.8, 5.4}, "This parameter decides how large area to rebuild the
                                                                   surrounding ground, multiple values mean we want to 
                                                                   use different radius for different ranges, the unit 
                                                                   is [meter]");
  declare_param("range_split", std::vector<float>{30}, "This parameter denotes the split of different ranges when using
                                                         different radius. It should correspond to the number of radiuses 
                                                         used. empty for global rebuild, the unit is [meter]");

  device_ = get_parameter("device").as_string();
  resolution_ = get_parameter("resolution");
  max_num_pillars_ = get_parameter("max_num_pillars");
  lidar_ranges_ = get_parameter("lidar_ranges");
  environment_radius_ = get_parameter("environment_radius");
  rebuild_radiuses_ = get_parameter("rebuild_radiuses");
  range_split_ = get_parameter("range_split");
  
}

void msgToTensor(const sensor_msgs::msg::PointCloud2 &point_cloud) {
  received_point_cloud_ = point_cloud;
}

PillarBasedRemoval::PillarBasedRemoval() : Node("pillar_based_removal_node") {
  set_params();
  

}