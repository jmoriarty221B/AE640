#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_publisher", ros::init_options::AnonymousName);

  if (argc <= 1) {
    ROS_ERROR("image_publisher requires filename. Typical command-line usage:\n"
              "\t$ rosrun image_publisher image_publisher <filename>");
    return 1;
  }

  ros::param::set("~filename", argv[1]);
  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv(argv + 1, argv + argc);
  my_argv.push_back("--shutdown-on-close"); // Internal

  manager.load(ros::this_node::getName(), "image_publisher/image_publisher", remappings, my_argv);

  ros::spin();
  return 0;
}
