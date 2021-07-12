#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_vpp/RoiOcTree.h>

octomap_vpp::RoiOcTree::StaticMemberInitializer octomap_vpp::RoiOcTree::ocTreeMemberInit;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_color_octree");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  const std::string map_frame = nhp.param<std::string>("map_frame", "world");
  const std::string file = nhp.param<std::string>("file", "tree.ot");
  const std::string topic = nhp.param<std::string>("topic", "/loaded_octree");

  ros::Publisher octomap_pub = nhp.advertise<octomap_msgs::Octomap>(topic, 1, true);

  octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(file);
  if (!tree)
    return -1;

  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  bool msg_generated = octomap_msgs::fullMapToMsg(*tree, map_msg);
  if (msg_generated)
  {
    octomap_pub.publish(map_msg);
  }

  ros::spin();

}
