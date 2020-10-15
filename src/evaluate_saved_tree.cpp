#include <ros/ros.h>
#include "evaluator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_saved_tree");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  if (argc < 2)
  {
    ROS_ERROR("Octree not specified");
    return -1;
  }

  const std::string TREE_FILENAME = argv[1];

  double tree_resolution = nh.param<double>("/roi_viewpoint_planner/tree_resolution", 0.01);
  std::string world_name = nh.param<std::string>("/world_name", "world14");

  Evaluator evaluator(nh, nhp, true, world_name, tree_resolution);
  if (!evaluator.readOctree(TREE_FILENAME))
  {
    ROS_ERROR("Octree file could not be read");
    return -2;
  }

  evaluator.processDetectedRois();

  for(ros::Rate rate(1); ros::ok(); rate.sleep());
}
