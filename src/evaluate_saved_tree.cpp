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

  Evaluator evaluator(nh, nhp);
  if (!evaluator.readOctree(TREE_FILENAME))
  {
    ROS_ERROR("Octree file could not be read");
    return -2;
  }

  evaluator.processDetectedRois();

  for(ros::Rate rate(1); ros::ok(); rate.sleep());
}
