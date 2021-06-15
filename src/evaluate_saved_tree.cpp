#include <ros/ros.h>
#include "roi_viewpoint_planner/evaluator.h"

#include "roi_viewpoint_planner/planner_interfaces/saved_tree_interface.h"

using namespace roi_viewpoint_planner;

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

  std::string world_name = nh.param<std::string>("/world_name", "");

  //ROS_INFO_STREAM("World_name: " << world_name);

  std::shared_ptr<SavedTreeInterface> interface(new SavedTreeInterface(tree_resolution));
  if (!interface->readOctree(TREE_FILENAME))
  {
    ROS_ERROR("Octree file could not be read");
    return -2;
  }

  Evaluator evaluator(interface, nh, nhp, world_name != "");


  const EvaluationParameters& params = evaluator.processDetectedRois();
  //ROS_INFO_STREAM("Detected ROIs: " << params.total_roi_clusters);

  for(ros::Rate rate(1); ros::ok(); rate.sleep());
}
