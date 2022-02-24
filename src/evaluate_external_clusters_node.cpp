#include <ros/ros.h>
#include "roi_viewpoint_planner/evaluator_external_clusters.h"

using namespace roi_viewpoint_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_external_clusters");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ExternalClusterEvaluator evaluator;
  ros::waitForShutdown();
}
