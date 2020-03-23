#include <ros/ros.h>

#include "viewpoint_planner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_viewpoint_planner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ViewpointPlanner planner(nh, argc, argv);
  planner.plannerLoop();
}
