#include <ros/ros.h>

#include "viewpoint_planner.h"
#include "roi_viewpoint_planner/ChangePlannerMode.h"
#include <std_srvs/SetBool.h>
#include <boost/algorithm/string/predicate.hpp>

ViewpointPlanner *planner;

bool activatePlanExecution(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  planner->execute_plan = req.data;
  res.success = true;
  return true;
}

bool changePlannerMode(roi_viewpoint_planner::ChangePlannerMode::Request &req, roi_viewpoint_planner::ChangePlannerMode::Response &res)
{
  if (req.mode < ViewpointPlanner::NUM_MODES)
  {
    planner->mode = (ViewpointPlanner::PlannerMode)req.mode;
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_viewpoint_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  planner = new ViewpointPlanner(nh, argc, argv);
  ros::ServiceServer changePlannerModeService = nhp.advertiseService("change_planner_mode", changePlannerMode);
  ros::ServiceServer activatePlanExecutionService = nhp.advertiseService("activate_plan_execution", activatePlanExecution);
  planner->plannerLoop();
}
