#include <ros/ros.h>

#include "viewpoint_planner.h"
#include "roi_viewpoint_planner/ChangePlannerMode.h"
#include <std_srvs/SetBool.h>
#include <boost/algorithm/string/predicate.hpp>

#include <dynamic_reconfigure/server.h>
#include "roi_viewpoint_planner/PlannerConfig.h"

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

void reconfigureCallback(roi_viewpoint_planner::PlannerConfig &config, uint32_t level)
{
  if (level & (1 << 0) && config.mode >= 0 && config.mode < ViewpointPlanner::NUM_MODES) // change mode
  {
    planner->mode = (ViewpointPlanner::PlannerMode) config.mode;
  }
  if (level & (1 << 1)) // activate execution
  {
    planner->execute_plan = config.activate_execution;
  }
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

  dynamic_reconfigure::Server<roi_viewpoint_planner::PlannerConfig> server;

  server.setCallback(reconfigureCallback);

  planner->plannerLoop();
}
