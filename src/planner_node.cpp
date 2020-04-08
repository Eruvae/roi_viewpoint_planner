#include <ros/ros.h>
#include <ros/package.h>

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
  ROS_INFO_STREAM("Reconfigure callback called");
  if (level & (1 << 0) && config.mode >= 0 && config.mode < ViewpointPlanner::NUM_MODES) // change mode
  {
    planner->mode = (ViewpointPlanner::PlannerMode) config.mode;
  }
  if (level & (1 << 1)) // activate execution
  {
    planner->execute_plan = config.activate_execution;
  }
  if (level & (1 << 2)) // request execution confirmation
  {
    planner->require_execution_confirmation = config.require_execution_confirmation;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_viewpoint_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string wstree_default_package = ros::package::getPath("phenorob_ur5e");
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/ur_with_cam/workspace_map.ot");

  planner = new ViewpointPlanner(nh, nhp, wstree_file);
  ros::ServiceServer changePlannerModeService = nhp.advertiseService("change_planner_mode", changePlannerMode);
  ros::ServiceServer activatePlanExecutionService = nhp.advertiseService("activate_plan_execution", activatePlanExecution);

  dynamic_reconfigure::Server<roi_viewpoint_planner::PlannerConfig> server(nhp);

  server.setCallback(reconfigureCallback);

  planner->plannerLoop();
}
