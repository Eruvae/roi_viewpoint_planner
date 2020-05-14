#include <ros/ros.h>
#include <ros/package.h>

#include "viewpoint_planner.h"
#include "roi_viewpoint_planner/ChangePlannerMode.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <boost/algorithm/string/predicate.hpp>

#include <dynamic_reconfigure/server.h>
#include "roi_viewpoint_planner/PlannerConfig.h"

#include "octomap_vpp/marching_cubes.h"

ViewpointPlanner *planner;

bool activatePlanExecution(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  planner->execute_plan = req.data;
  res.success = true;
  return true;
}

bool saveTreeAsObj(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = planner->saveTreeAsObj("planning_tree.obj");
  planner->saveROIsAsObj("roi_mesh.obj");
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
  if (level & (1 << 3)) // minimum sensor range
  {
    planner->sensor_min_range = config.sensor_min_range;
    if (planner->sensor_max_range < planner->sensor_min_range && !(level & (1 << 4)))
    {
      planner->sensor_max_range = planner->sensor_min_range;
      config.sensor_max_range = planner->sensor_max_range;
    }
  }
  if (level & (1 << 4)) // maximum sensor range
  {
    planner->sensor_max_range = config.sensor_max_range;
    if (planner->sensor_min_range > planner->sensor_max_range)
    {
      planner->sensor_min_range = planner->sensor_max_range;
      config.sensor_min_range = planner->sensor_min_range;
    }
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
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/ur_retractable/workspace_map.ot");

  planner = new ViewpointPlanner(nh, nhp, wstree_file);
  ros::ServiceServer changePlannerModeService = nhp.advertiseService("change_planner_mode", changePlannerMode);
  ros::ServiceServer activatePlanExecutionService = nhp.advertiseService("activate_plan_execution", activatePlanExecution);
  ros::ServiceServer saveTreeAsObjService = nhp.advertiseService("save_tree_as_obj", saveTreeAsObj);

  dynamic_reconfigure::Server<roi_viewpoint_planner::PlannerConfig> server(nhp);

  server.setCallback(reconfigureCallback);

  std::vector<double> joint_start_values;
  if(nhp.getParam("initial_joint_values", joint_start_values))
  {
    planner->moveToState(joint_start_values);
  }
  else
  {
    ROS_WARN("No inital joint values set");
  }

  planner->plannerLoop();
}
