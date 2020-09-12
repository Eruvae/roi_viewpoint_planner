#include <ros/ros.h>
#include <ros/package.h>

#include "viewpoint_planner.h"
#include "roi_viewpoint_planner_msgs/ChangePlannerMode.h"
#include "roi_viewpoint_planner_msgs/SaveOctomap.h"
#include "roi_viewpoint_planner_msgs/LoadOctomap.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <boost/algorithm/string/predicate.hpp>

#include <dynamic_reconfigure/server.h>
#include "roi_viewpoint_planner_msgs/PlannerConfig.h"

#include "octomap_vpp/marching_cubes.h"
#include "trolley_remote/trolley_remote.h"

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

bool changePlannerMode(roi_viewpoint_planner_msgs::ChangePlannerMode::Request &req, roi_viewpoint_planner_msgs::ChangePlannerMode::Response &res)
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

bool saveOctomap(roi_viewpoint_planner_msgs::SaveOctomap::Request &req, roi_viewpoint_planner_msgs::SaveOctomap::Response &res)
{
  res.filename = planner->saveOctomap();
  res.success = res.filename != "";
  return true;
}

bool loadOctomap(roi_viewpoint_planner_msgs::LoadOctomap::Request &req, roi_viewpoint_planner_msgs::LoadOctomap::Response &res)
{
  int err_code = planner->loadOctomap(req.filename);
  res.success = (err_code == 0);
  if (err_code == -1) res.error_message = "Deserialization failed";
  else if (err_code == -2) res.error_message = "Wrong Octree type";
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
  if (level & (1 << 5)) // insert_occ_if_not_moved
  {
    planner->insert_occ_if_not_moved = config.insert_occ_if_not_moved;
  }
  if (level & (1 << 6)) // insert_roi_if_not_moved
  {
    planner->insert_roi_if_not_moved = config.insert_roi_if_not_moved;
  }
  if (level & (1 << 7)) // insert_occ_while_moving
  {
    planner->insert_occ_while_moving = config.insert_occ_while_moving;
  }
  if (level & (1 << 8)) // insert_roi_while_moving
  {
    planner->insert_roi_while_moving = config.insert_roi_while_moving;
  }
  if (level & (1 << 9)) // wait_for_occ_scan
  {
    planner->wait_for_occ_scan = config.wait_for_occ_scan;
  }
  if (level & (1 << 10)) // wait_for_roi_scan
  {
    planner->wait_for_roi_scan = config.wait_for_occ_scan;
  }
  if (level & (1 << 11)) // publish_planning_state
  {
    planner->publish_planning_state = config.publish_planning_state;
  }
  if (level & (1 << 12)) // planner
  {
    planner->setPlannerId(config.planner);
  }
  if (level & (1 << 13)) // planning_time
  {
    planner->setPlanningTime(config.planning_time);
  }
  if (level & (1 << 14)) // use_cartesian_motion
  {
    planner->use_cartesian_motion = config.use_cartesian_motion;
  }
  if (level & (1 << 15)) // compute_ik_when_sampling
  {
    planner->compute_ik_when_sampling = config.compute_ik_when_sampling;
  }
  if (level & (1 << 16)) // velocity_scaling
  {
    planner->setMaxVelocityScalingFactor(config.velocity_scaling);
  }
  if (level & (1 << 17)) // record_map_updates
  {
    planner->record_map_updates = config.record_map_updates;
  }
  if (level & (1 << 18)) // record_viewpoints
  {
    planner->record_viewpoints = config.record_viewpoints;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_viewpoint_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  /*trolley_remote::TrolleyRemote trm(nh, nhp);
  ROS_INFO_STREAM("Trolley status: " << trm.getStatus());
  ROS_INFO_STREAM("Current position/height: " << trm.getPosition() << ", " << trm.getHeight());*/

  double tree_resolution = 0.01;
  if (nhp.hasParam("tree_resolution"))
    nhp.getParam("tree_resolution", tree_resolution);
  else
    nhp.setParam("tree_resolution", 0.01);

  std::string wstree_default_package = ros::package::getPath("phenorob_ur5e");
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/ur_retractable/workspace_map.ot");
  std::string sampling_tree_file = nhp.param<std::string>("sampling_tree", wstree_default_package + "/workspace_trees/ur_retractable/inflated_workspace_map.ot");
  std::string map_frame = nhp.param<std::string>("map_frame", "world");
  std::string ws_frame = nhp.param<std::string>("ws_frame", "arm_base_link");

  planner = new ViewpointPlanner(nh, nhp, wstree_file, sampling_tree_file, tree_resolution, map_frame, ws_frame);
  ros::ServiceServer changePlannerModeService = nhp.advertiseService("change_planner_mode", changePlannerMode);
  ros::ServiceServer activatePlanExecutionService = nhp.advertiseService("activate_plan_execution", activatePlanExecution);
  ros::ServiceServer saveTreeAsObjService = nhp.advertiseService("save_tree_as_obj", saveTreeAsObj);
  ros::ServiceServer saveOctomapService = nhp.advertiseService("save_octomap", saveOctomap);
  ros::ServiceServer loadOctomapService = nhp.advertiseService("load_octomap", loadOctomap);

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

  delete planner;
}
