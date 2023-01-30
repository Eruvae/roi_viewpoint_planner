#include <ros/ros.h>
#include <ros/package.h>

#include "roi_viewpoint_planner/viewpoint_planner.h"
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/MoveToState.h>
#include <roi_viewpoint_planner_msgs/RandomizePlantPositions.h>
#include <roi_viewpoint_planner_msgs/ResetPlanner.h>
#include <roi_viewpoint_planner_msgs/LoadOctomap.h>
#include <roi_viewpoint_planner_msgs/StartEvaluator.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string/predicate.hpp>

#include <dynamic_reconfigure/server.h>
#include "roi_viewpoint_planner_msgs/PlannerConfig.h"

using namespace roi_viewpoint_planner;

ros::NodeHandle *nhp_pt;
ViewpointPlanner *planner;

bool startEvaluator(roi_viewpoint_planner_msgs::StartEvaluator::Request &req, roi_viewpoint_planner_msgs::StartEvaluator::Response &res)
{
  if (req.episode_end_param >= static_cast<uint8_t>(roi_viewpoint_planner::EvalEpisodeEndParam::NUM_EPEND_PARAMS))
  {
    res.success = false;
  }
  else
  {
    roi_viewpoint_planner::EvalEpisodeEndParam epEndParam = static_cast<roi_viewpoint_planner::EvalEpisodeEndParam>(req.episode_end_param);
    res.success = planner->startEvaluator(req.num_evals, epEndParam, req.episode_duration, req.starting_index,
                  req.randomize_plants, octomap_vpp::pointToOctomath(req.min_point), octomap_vpp::pointToOctomath(req.max_point), req.min_dist);
  }
  return true;
}

bool randomizePlantPositions(roi_viewpoint_planner_msgs::RandomizePlantPositions::Request &req, roi_viewpoint_planner_msgs::RandomizePlantPositions::Response &res)
{
  res.success = planner->randomizePlantPositions(req.min_point, req.max_point, req.min_dist);
  return true;
}

bool saveTreeAsObj(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = planner->saveTreeAsObj("planning_tree.obj");
  planner->saveROIsAsObj("roi_mesh.obj");
  return true;
}

bool saveOctomap(roi_viewpoint_planner_msgs::SaveOctomap::Request &req, roi_viewpoint_planner_msgs::SaveOctomap::Response &res)
{
  if (req.specify_filename)
    res.filename = planner->saveOctomap(req.name, req.name_is_prefix);
  else
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

bool resetOctomap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  planner->resetOctomap();
  return true;
}

bool resetPlanner(roi_viewpoint_planner_msgs::ResetPlanner::Request &req, roi_viewpoint_planner_msgs::ResetPlanner::Response &res)
{
  planner->setMode(Planner_IDLE);

  planner->resetOctomap();

  std::vector<double> joint_start_values;
  if(nhp_pt->getParam("initial_joint_values", joint_start_values))
  {
    res.success = planner->getMotionManager()->moveToState(joint_start_values, req.async, false);
  }
  else
  {
    ROS_WARN("No inital joint values set");
  }
  return true;
}

bool moveToState(roi_viewpoint_planner_msgs::MoveToState::Request &req, roi_viewpoint_planner_msgs::MoveToState::Response &res)
{
  res.success = planner->getMotionManager()->moveToState(req.joint_values, req.async, false);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roi_viewpoint_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  nhp_pt = &nhp;
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

  std::string wstree_default_package = ros::package::getPath("ur_with_cam_gazebo");
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/static/workspace_map.ot");
  std::string sampling_tree_file = nhp.param<std::string>("sampling_tree", wstree_default_package + "/workspace_trees/static/inflated_ws_tree.ot");
  std::string map_frame = nhp.param<std::string>("map_frame", "world");
  std::string ws_frame = nhp.param<std::string>("ws_frame", "arm_base_link");
  bool update_planning_tree = nhp.param<bool>("update_planning_tree", true);
  bool initialize_evaluator = nhp.param<bool>("initialize_evaluator", true);

  planner = new ViewpointPlanner(nh, nhp, wstree_file, sampling_tree_file, tree_resolution, map_frame, ws_frame, update_planning_tree, initialize_evaluator);
  ros::ServiceServer saveTreeAsObjService = nhp.advertiseService("save_tree_as_obj", saveTreeAsObj);
  ros::ServiceServer saveOctomapService = nhp.advertiseService("save_octomap", saveOctomap);
  ros::ServiceServer loadOctomapService = nhp.advertiseService("load_octomap", loadOctomap);
  ros::ServiceServer moveToStateService = nhp.advertiseService("move_to_state", moveToState);
  ros::ServiceServer randomizePlantPositionsService = nhp.advertiseService("randomize_plant_positions", randomizePlantPositions);
  ros::ServiceServer resetOctomapService = nhp.advertiseService("reset_octomap", resetOctomap);
  ros::ServiceServer resetPlannerService = nhp.advertiseService("reset_planner", resetPlanner);
  ros::ServiceServer startEvaluatorService = nhp.advertiseService("start_evaluator", startEvaluator);

  planner->getMotionManager()->moveToHomePose(false, false);

  planner->plannerLoop();

  delete planner;
}
