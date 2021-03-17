#include "planner_interfaces/external_planner_interface.h"

#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <std_srvs/Trigger.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>

namespace roi_viewpoint_planner
{

ExternalPlannerInterface::ExternalPlannerInterface(ros::NodeHandle &nh, int planning_mode, double tree_resolution)
  : configClient("/roi_viewpoint_planner"), planning_mode(planning_mode), planningTree(new octomap_vpp::RoiOcTree(tree_resolution))
{
  octomap_sub = nh.subscribe("/octomap", 10, &ExternalPlannerInterface::octomapCallback, this);

  saveOctomapClient = nh.serviceClient<roi_viewpoint_planner_msgs::SaveOctomap>("/roi_viewpoint_planner/save_octomap");
  resetPlannerClient = nh.serviceClient<std_srvs::Trigger>("/roi_viewpoint_planner/reset_planner");
}

void ExternalPlannerInterface::octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap_vpp::RoiOcTree* newRoiTree = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
    if(newRoiTree){
      //tree_mtx.lock();
      planningTree.reset(newRoiTree);
      //tree_mtx.unlock();
    }
    else {
      ROS_ERROR("Wrong octree type received; expecting ROI octree");
      delete tree;
    }
  }
  else
  {
    ROS_ERROR("Failed to deserialize octree message");
  }
}

std::shared_ptr<octomap_vpp::RoiOcTree> ExternalPlannerInterface::getPlanningTree()
{
  return planningTree;
}

boost::mutex& ExternalPlannerInterface::getTreeMutex()
{
  return tree_mtx;
}

bool ExternalPlannerInterface::activatePlanner()
{
  roi_viewpoint_planner::PlannerConfig planner_config;
  if (!configClient.getCurrentConfiguration(planner_config, ros::Duration(1)))
  {
    ROS_ERROR("Could not contact configuration server");
    return false;
  }
  planner_config.activate_execution = true;
  planner_config.require_execution_confirmation = false;
  planner_config.auto_expl_sampling = roi_viewpoint_planner::Planner_SAMPLE_CONTOURS;
  planner_config.mode = planning_mode;
  if (!configClient.setConfiguration(planner_config))
  {
    ROS_ERROR("Applying configuration not successful");
    return false;
  }
  return true;
}

bool ExternalPlannerInterface::stopPlanner()
{
  roi_viewpoint_planner::PlannerConfig planner_config;
  if (!configClient.getCurrentConfiguration(planner_config, ros::Duration(1)))
  {
    ROS_ERROR("Could not contact configuration server");
    return false;
  }
  planner_config.activate_execution = false;
  planner_config.mode = roi_viewpoint_planner::Planner_IDLE;
  if (!configClient.setConfiguration(planner_config))
  {
    ROS_ERROR("Applying configuration not successful");
    return false;
  }
  return true;
}

bool ExternalPlannerInterface::resetPlanner()
{
  std_srvs::Trigger srv_pr;
  if (resetPlannerClient.call(srv_pr))
  {
    if (!srv_pr.response.success)
    {
      ROS_WARN("Planner reset not successful");
      return false;
    }
  }
  else
  {
    ROS_WARN("Planner reset service could not be called");
    return false;
  }
  return true;
}

bool ExternalPlannerInterface::saveOctree(const std::string &filename)
{
  roi_viewpoint_planner_msgs::SaveOctomap srv_sm;
  srv_sm.request.specify_filename = true;
  srv_sm.request.name_is_prefix = false;
  srv_sm.request.name = filename;
  if (saveOctomapClient.call(srv_sm))
  {
    if (!srv_sm.response.success)
    {
      ROS_WARN("Map could not be saved");
      return false;
    }
  }
  else
  {
    ROS_WARN("Map save service could not be called");
    return false;
  }
  return true;
}

void ExternalPlannerInterface::clearOctree()
{
  tree_mtx.lock();
  planningTree->clear();
  planningTree->clearRoiKeys();
  tree_mtx.unlock();
}

}
