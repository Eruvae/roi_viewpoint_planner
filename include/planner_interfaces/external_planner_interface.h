#ifndef EXTERNAL_PLANNER_INTERFACE_H
#define EXTERNAL_PLANNER_INTERFACE_H

#include "planner_interface.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <octomap_msgs/Octomap.h>

namespace roi_viewpoint_planner
{

class ExternalPlannerInterface : public PlannerInterface
{
private:
  ros::Subscriber octomap_sub;
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient resetPlannerClient;
  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> configClient;
  int planning_mode;

  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  boost::mutex tree_mtx;

  void octomapCallback(const octomap_msgs::OctomapConstPtr& msg);

public:
  ExternalPlannerInterface(ros::NodeHandle &nh, int planning_mode, double tree_resolution);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual boost::mutex& getTreeMutex();

  bool activatePlanner();
  bool stopPlanner();
  bool resetPlanner();
  bool saveOctree(const std::string &filename);
  void clearOctree();
};

}

#endif // EXTERNAL_PLANNER_INTERFACE_H
