#pragma once

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_interface/planning_interface.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "motion_manager_base.h"

namespace roi_viewpoint_planner
{

class ViewpointPlanner;

using moveit::planning_interface::MoveGroupInterface;
using moveit::core::MoveItErrorCode;

class RobotManager : public MotionManagerBase
{
private:
  const std::string group_name;
  MoveGroupInterface manipulator_group;
  //robot_model_loader::RobotModelLoaderPtr rml;
  //planning_scene_monitor::PlanningSceneMonitorPtr psm;
  //robot_model::RobotModelConstPtr kinematic_model;
  //const robot_state::JointModelGroup* jmg;
  //robot_state::RobotStatePtr kinematic_state;

  bool planAndExecuteFromMoveGroup(bool async, bool safe);

  bool safeExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);

  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);

public:
  RobotManager(ViewpointPlanner *parent, const std::string &pose_reference_frame = "world", const std::string &end_effector_link = "camera_link",
               const std::string& group_name = "manipulator", const std::string &robot_description_param_name = "robot_description");

  // Set configurable planner parameters
  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz);
  void setPlannerId(const std::string& planner_id);
  void setPlanningTime(double seconds);
  void setMaxVelocityScalingFactor(double max_velocity_scaling_factor);

  bool getJointValuesFromPose(const geometry_msgs::Pose &pose, std::vector<double> &joints);

  bool moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToState(const std::vector<double> &joint_values, bool async=false, bool safe=true);
};

} // namespace view_motion_planner
