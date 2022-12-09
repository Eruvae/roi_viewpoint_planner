#pragma once

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace roi_viewpoint_planner
{

class ViewpointPlanner;

using moveit::planning_interface::MoveGroupInterface;
using moveit::core::MoveItErrorCode;

class RobotManager
{
private:
  ViewpointPlanner *parent;

  const std::string group_name;
  MoveGroupInterface manipulator_group;
  robot_model_loader::RobotModelLoaderPtr rml;
  planning_scene_monitor::PlanningSceneMonitorPtr psm;
  robot_model::RobotModelConstPtr kinematic_model;
  const robot_state::JointModelGroup* jmg;
  robot_state::RobotStatePtr kinematic_state;

  const std::string pose_reference_frame;
  const std::string end_effector_link;

public:
  RobotManager(ViewpointPlanner *parent, const std::string &pose_reference_frame = "world",
               const std::string &robot_description_param_name = "robot_description",
               const std::string& group_name = "manipulator", const std::string &ee_link_name = "camera_link");

  moveit::core::RobotModelConstPtr getRobotModel()
  {
    return kinematic_model;
  }

  const robot_state::JointModelGroup* getJointModelGroup()
  {
    return jmg;
  }

  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor()
  {
    return psm;
  }

  // Set planner parameters

  void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
  {
    manipulator_group.setWorkspace(minx, miny, minz, maxx, maxy, maxz);
  }

  void setPoseReferenceFrame(const std::string& pose_reference_frame)
  {
    manipulator_group.setPoseReferenceFrame(pose_reference_frame);
  }

  void setPlannerId(const std::string& planner_id)
  {
    manipulator_group.setPlannerId(planner_id);
  }

  void setPlanningTime(double seconds)
  {
    manipulator_group.setPlanningTime(seconds);
  }

  void setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
  {
    manipulator_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  }

  bool setJointValueTarget(const geometry_msgs::Pose &p)
  {
    return manipulator_group.setJointValueTarget(p, end_effector_link);
  }

  void getJointValueTarget(std::vector<double> &joints)
  {
    manipulator_group.getJointValueTarget(joints);
  }

  bool moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToState(const moveit::core::RobotStateConstPtr &goal_state, bool async=false, bool safe=true);

  bool moveToState(const std::vector<double> &joint_values, bool async=false, bool safe=true);

  bool planAndExecuteFromMoveGroup(bool async, bool safe);

  bool safeExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);

  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);
};

} // namespace view_motion_planner
