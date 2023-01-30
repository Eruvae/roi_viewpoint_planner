#include "roi_viewpoint_planner/motion_manager/robot_manager.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rvp_evaluation/rvp_utils.h>
#include <std_srvs/Trigger.h>

#include "roi_viewpoint_planner/viewpoint_planner.h"

namespace roi_viewpoint_planner
{

RobotManager::RobotManager(ViewpointPlanner *parent, const std::string &pose_reference_frame, const std::string &end_effector_link)
  : MotionManagerBase(parent, pose_reference_frame, end_effector_link),
    nh(""),
    robot_description_param_name(nh.param<std::string>("/roi_viewpoint_planner/robot_description_param_name", "robot_description")),
    group_name(nh.param<std::string>("/roi_viewpoint_planner/group_name", "manipulator")),
    manipulator_group(MoveGroupInterface(MoveGroupInterface::Options(group_name, robot_description_param_name)))
    //rml(new robot_model_loader::RobotModelLoader(robot_description_param_name)),
    //psm(new planning_scene_monitor::PlanningSceneMonitor(rml)),
    //kinematic_model(rml->getModel()),
    //jmg(kinematic_model->getJointModelGroup(group_name)),
    //kinematic_state(new robot_state::RobotState(kinematic_model))
{
  manipulator_group.setPoseReferenceFrame(pose_reference_frame);
  manipulator_group.setEndEffectorLink(end_effector_link);
  //psm->startWorldGeometryMonitor();
  //psm->startStateMonitor("/joint_states");
  //psm->startSceneMonitor("/move_group/monitored_planning_scene");
}

void RobotManager::setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz)
{
  manipulator_group.setWorkspace(minx, miny, minz, maxx, maxy, maxz);
}

void RobotManager::setPlannerId(const std::string& planner_id)
{
  manipulator_group.setPlannerId(planner_id);
}

void RobotManager::setPlanningTime(double seconds)
{
  manipulator_group.setPlanningTime(seconds);
}

void RobotManager::setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
{
  manipulator_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
}

bool RobotManager::getJointValuesFromPose(const geometry_msgs::Pose &pose, std::vector<double> &joints)
{
  if (!manipulator_group.setJointValueTarget(pose, end_effector_link))
    return false;

  manipulator_group.getJointValueTarget(joints);
  return true;
}

bool RobotManager::moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(goal_pose);
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;
  double fraction = manipulator_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction > 0.95) // execute plan if at least 95% of goal reached
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_= trajectory;
    if (safe)
      return safeExecutePlan(plan, async);
    else
      return executePlan(plan, async);
  }
  return false;
}

bool RobotManager::moveToPose(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  ros::Time setTargetTime = ros::Time::now();
  if (!manipulator_group.setJointValueTarget(goal_pose, end_effector_link))
  {
    ROS_INFO_STREAM("Could not find IK for specified pose (Timeout: " << (ros::Time::now() - setTargetTime) << ")");
    return false;
  }
  ROS_INFO_STREAM("IK solve time: " << (ros::Time::now() - setTargetTime));

  return planAndExecuteFromMoveGroup(async, safe);
}

bool RobotManager::moveToState(const std::vector<double> &joint_values, bool async, bool safe)
{
  if (!manipulator_group.setJointValueTarget(joint_values))
  {
    ROS_INFO_STREAM("Couldn't set joint target, make sure values are in bounds");
    return false;
  }

  return planAndExecuteFromMoveGroup(async, safe);
}

bool RobotManager::moveToNamedPose(const std::string &pose_name, bool async, bool safe)
{
  manipulator_group.setNamedTarget(pose_name);
  return planAndExecuteFromMoveGroup(async, safe);
}

bool RobotManager::moveToHomePose(bool async, bool safe)
{
  manipulator_group.setNamedTarget("home");
  return planAndExecuteFromMoveGroup(async, safe);
}

bool RobotManager::planAndExecuteFromMoveGroup(bool async, bool safe)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::Time planStartTime = ros::Time::now();
  moveit::core::MoveItErrorCode res = manipulator_group.plan(plan);
  ROS_INFO_STREAM("Planning duration: " << (ros::Time::now() - planStartTime));
  if (res != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Could not find plan");
    return false;
  }
  if (safe)
    return safeExecutePlan(plan, async);
  else
    return executePlan(plan, async);
}

bool RobotManager::safeExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async)
{
  if (parent->config.require_execution_confirmation)
  {
    std_srvs::Trigger requestExecution;
    if (!parent->requestExecutionConfirmation.call(requestExecution) || !requestExecution.response.success)
    {
      ROS_INFO_STREAM("Plan execution denied");
      return false;
    }
  }

  parent->robotIsMoving.store(true);
  parent->scanInserted.store(false);
  if (parent->config.publish_planning_state)
  {
    parent->state.robot_is_moving = true;
    parent->state.scan_inserted = false;
    parent->plannerStatePub.publish(parent->state);
  }

  bool res = executePlan(plan, async);

  parent->robotIsMoving.store(false);
  if (parent->config.publish_planning_state)
  {
    parent->state.robot_is_moving = false;
    parent->plannerStatePub.publish(parent->state);
  }

  parent->timeLogger.saveTime(TimeLogger::PLAN_EXECUTED);

  /*if (!res)
  {
    ROS_INFO("Could not execute plan");
    return false;
  }*/

  if (parent->eval_running)
  {
    for (ros::Rate r(100); !parent->scanInserted; r.sleep()); // wait for scan
    parent->timeLogger.saveTime(TimeLogger::WAITED_FOR_SCAN);
    parent->saveEvaluatorData(rvp_evaluation::computeTrajectoryLength(plan), rvp_evaluation::getTrajectoryDuration(plan));
    parent->timeLogger.saveTime(TimeLogger::EVALUATED);
  }

  return res;
}

bool RobotManager::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async)
{
  moveit::core::MoveItErrorCode res;
  if (async)
    res = manipulator_group.asyncExecute(plan);
  else
    res = manipulator_group.execute(plan);

  // Ignore failed during execution error for now
  return (res == moveit::core::MoveItErrorCode::SUCCESS || res == moveit::core::MoveItErrorCode::CONTROL_FAILED);
}

} // namespace view_motion_planner
