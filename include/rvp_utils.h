#ifndef RVP_UTILS_H
#define RVP_UTILS_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

static double doubleVecDiff(const std::vector<double> &v1, const std::vector<double> &v2)
{
  double res = 0;
  for (size_t i=0; i < v1.size(); i++)
  {
    res += std::abs(v2[i] - v1[i]);
  }
  return res;
}

static double computeTrajectoryLength(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
  double traj_length = 0;
  const std::vector<double> *last_pos = &plan.start_state_.joint_state.position;
  for (const trajectory_msgs::JointTrajectoryPoint &point : plan.trajectory_.joint_trajectory.points)
  {
    const std::vector<double> *point_pos = &point.positions;
    traj_length += doubleVecDiff(*last_pos, *point_pos);
    last_pos = point_pos;
  }
  return traj_length;
}

inline static double getTrajectoryDuration(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
  return plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
}

#endif // RVP_UTILS_H
