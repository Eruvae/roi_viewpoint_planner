#ifndef RVP_TYPES_H
#define RVP_TYPES_H

#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/robot_state.h>
#include <octomap/octomap_types.h>

namespace roi_viewpoint_planner
{

struct Viewpoint
{
  geometry_msgs::Pose pose;
  robot_state::RobotStatePtr joint_target;
  octomap::point3d target;
  double infoGain;
  double distance;
  double utility;
  bool isFree;
};

enum class SamplerType
{
    ROI_CONTOUR_SAMPLER=3, ROI_ADJACENT_SAMPLER=4, ROI_CENTER_SAMPLER=5, EXPLORATION_SAMPLER=6, CONTOUR_SAMPLER=7, BORDER_SAMPLER=8
};

enum class UtilityType
{
    SINGLE_RAY_UTILITY=0, MULTI_RAY_UTILITY=1, ROI_VICINITY_UTILITY=2, ROI_OCCLUSION_UTILITY=3
};

} // namespace roi_viewpoint_planner

#endif // RVP_TYPES_H
