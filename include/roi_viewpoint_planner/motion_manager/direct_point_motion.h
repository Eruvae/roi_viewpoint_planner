#pragma once

#include <ros/ros.h>

#include "motion_manager_base.h"

namespace roi_viewpoint_planner
{

class DirectPointMotion : public MotionManagerBase
{
private:
  ros::Publisher motion_pub;

public:
  DirectPointMotion(ViewpointPlanner *parent, const std::string &pose_reference_frame = "world", const std::string &ee_link_name = "camera_link");

  bool moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);
};

} // namespace roi_viewpoint_planner
