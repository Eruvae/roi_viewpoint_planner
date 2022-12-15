#include "roi_viewpoint_planner/motion_manager/direct_point_motion.h"

namespace roi_viewpoint_planner
{

DirectPointMotion::DirectPointMotion(ViewpointPlanner *parent, const std::string &pose_reference_frame, const std::string &end_effector_link)
  : MotionManagerBase(parent, pose_reference_frame, end_effector_link)
{
  ros::NodeHandle nh;
  motion_pub = nh.advertise<geometry_msgs::Pose>("/TODO/POSE_TOPIC", 1);
}

bool DirectPointMotion::moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  motion_pub.publish(goal_pose);
  return true;
}

bool DirectPointMotion::moveToPose(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  motion_pub.publish(goal_pose);
  return true;
}

} // namespace roi_viewpoint_planner
