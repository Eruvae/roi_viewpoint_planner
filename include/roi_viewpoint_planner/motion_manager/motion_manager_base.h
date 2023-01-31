#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

namespace roi_viewpoint_planner
{

class ViewpointPlanner;

class MotionManagerBase
{
protected:
  ViewpointPlanner *parent;

  std::string home_pose;
  const std::string pose_reference_frame;
  const std::string end_effector_link;

public:
  MotionManagerBase(ViewpointPlanner *parent, const std::string &pose_reference_frame = "world", const std::string &end_effector_link = "camera_link")
    : parent(parent), home_pose("home"), pose_reference_frame(pose_reference_frame), end_effector_link(end_effector_link) {}

  // Set configurable planner parameters
  virtual void setWorkspace(double minx, double miny, double minz, double maxx, double maxy, double maxz) {}
  virtual void setPlannerId(const std::string& planner_id) {}
  virtual void setPlanningTime(double seconds) {}
  virtual void setMaxVelocityScalingFactor(double max_velocity_scaling_factor) {}

  virtual bool getJointValuesFromPose(const geometry_msgs::Pose &pose, std::vector<double> &joints) {return false;}

  virtual bool moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true) = 0;

  virtual bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true) = 0;

  virtual bool moveToState(const std::vector<double> &joint_values, bool async=false, bool safe=true) {return false;}

  virtual bool moveToNamedPose(const std::string &pose_name, bool async=false, bool safe=false) {return false;}

  void setHomePoseName(const std::string &name)
  {
    home_pose = name;
  }

  std::string getHomePoseName()
  {
    return home_pose;
  }

  void flipHomePose()
  {
    if (home_pose == "home")
      home_pose = "home_flipped";
    else
      home_pose = "home";
  }

  virtual bool moveToHomePose(bool async=false, bool safe=false) {return false;};
};

} // namespace view_motion_planner
