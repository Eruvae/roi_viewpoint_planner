#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

namespace trolley_remote
{

enum Command
{
  MOVE_FORWARD = 0,
  MOVE_BACKWARD = 1,
  MOVE_UP = 2,
  MOVE_DOWN = 3,
  OIL_CHOKE = 4,
  SET_BEEPER = 5,
  SET_TIMEOUT = 6,
  RESET_POSITION = 7,
  RESET_HEIGHT = 8,
  SET_VELOCITY = 9,
  MOVE_TO = 10,
  LIFT_TO = 11,
  STOP_ALL = 12,
  SET_WAKE_INTERVAL = 13,
  SET_CMD_SPACING = 14,
  REQUEST_UPDATE = 15,
  TRY_RECONNECT = 16,
  CONFIG_INFO = 17
};

enum Topic
{
  POSITION = 0,
  HEIGHT = 1,
  STATUS = 2,
  ODOMETRY = 3,
  COMMANDS = 4
};

static inline std::string topicName(Topic t)
{
  switch(t)
  {
  case POSITION: return "/trollomatic/position";
  case HEIGHT: return "/trollomatic/height";
  case STATUS: return "/trollomatic/status";
  case ODOMETRY: return "/trollomatic/odom";
  case COMMANDS: return "/trollomatic/trolley_commands";
  default: return "";
  }
}

class TrolleyRemote
{
public:
  TrolleyRemote(const ros::NodeHandle &nodeHandle = ros::NodeHandle(), const ros::NodeHandle privateNodeHandle = ros::NodeHandle("~"));

  double getPosition();
  double getHeight();
  const std::string& getStatus();
  bool isReady();
  void sendCommand(Command cmd, float value = 0.f);
  void moveForward(bool enable = true);
  void moveBackward(bool enable = true);
  void moveUp(bool enable = true);
  void moveDown(bool enable = true);
  void oilChoke(bool enable = true);
  void setBeeper(bool enable = true);
  void setTimeout(float value_ms = 1500);
  void configInfo(bool enable);
  void resetPosition();
  void resetHeight();
  void setVelocity(float value = 0);
  void stopAll();
  void setWakeInterval(float value_ms = 400);
  void setCmdSpacing(float value_us = 200000);
  void moveTo(float position = 0);
  void liftTo(float height = 0);
  void tryReconnect();
  void changeRow(int row_number);

private:
  ros::NodeHandle nh;
  ros::NodeHandle nhp;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  tf2_ros::StaticTransformBroadcaster staticTfBroadcaster;
  ros::Subscriber status_sub;
  ros::Publisher cmd_pub;

  double position;
  double height;
  std::string status;
  bool ready;
  float row_length;
  float row_separation;

  void updatePositionTransform();
  void updateHeightTransform();

  void statusCallback(const std_msgs::StringConstPtr &status);
};

} // namespace trolley_remote
