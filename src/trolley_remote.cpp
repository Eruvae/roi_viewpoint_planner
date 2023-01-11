#include "roi_viewpoint_planner/trolley_remote.h"
#include <boost/algorithm/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace trolley_remote
{

TrolleyRemote::TrolleyRemote(const ros::NodeHandle &nodeHandle, const ros::NodeHandle privateNodeHandle) :
  nh(nodeHandle), nhp(privateNodeHandle),
  tfBuffer(ros::Duration(30)),
  tfListener(tfBuffer)
{
  status_sub = nh.subscribe<std_msgs::String>(topicName(STATUS), 10, &TrolleyRemote::statusCallback, this);
  cmd_pub = nh.advertise<sensor_msgs::Joy>(topicName(COMMANDS), 1);
  sendCommand(REQUEST_UPDATE);


  /**
  Quick map transform broadcaster to change rows.
  Also includes self.changeRow() function.
  TODO: This should be done by a mapping node, and in case the row selection
  is still manual, this remote node should only send a message/service to
  trigger the row change instead of broadcasting the transform
  '''
  */
  // get row dimensions
  row_length = nhp.param<float>("row_length", 100);
  row_separation = nhp.param<float>("row_separation", 30);
  // broadcast initial transforms between standard frames
  /*geometry_msgs::TransformStamped t;
  t.header.stamp = ros::Time::now();
  t.transform.rotation.w = 1.0;
  t.header.frame_id = "world";
  t.child_frame_id = "map";
  staticTfBroadcaster.sendTransform(t);
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  staticTfBroadcaster.sendTransform(t);*/
}

double TrolleyRemote::getPosition()
{
  updatePositionTransform();
  return position;
}

double TrolleyRemote::getHeight()
{
  updateHeightTransform();
  return height;
}

const std::string& TrolleyRemote::getStatus()
{
  return status;
}

bool TrolleyRemote::isReady()
{
  return boost::iequals(status, "stopped")
      || boost::iequals(status, "reached")
      || boost::iequals(status, "ready");
}

void TrolleyRemote::sendCommand(Command cmd, float value)
{
  sensor_msgs::Joy msg;
  msg.buttons.push_back(cmd);
  msg.axes.push_back(value);
  msg.header.stamp = ros::Time::now();
  cmd_pub.publish(msg);
}

void TrolleyRemote::moveForward(bool enable)
{
  sendCommand(MOVE_FORWARD, enable);
}

void TrolleyRemote::moveBackward(bool enable)
{
  sendCommand(MOVE_BACKWARD, enable);
}

void TrolleyRemote::moveUp(bool enable)
{
  sendCommand(MOVE_UP, enable);
}

void TrolleyRemote::moveDown(bool enable)
{
  sendCommand(MOVE_DOWN, enable);
}

void TrolleyRemote::oilChoke(bool enable)
{
  sendCommand(OIL_CHOKE, enable);
}

void TrolleyRemote::setBeeper(bool enable)
{
  sendCommand(SET_BEEPER, enable);
}

void TrolleyRemote::setTimeout(float value_ms)
{
  sendCommand(SET_TIMEOUT, value_ms);
}

void TrolleyRemote::configInfo(bool enable)
{
  sendCommand(CONFIG_INFO, enable);
}

void TrolleyRemote::resetPosition()
{
  sendCommand(RESET_POSITION);
}

void TrolleyRemote::resetHeight()
{
  sendCommand(RESET_HEIGHT);
}

void TrolleyRemote::setVelocity(float value)
{
  sendCommand(SET_VELOCITY, value);
}

void TrolleyRemote::stopAll()
{
  sendCommand(STOP_ALL);
}

void TrolleyRemote::setWakeInterval(float value_ms)
{
  sendCommand(SET_WAKE_INTERVAL, value_ms);
}

void TrolleyRemote::setCmdSpacing(float value_us)
{
  sendCommand(SET_CMD_SPACING, value_us);
}

void TrolleyRemote::moveTo(float position)
{
  sendCommand(MOVE_TO, position);
}

void TrolleyRemote::liftTo(float height)
{
  sendCommand(LIFT_TO, height);
}

void TrolleyRemote::tryReconnect()
{
  sendCommand(TRY_RECONNECT);
}

/* Change the row in which the trolley operates by broadcasting a tf
between /world and /map to the beginning of the specified row, using the row
dimensions to compute it.
(assuming CKA's glasshouses numbering rows from left to right from the
entrance door).
    In practice, this is similar to having a separate map for each row.
    See note on __init__ function.

Args:
    row_number (int)
*/
void TrolleyRemote::changeRow(int row_number)
{
  // Populate transform header
  geometry_msgs::TransformStamped row_tf;
  row_tf.header.stamp = ros::Time::now();
  row_tf.header.frame_id = "world";
  row_tf.child_frame_id = "map";
  //Compute XY shift and Z rotation of the trolley according to the row number
  //and dimension
  row_tf.transform.translation.x = row_length * ((row_number - 1) % 2);
  row_tf.transform.translation.y = -row_separation * int((row_number - 1) / 2);
  tf2::Quaternion quat;
  quat.setEuler(0, 0, M_PI * ((row_number - 1) % 2));
  row_tf.transform.rotation = tf2::toMsg(quat);
  staticTfBroadcaster.sendTransform(row_tf);
}

void TrolleyRemote::updatePositionTransform()
{
  geometry_msgs::TransformStamped positionTransform;
  try
  {
    positionTransform = tfBuffer.lookupTransform("base_link", "platform_start_link", ros::Time(0), ros::Duration(1));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find position transform: " << e.what());
  }
  position = positionTransform.transform.translation.x * 1000;
}

void TrolleyRemote::updateHeightTransform()
{
  geometry_msgs::TransformStamped heightTransform;
  try
  {
    heightTransform = tfBuffer.lookupTransform("base_link", "platform_base", ros::Time(0), ros::Duration(1));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find height transform: " << e.what());
  }
  height = heightTransform.transform.translation.z * 1000;
}

void TrolleyRemote::statusCallback(const std_msgs::StringConstPtr &status)
{
  this->status = status->data;
}

}
