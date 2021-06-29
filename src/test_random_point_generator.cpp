#include "roi_viewpoint_planner/random_point_generator.h"
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber polygonSub;
ros::Publisher pointsPub;

void polygonCallback(const geometry_msgs::PolygonStampedConstPtr &msg)
{
  if (msg->polygon.points.size() < 3)
  {
    ROS_ERROR_STREAM("At least 3 points are required for triangulation");
    return;
  }
  random_point_generator::Polyline simplePoly;
  for (const auto &point : msg->polygon.points)
  {
    simplePoly.push_back({point.x, point.y});
  }
  random_point_generator::Polygon poly {simplePoly};

  random_point_generator::PolygonRandomPointGenerator point_gen(poly);

  visualization_msgs::Marker marker;
  marker.header = msg->header;
  marker.ns = "polygon_sampled_points";
  marker.id = 0;
  marker.type = marker.POINTS;
  marker.action = marker.ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  for (size_t i = 0; i < 1000; i++)
  {
    random_point_generator::Point p = point_gen.getRandomPoint();
    geometry_msgs::Point p_msg;
    p_msg.x = p[0]; p_msg.y = p[1];
    marker.points.push_back(p_msg);
  }
  pointsPub.publish(marker);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_random_point_generator");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  pointsPub = nhp.advertise<visualization_msgs::Marker>("points", 1);
  polygonSub = nh.subscribe("/polygon", 1, polygonCallback);

  ros::spin();
}
