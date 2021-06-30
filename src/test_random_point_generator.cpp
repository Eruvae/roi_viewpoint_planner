#include "roi_viewpoint_planner/random_point_generator.h"
#include <ros/ros.h>
#include <octomap/octomap_types.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber polygonSub;
ros::Publisher pointsPub;

namespace random_point_generator {
namespace util {

template <>
struct nth<0, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_x_type get(const geometry_msgs::Point32 &t) {
        return t.x;
    }
};

template <>
struct nth<1, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_y_type get(const geometry_msgs::Point32 &t) {
        return t.y;
    }
};

template <>
struct pt<geometry_msgs::Point32> {
    inline static geometry_msgs::Point32 create(const geometry_msgs::Point32::_x_type &x, const geometry_msgs::Point32::_y_type &y) {
      geometry_msgs::Point32 p;
      p.x = x;
      p.y = y;
      return p;
    }
};

template <>
struct nth<0, octomap::point3d> {
    inline static float get(const octomap::point3d &t) {
        return t.x();
    }
};

template <>
struct nth<1, octomap::point3d> {
    inline static float get(const octomap::point3d &t) {
        return t.y();
    }
};

template <>
struct pt<octomap::point3d> {
    inline static octomap::point3d create(const float &x, const float &y) {
      return {x, y, 0};
    }
};

} // namespace util
} // namespace random_point_generator

//using Point = std::array<double, 2>;
//using Point = octomap::point3d;
using Point = geometry_msgs::Point32;

void polygonCallback(const geometry_msgs::PolygonStampedConstPtr &msg)
{
  if (msg->polygon.points.size() < 3)
  {
    ROS_ERROR_STREAM("At least 3 points are required for triangulation");
    return;
  }

  /*random_point_generator::Polyline<Point> simplePoly;
  for (const auto &point : msg->polygon.points)
  {
    simplePoly.push_back(random_point_generator::pt<Point>::create(point.x, point.y));
  }
  random_point_generator::PolygonRandomPointGenerator<Point> point_gen(simplePoly);*/

  random_point_generator::PolygonRandomPointGenerator<Point> point_gen(msg->polygon.points);

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
    Point p = point_gen.getRandomPoint();
    geometry_msgs::Point p_msg;
    p_msg.x = random_point_generator::nth<0, Point>::get(p);
    p_msg.y = random_point_generator::nth<1, Point>::get(p);
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
