#ifndef COMPUTE_CUBES_H
#define COMPUTE_CUBES_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <octomap/octomap_types.h>
#include <octomap_ros/conversions.h>

//const std_msgs::ColorRGBA COLOR_RED = {.r = 1.f, .g = 0.f, .b = 0.f, .a = 1.f};
//const std_msgs::ColorRGBA COLOR_GREEN = {1.f, 1.f, 0.f, 1.f};
//const std_msgs::ColorRGBA COLOR_BLUE = {0.f, 0.f, 1.f, 1.f};

const std_msgs::ColorRGBA COLOR_RED = []{std_msgs::ColorRGBA c; c.r = 1.f; c.g = 0.f; c.b = 0.f; c.a = 1.f; return c; } ();
const std_msgs::ColorRGBA COLOR_GREEN = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 1.f; c.b = 0.f; c.a = 1.f; return c; } ();
const std_msgs::ColorRGBA COLOR_BLUE = []{std_msgs::ColorRGBA c; c.r = 0.f; c.g = 0.f; c.b = 1.f; c.a = 1.f; return c; } ();
const std_msgs::ColorRGBA COLOR_VIOLET = []{std_msgs::ColorRGBA c; c.r = 0.6f; c.g = 0.2f; c.b = 0.8f; c.a = 1.f; return c; } ();
const std_msgs::ColorRGBA COLOR_ORANGE = []{std_msgs::ColorRGBA c; c.r = 1.0f; c.g = 0.5f; c.b = 0.f; c.a = 1.f; return c; } ();
const std_msgs::ColorRGBA COLOR_BROWN = []{std_msgs::ColorRGBA c; c.r = 0.8f; c.g = 0.4f; c.b = 0.1f; c.a = 1.f; return c; } ();

static inline void addEdgesFromCorner(const geometry_msgs::Point &corner, const geometry_msgs::Point &opp, std::vector<geometry_msgs::Point> &list)
{
  geometry_msgs::Point p = corner;
  list.push_back(p);
  p.x = opp.x;
  list.push_back(p);
  p.x = corner.x;
  list.push_back(p);
  p.y = opp.y;
  list.push_back(p);
  p.y = corner.y;
  list.push_back(p);
  p.z = opp.z;
  list.push_back(p);
}

static inline void swapCoords(geometry_msgs::Point &p1, geometry_msgs::Point &p2, size_t dim)
{
  if (dim == 0)
    std::swap(p1.x, p2.x);
  else if (dim == 1)
    std::swap(p1.y, p2.y);
  else if (dim == 2)
    std::swap(p1.z, p2.z);
}


static inline void addCubeEdges(const octomap::point3d &min, const octomap::point3d &max, std::vector<geometry_msgs::Point> &list)
{
  geometry_msgs::Point corner = octomap::pointOctomapToMsg(min);
  geometry_msgs::Point opp = octomap::pointOctomapToMsg(max);
  addEdgesFromCorner(corner, opp, list);
  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = i + 1; j < 3; j++)
    {
      swapCoords(corner, opp, i);
      swapCoords(corner, opp, j);
      addEdgesFromCorner(corner, opp, list);
      swapCoords(corner, opp, i);
      swapCoords(corner, opp, j);
    }
  }
}

static inline void publishCubeVisualization(ros::Publisher &pub, const std::vector<octomap::point3d> &centers, const std::vector<octomap::point3d> &volumes,
                                            const std_msgs::ColorRGBA &col = COLOR_RED, const std::string &ns = "roi_cubes")
{
  visualization_msgs::Marker cube_msg;
  cube_msg.header.frame_id = "world";
  cube_msg.header.stamp = ros::Time::now();
  cube_msg.ns = ns;
  cube_msg.id = 0;
  cube_msg.type = visualization_msgs::Marker::LINE_LIST;
  cube_msg.color = col;
  cube_msg.scale.x = 0.01;
  cube_msg.pose.orientation.w = 1.0; // normalize quaternion

  for (size_t i = 0; i < centers.size(); i++)
  {
    octomap::point3d min = centers[i] - volumes[i] * 0.5;
    octomap::point3d max = centers[i] + volumes[i] * 0.5;
    addCubeEdges(min, max, cube_msg.points);
  }

  pub.publish(cube_msg);
}

#endif // COMPUTE_CUBES_H
