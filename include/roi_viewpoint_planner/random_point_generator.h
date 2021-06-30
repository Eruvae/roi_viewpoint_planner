#ifndef RANDOM_POINT_GENERATOR_H
#define RANDOM_POINT_GENERATOR_H

#include "earcut.hpp"
#include <random>
#include <array>
#include <vector>
#include <geometry_msgs/Point32.h>

namespace random_point_generator {

// define types

template<typename Point>
using Triangle = std::array<Point, 3>;

template<typename Point>
using TriangleList = std::vector<Triangle<Point>>;

template<typename Point>
using Polyline = std::vector<Point>;

template<typename Point>
using Polygon = std::vector<Polyline<Point>>;

using namespace util;

template<typename Point>
Point operator-(const Point &p1, const Point &p2)
{
  return pt<Point>::create(nth<0, Point>::get(p1) - nth<0, Point>::get(p2), nth<1, Point>::get(p1) - nth<1, Point>::get(p2));
}

template<typename Point>
Point operator+(const Point &p1, const Point &p2)
{
  return pt<Point>::create(nth<0, Point>::get(p1) + nth<0, Point>::get(p2), nth<1, Point>::get(p1) + nth<1, Point>::get(p2));
}

template<typename Point>
Point operator*(double s, const Point &p)
{
  return pt<Point>::create(s * nth<0, Point>::get(p), s * nth<1, Point>::get(p));
}

template<typename Point>
Point operator*(const Point &p, double s)
{
  return pt<Point>::create(s * nth<0, Point>::get(p), s * nth<1, Point>::get(p));
}

template<typename Point>
double abs(const Point &p)
{
  return std::sqrt(nth<0, Point>::get(p)*nth<0, Point>::get(p) + nth<1, Point>::get(p)*nth<1, Point>::get(p));
}

template<typename Point>
double dist(const Point &p1, const Point &p2)
{
  return abs(p1 - p2);
}

template<typename Point>
const Point& getIth(const Polygon<Point> &poly, uint32_t index)
{
  for (const Polyline<Point> &line : poly)
  {
    if (index >= line.size())
      index -= line.size();
    else
      return line[index];
  }
  throw std::out_of_range("Index not contained by polygon");
}

template<typename Point>
double area(const Triangle<Point> &tri)
{
  auto v1 = nth<0, Point>::get(tri[0]) * (nth<1, Point>::get(tri[1]) - nth<1, Point>::get(tri[2]));
  auto v2 = nth<0, Point>::get(tri[1]) * (nth<1, Point>::get(tri[2]) - nth<1, Point>::get(tri[0]));
  auto v3 = nth<0, Point>::get(tri[2]) * (nth<1, Point>::get(tri[0]) - nth<1, Point>::get(tri[1]));
  return std::abs((v1 + v2 + v3) / 2);
}

template<typename Point>
class PolygonRandomPointGenerator
{
private:
  TriangleList<Point> triangles;
  std::vector<double> areas;
  std::discrete_distribution<uint32_t> triangle_selector;
  std::uniform_real_distribution<double> distrib01;
  std::default_random_engine generator;

  Point getRandomPointInTriangle(const Triangle<Point> &tri)
  {
    Point v1 = tri[1] - tri[0];
    Point v2 = tri[2] - tri[0];
    double s1 = distrib01(generator);
    double s2 = distrib01(generator);
    if (s1 + s2 > 1)
    {
      s1 = 1.0 - s1;
      s2 = 1.0 - s2;
    }
    return tri[0] + s1 * v1 + s2 * v2;
  }

public:
  PolygonRandomPointGenerator(const Polygon<Point> &poly) : distrib01(0.0, 1.0), generator(std::random_device{}())
  {
    std::vector<uint32_t> triangle_inds = earcut(poly);
    for (uint32_t i = 0; i < triangle_inds.size(); i += 3)
    {
      Triangle<Point> tri = {getIth(poly, triangle_inds[i]), getIth(poly, triangle_inds[i+1]), getIth(poly, triangle_inds[i+2])};
      triangles.push_back(tri);
      areas.push_back(area(tri));
    }
    triangle_selector = std::discrete_distribution<uint32_t>{areas.begin(), areas.end()};
  }

  PolygonRandomPointGenerator(const Polyline<Point> &poly) : PolygonRandomPointGenerator(Polygon<Point>{poly}) {}

  Point getRandomPoint()
  {
    uint32_t selected_triangle = triangle_selector(generator);
    return getRandomPointInTriangle(triangles[selected_triangle]);
  }
};

} // namespace random_point_generator

#endif // RANDOM_POINT_GENERATOR_H
