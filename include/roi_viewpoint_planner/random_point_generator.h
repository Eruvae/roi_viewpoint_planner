#ifndef RANDOM_POINT_GENERATOR_H
#define RANDOM_POINT_GENERATOR_H

#include "earcut.hpp"
#include <random>
#include <array>
#include <vector>

namespace random_point_generator {

// define types
using Point = std::array<double, 2>;
using Triangle = std::array<Point, 3>;
using TriangleList = std::vector<Triangle>;
using Polyline = std::vector<Point>;
using Polygon = std::vector<Polyline>;

Point operator-(const Point &p1, const Point &p2)
{
  return {p1[0] - p2[0], p1[1] - p2[1]};
}

Point operator+(const Point &p1, const Point &p2)
{
  return {p1[0] + p2[0], p1[1] + p2[1]};
}

Point operator*(double s, const Point &p)
{
  return {s * p[0], s * p[1]};
}

Point operator*(const Point &p, double s)
{
  return {s * p[0], s * p[1]};
}

double abs(const Point &p)
{
  return std::sqrt(p[0]*p[0] + p[1]*p[1]);
}

double dist(const Point &p1, const Point &p2)
{
  return abs(p1 - p2);
}

const Point& getIth(const Polygon &poly, uint32_t index)
{
  for (const Polyline &line : poly)
  {
    if (index >= line.size())
      index -= line.size();
    else
      return line[index];
  }
  throw std::out_of_range("Index not contained by polygon");
}

double area(const Triangle &tri)
{
  auto v1 = tri[0][0] * (tri[1][1] - tri[2][1]);
  auto v2 = tri[1][0] * (tri[2][1] - tri[0][1]);
  auto v3 = tri[2][0] * (tri[0][1] - tri[1][1]);
  return std::abs((v1 + v2 + v3) / 2);
}

class PolygonRandomPointGenerator
{
private:
  TriangleList triangles;
  std::vector<double> areas;
  std::discrete_distribution<uint32_t> triangle_selector;
  std::uniform_real_distribution<double> distrib01;
  std::default_random_engine generator;

  Point getRandomPointInTriangle(const Triangle &tri)
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
  PolygonRandomPointGenerator(const Polygon &poly) : distrib01(0.0, 1.0)
  {
    std::vector<uint32_t> triangle_inds = mapbox::earcut(poly);
    for (uint32_t i = 0; i < triangle_inds.size(); i += 3)
    {
      Triangle tri = {getIth(poly, triangle_inds[i]), getIth(poly, triangle_inds[i+1]), getIth(poly, triangle_inds[i+2])};
      triangles.push_back(tri);
      areas.push_back(area(tri));
    }
    triangle_selector = std::discrete_distribution<uint32_t>{areas.begin(), areas.end()};
  }

  Point getRandomPoint()
  {
    uint32_t selected_triangle = triangle_selector(generator);
    return getRandomPointInTriangle(triangles[selected_triangle]);
  }
};

} // namespace random_point_generator

#endif // RANDOM_POINT_GENERATOR_H
