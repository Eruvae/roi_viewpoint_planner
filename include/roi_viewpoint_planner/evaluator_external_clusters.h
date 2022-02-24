#include <ros/ros.h>
#include "gt_octree_loader.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/norms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/numeric/ublas/matrix.hpp>

namespace roi_viewpoint_planner
{

template<typename PointT>
struct ClusterInfo
{
  pcl::PointIndicesPtr inds = pcl::make_shared<pcl::PointIndices>();
  pcl::CentroidPoint<PointT> centroid;
  pcl::ConvexHull<PointT> hull;
  pcl::PointCloud<PointT> hull_cloud;
  pcl::PointXYZ center;
  double volume;
};

static std::vector<ClusterInfo<pcl::PointXYZ>> getClusterInfos(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc, const std::shared_ptr<const std::vector<pcl::PointIndices>> &inds)
{
  std::vector<ClusterInfo<pcl::PointXYZ>> clusters(inds->size());

  for (size_t i = 0; i< inds->size(); i++)
  {
    clusters[i].inds = pcl::make_shared<pcl::PointIndices>(inds->at(i));
    for (int index : inds->at(i).indices)
    {
      clusters[i].centroid.add(pc->at(index));
    }
    clusters[i].centroid.get<pcl::PointXYZ>(clusters[i].center);
    clusters[i].hull.setDimension(3);
    clusters[i].hull.setComputeAreaVolume(true);
    clusters[i].hull.setInputCloud(pc);
    clusters[i].hull.setIndices(clusters[i].inds);
    clusters[i].hull.reconstruct(clusters[i].hull_cloud);
    clusters[i].volume = clusters[i].hull.getTotalVolume();
  }
  return clusters;
}

static std::vector<ClusterInfo<pcl::PointXYZLNormal>> getClusterInfos(const pcl::PointCloud<pcl::PointXYZLNormal>::ConstPtr &cluster_pc)
{
  std::vector<ClusterInfo<pcl::PointXYZLNormal>> clusters(cluster_pc->back().label + 1);
  for (size_t i = 0; i < cluster_pc->size(); i++)
  {
    uint32_t label = cluster_pc->at(i).label;
    if (clusters.size() <= label) clusters.resize(label + 1);
    clusters[label].inds->indices.push_back(static_cast<int>(i));
    clusters[label].centroid.add(cluster_pc->at(i));
  }
  for (auto &cluster : clusters)
  {
    cluster.centroid.get<pcl::PointXYZ>(cluster.center);
    cluster.hull.setDimension(3);
    cluster.hull.setComputeAreaVolume(true);
    cluster.hull.setInputCloud(cluster_pc);
    cluster.hull.setIndices(cluster.inds);
    cluster.hull.reconstruct(cluster.hull_cloud);
    cluster.volume = cluster.hull.getTotalVolume();
  }
  return clusters;
}

static std::vector<std::pair<size_t, size_t>> computePairs(const std::vector<ClusterInfo<pcl::PointXYZ>> &gt, const std::vector<ClusterInfo<pcl::PointXYZLNormal>> &detected)
{
  // Build up matrix with all pairs
  boost::numeric::ublas::matrix<double> distances = boost::numeric::ublas::matrix<double>(gt.size(), detected.size());
  std::vector<std::pair<size_t, size_t>> indices;
  indices.reserve(gt.size() * detected.size());
  for (size_t i = 0; i < gt.size(); i++)
  {
    for (size_t j = 0; j < detected.size(); j++)
    {
      distances(i, j) = pcl::L2_Norm(gt[i].center.getArray3fMap(), detected[j].center.getArray3fMap(), 3);
      indices.push_back(std::make_pair(i, j));
    }
  }

  auto indicesComp = [&](const std::pair<size_t, size_t> &a, const std::pair<size_t, size_t> &b)
  {
    return distances(a.first, a.second) > distances(b.first, b.second);
  };

  boost::dynamic_bitset<> usedGtPoints(gt.size());
  boost::dynamic_bitset<> usedDetPoints(detected.size());

  std::vector<std::pair<size_t, size_t>> matchedPairs;

  matchedPairs.reserve(std::min(gt.size(), detected.size()));

  double MAX_DISTANCE = 0.2; // Maximal distance to be considered as same ROI

  for (std::make_heap(indices.begin(), indices.end(), indicesComp); !usedGtPoints.all() && !usedDetPoints.all(); std::pop_heap(indices.begin(), indices.end(), indicesComp), indices.pop_back())
  {
    const std::pair<size_t, size_t> &pair = indices.front();

    if (distances(pair.first, pair.second) > MAX_DISTANCE)
      break;

    if (usedGtPoints.test(pair.first) || usedDetPoints.test(pair.second))
      continue;

    matchedPairs.push_back(pair);
    usedGtPoints.set(pair.first);
    usedDetPoints.set(pair.second);
  }
  return matchedPairs;
}

class ExternalClusterEvaluator
{
private:
  std::shared_ptr<GtOctreeLoader> gtLoader;
  ros::Subscriber cluster_pointcloud_sub;
  std::vector<ClusterInfo<pcl::PointXYZ>> gt_cluster_info;

public:
  ExternalClusterEvaluator(std::shared_ptr<GtOctreeLoader> gtLoader = nullptr);

  void processReceivedClusters(const sensor_msgs::PointCloud2::ConstPtr &cluster_pc_msg);
};

} // namespace roi_viewpoint_planner
