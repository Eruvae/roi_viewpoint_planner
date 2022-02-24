#include "roi_viewpoint_planner/evaluator_external_clusters.h"

namespace roi_viewpoint_planner
{

ExternalClusterEvaluator::ExternalClusterEvaluator(std::shared_ptr<GtOctreeLoader> gtLoader) : gtLoader(gtLoader)
{
  ros::NodeHandle nh;

  if (!gtLoader)
  {
    gtLoader.reset(new GtOctreeLoader(0.005));
  }

  gt_cluster_info = getClusterInfos(gtLoader->getPclCloud(), gtLoader->getPclClusters());

  cluster_pointcloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/capsicum_superellipsoid_detector/xyz_label_normal", 5,
                           boost::bind(&ExternalClusterEvaluator::processReceivedClusters, this, _1));
}

void ExternalClusterEvaluator::processReceivedClusters(const sensor_msgs::PointCloud2::ConstPtr &cluster_pc_msg)
{
  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cluster_pc(new pcl::PointCloud<pcl::PointXYZLNormal>);
  pcl::fromROSMsg(*cluster_pc_msg, *cluster_pc);
  ros::Time comp_start_time = ros::Time::now();
  std::vector<ClusterInfo<pcl::PointXYZLNormal>> clusters = getClusterInfos(cluster_pc);
  std::vector<std::pair<size_t, size_t>> pairs = computePairs(gt_cluster_info, clusters);

  ROS_INFO_STREAM("Clusters received, processing took " << (ros::Time::now() - comp_start_time));
  for (const auto &pair : pairs)
  {
    const auto &gt_cluster = gt_cluster_info[pair.first];
    const auto &det_cluster = clusters[pair.second];
    ROS_INFO_STREAM("GT Center: " << gt_cluster.center << ", GT Volume: " << gt_cluster.volume);
    ROS_INFO_STREAM("Center: " << det_cluster.center << ", Volume: " << det_cluster.volume);
    ROS_INFO_STREAM("Distance: " << pcl::L2_Norm(gt_cluster.center.getArray3fMap(), det_cluster.center.getArray3fMap(), 3) << "Volume ratio: " << (det_cluster.volume / gt_cluster.volume));
  }
}

} // namespace roi_viewpoint_planner
