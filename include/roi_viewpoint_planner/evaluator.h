#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <octomap_vpp/marching_cubes.h>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/distances.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Trigger.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include <roi_viewpoint_planner_msgs/EvaluatorConfig.h>
#include "roi_viewpoint_planner/compute_cubes.h"
#include "roi_viewpoint_planner/point_cloud_color_handler_clusters.h"
#include "roi_viewpoint_planner/planner_interface.h"
#include "roi_viewpoint_planner/gt_octree_loader.h"
#include <octomap_vpp/NearestRegionOcTree.h>
#include <pcl/io/pcd_io.h>

namespace roi_viewpoint_planner
{

struct IndexPair
{
  IndexPair(size_t gt_ind, size_t det_ind) : gt_ind(gt_ind), det_ind(det_ind) {}
  size_t gt_ind;
  size_t det_ind;
};

struct EvaluationParametersOld
{
  EvaluationParametersOld() : detected_roi_clusters(0), total_roi_clusters(0), roi_percentage(0),
    average_distance(0), average_accuracy(0), covered_roi_volume(0), false_roi_volume(0),
    roi_key_count(0), true_roi_key_count(0), false_roi_key_count(0) {}

  size_t detected_roi_clusters;
  size_t total_roi_clusters;
  double roi_percentage;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  double false_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
};

struct EvaluationParameters
{
  EvaluationParameters() : detected_roi_clusters(0), average_distance(0), average_accuracy(0),
    covered_roi_volume(0), roi_key_count(0), true_roi_key_count(0), false_roi_key_count(0) {}

  size_t detected_roi_clusters;
  double average_distance;
  double average_accuracy;
  double covered_roi_volume;
  size_t roi_key_count;
  size_t true_roi_key_count;
  size_t false_roi_key_count;
  std::vector<double> fruit_cell_counts;
  std::vector<double> fruit_cell_percentages;
  std::vector<double> volume_accuracies;
  std::vector<double> distances;
  std::vector<double> volumes;
};

struct GroundtruthParameters
{
  size_t num_fruits;
  std::vector<size_t> fruit_cell_counts;
  std::vector<double> fruit_volumes;
};

class Evaluator
{
private:
  std::shared_ptr<PlannerInterface> planner;
  std::shared_ptr<GtOctreeLoader> gtLoader;

  bool gt_comparison;
  std::string world_name;
  bool use_pcl_visualizer;

  pcl::visualization::PCLVisualizer *viewer;
  int vp1, vp2;
  boost::mutex viewer_mtx;
  std::atomic_bool viewer_initialized;

  roi_viewpoint_planner::EvaluatorConfig config;
  dynamic_reconfigure::Server<roi_viewpoint_planner::EvaluatorConfig> server;

  std::shared_ptr<const std::vector<octomap::point3d>> roi_locations;
  std::shared_ptr<const std::vector<octomap::point3d>> roi_sizes;
  octomap::KeySet gtRoiKeys;
  octomap::KeySet gt_tree_keys;

  GroundtruthParameters gt_params;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr gt_pcl;
  std::shared_ptr<const std::vector<pcl::PointIndices>> gt_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> gt_hulls_clouds;
  std::vector<std::vector<pcl::Vertices>> gt_hulls_polygons;
  std::vector<pcl::PointXYZ> gt_centroids;

  std::vector<octomap::point3d> detected_locs;
  std::vector<octomap::point3d> detected_sizes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_pcl;
  std::vector<pcl::PointIndices> clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> hulls_clouds;
  std::vector<std::vector<pcl::Vertices>> hulls_polygons;

  boost::numeric::ublas::matrix<double> distances;
  std::vector<IndexPair> roiPairs;

  boost::thread visualizeThread;
  pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>::Ptr gt_color_handler;
  pcl::visualization::PointCloudColorHandler<pcl::PointXYZ>::Ptr roi_color_handler;

  ros::Publisher gt_pub;
  ros::Publisher gt_fruit_pub;
  ros::Publisher gt_fruits_inflated_pub;

  bool readGroundtruth();
  //void computeGroundtruthPCL(); // call if clustering configuration is updated
  //void computeDetectionsPCL();
  void updateVisualizerGroundtruth();
  void updateVisualizerDetections();
  void computePairsAndDistances();

  void reconfigureCallback(roi_viewpoint_planner::EvaluatorConfig &new_config, uint32_t level);
  void visualizeLoop();

  //octomap_vpp::CountingOcTree* generateCountingOctree(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, std::vector<pcl::PointIndices> clusters, const std::string &name);

  bool clusterWithPCL(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = nullptr);
  bool computeHullsAndVolumes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters,
                             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &hulls_clouds,
                             std::vector<std::vector<pcl::Vertices>> &hulls_polygons,
                             std::vector<double> &cluster_volumes,
                             std::vector<pcl::PointXYZ> &centroids
                             );
public:
  Evaluator(std::shared_ptr<PlannerInterface> planner, ros::NodeHandle &nh, ros::NodeHandle &nhp,
            bool gt_comparison = true, bool use_pcl_visualizer = false, std::shared_ptr<GtOctreeLoader> gtLoader = nullptr);

  EvaluationParametersOld processDetectedRoisOld();

  EvaluationParameters processDetectedRois(bool save_pointcloud=false, size_t trial_num=0, size_t step=0);

  void saveClustersAsColoredCloud(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters);

  void saveGtAsColoredCloud()
  {
    std::string filename = "gt_" + world_name + ".pcd";
    saveClustersAsColoredCloud(filename, gt_pcl, *gt_clusters);
  }

  const GroundtruthParameters& getGtParams()
  {
    return gt_params;
  }

  ostream& writeHeaderOld(ostream &os);
  ostream& writeParamsOld(ostream &os, const EvaluationParametersOld &res);

  std::ostream& writeHeader(ostream &os);
  std::ostream& writeParams(ostream &os, const EvaluationParameters &res);

  void randomizePlantPositions(const octomap::point3d &min, const octomap::point3d &max, double min_dist);
};

} // namespace roi_viewpoint_planner

#endif // EVALUATOR_H
