#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <yaml-cpp/yaml.h>
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
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Trigger.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include "roi_viewpoint_planner_msgs/EvaluatorConfig.h"
#include "compute_cubes.h"
#include "point_cloud_color_handler_clusters.h"

namespace YAML {
template<>
struct convert<octomap::point3d> {
  static Node encode(const octomap::point3d& rhs) {
    Node node;
    node.push_back(rhs.x());
    node.push_back(rhs.y());
    node.push_back(rhs.z());
    return node;
  }

  static bool decode(const Node& node, octomap::point3d& rhs) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x() = node[0].as<double>();
    rhs.y() = node[1].as<double>();
    rhs.z() = node[2].as<double>();
    return true;
  }
};
}

struct IndexPair
{
  IndexPair(size_t gt_ind, size_t det_ind) : gt_ind(gt_ind), det_ind(det_ind) {}
  size_t gt_ind;
  size_t det_ind;
};

struct EvaluationParameters
{
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


class Evaluator
{
  ros::NodeHandle nh;
  ros::NodeHandle nhp;

  pcl::visualization::PCLVisualizer *viewer;
  int vp1, vp2;
  boost::mutex viewer_mtx;
  std::atomic_bool viewer_initialized;

  octomap_vpp::RoiOcTree *roiTree;
  boost::mutex tree_mtx;
  roi_viewpoint_planner::EvaluatorConfig config;
  dynamic_reconfigure::Server<roi_viewpoint_planner::EvaluatorConfig> server;

  int planning_mode;

  std::vector<octomap::point3d> roi_locations;
  std::vector<octomap::point3d> roi_sizes;
  octomap::KeySet gtRoiKeys;
  octomap::KeySet gt_tree_keys;

  pcl::PointCloud<pcl::PointXYZ>::Ptr gt_pcl;
  std::vector<pcl::PointIndices> gt_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> gt_hulls_clouds;
  std::vector<std::vector<pcl::Vertices>> gt_hulls_polygons;
  std::vector<double> gt_cluster_volumes;

  std::vector<octomap::point3d> detected_locs;
  std::vector<octomap::point3d> detected_sizes;

  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_pcl;
  std::vector<pcl::PointIndices> clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> hulls_clouds;
  std::vector<std::vector<pcl::Vertices>> hulls_polygons;
  std::vector<double> cluster_volumes;

  EvaluationParameters results;

  boost::thread visualizeThread;

  ros::Publisher gt_pub;
  ros::Subscriber octomap_sub;
  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> configClient;
  ros::ServiceClient saveOctomapClient;
  ros::ServiceClient resetPlannerClient;

  bool readGroundtruth();
  void computeGroundtruthPCL(); // call if clustering configuration is updated
  void updateVisualizerGroundtruth();
  void updateVisualizerDetections();

  void reconfigureCallback(roi_viewpoint_planner::EvaluatorConfig &new_config, uint32_t level);
  void octomapCallback(const octomap_msgs::OctomapConstPtr& msg);
  void visualizeLoop();

  bool clusterWithPCL(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = nullptr);
  bool computeHullsAndVolumes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters,
                             std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &hulls_clouds,
                             std::vector<std::vector<pcl::Vertices>> &hulls_polygons,
                             std::vector<double> &cluster_volumes
                             );
public:
  Evaluator(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &nhp = ros::NodeHandle("~"));

  bool activatePlanner();
  bool stopPlanner();
  bool resetPlanner();
  bool saveOctree(const std::string &filename);
  void clearOctree();

  const EvaluationParameters& processDetectedRois();
};

#endif // EVALUATOR_H
