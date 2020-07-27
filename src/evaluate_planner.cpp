#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap/AbstractOcTree.h>
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
#include "roi_viewpoint_planner_msgs/EvaluatorConfig.h"
#include "compute_cubes.h"
#include "point_cloud_color_handler_clusters.h"

namespace ublas = boost::numeric::ublas;

pcl::visualization::PCLVisualizer *viewer = nullptr;
int vp1 = 0, vp2 = 1;
boost::mutex viewer_mtx;
std::atomic_bool viewer_initialized(false);

octomap_vpp::RoiOcTree* roiTree;
boost::mutex tree_mtx;
roi_viewpoint_planner::EvaluatorConfig config;

void octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
  if (tree){
    octomap_vpp::RoiOcTree* newRoiTree = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
    if(newRoiTree){
      tree_mtx.lock();
      delete roiTree;
      roiTree = newRoiTree;
      tree_mtx.unlock();
    }
    else {
      ROS_ERROR("Wrong octree type received; expecting ROI octree");
      delete tree;
    }
  }
  else
  {
    ROS_ERROR("Failed to deserialize octree message");
  }
}

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


void visualizeLoop()
{
  viewer_mtx.lock();
  viewer = new pcl::visualization::PCLVisualizer("ROI viewer");
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
  viewer->setBackgroundColor (0, 0, 0, vp1);
  viewer->addText ("Groundtruth", 10, 10, "vp1cap", vp1);
  //viewer->createViewPortCamera(vp1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, vp2);
  viewer->addText ("Detection", 10, 10, "vp2cap", vp2);
  //viewer->createViewPortCamera(vp2);
  viewer_initialized.store(true);
  viewer_mtx.unlock();
  for (ros::Rate rate(30); ros::ok(); rate.sleep())
  {
    viewer_mtx.lock();
    viewer->spinOnce();
    viewer_mtx.unlock();
  }
}

bool clusterWithPCL(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, std::vector<pcl::PointIndices> &clusters, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud = nullptr)
{
  if (input_cloud->empty())
  {
    ROS_WARN("Input cloud was emtpy; no clustering performed");
    return false;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_cloud);

  if (config.clustering == roi_viewpoint_planner::Evaluator_EUCLIDEAN_CLUSTERING)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (config.cluster_tolerance);
    ec.setMinClusterSize (config.min_cluster_size);
    ec.setMaxClusterSize (config.max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (clusters);
  }
  else if (config.clustering == roi_viewpoint_planner::Evaluator_REGION_GROWING)
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(input_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(config.normal_search_radius);
    normal_cloud.reset(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normal_cloud);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
    rg.setInputCloud(input_cloud);
    rg.setSearchMethod(tree);
    rg.setInputNormals(normal_cloud);
    rg.setMinClusterSize(config.min_cluster_size);
    rg.setMaxClusterSize(config.max_cluster_size);
    rg.setSmoothnessThreshold(config.smoothness_threshold / 180.0f * M_PI);
    rg.setCurvatureThreshold(config.curvature_threshold);
    rg.setResidualThreshold(config.residual_threshold);
    rg.setSmoothModeFlag(config.smooth_mode_flag);
    rg.setCurvatureTestFlag(config.curvature_test_flag);
    rg.setResidualTestFlag(config.residual_test_flag);
    rg.extract(clusters);
  }
  else
  {
    ROS_ERROR("No valid clustering method was selected");
    return false;
  }
  return true;
}

bool computeHullsAndVolumes(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud, const std::vector<pcl::PointIndices> &clusters,
                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &hulls_clouds,
                           std::vector<std::vector<pcl::Vertices>> &hulls_polygons,
                           std::vector<double> &cluster_volumes
                           )
{
  if (input_cloud->empty() || clusters.empty())
    return false;

  for (const pcl::PointIndices &cluster : clusters)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::Vertices> hull_polygons;

    pcl::PointIndicesPtr cluster_ptr(new pcl::PointIndices(cluster));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extr_inds;
    extr_inds.setInputCloud(input_cloud);
    extr_inds.setIndices(cluster_ptr);
    extr_inds.filter(*cluster_cloud);

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cluster_cloud);
    //hull.setIndices(cluster_ptr); // doesn't work -> use extract indices
    hull.setComputeAreaVolume(true);

    hull.reconstruct(*hull_cloud, hull_polygons);
    int dim = hull.getDimension();
    double vol;
    if (dim == 2)
    {
      const double thickness = 0.01; // Assume 1 cm thickness (might be reasonable to set to octomap resolution)
      vol = hull.getTotalArea() * thickness * 1000000; // to cm3
    }
    else // if dim == 3
    {
      vol = hull.getTotalVolume() * 1000000; // to cm3
    }

    ROS_INFO_STREAM("Hull dimension: " << dim << "Cluster volume: " << vol);

    hulls_clouds.push_back(hull_cloud);
    hulls_polygons.push_back(hull_polygons);
    cluster_volumes.push_back(vol);
  }
  return true;
}

void reconfigureCallback(roi_viewpoint_planner::EvaluatorConfig &new_config, uint32_t level)
{
  if (new_config.min_cluster_size > new_config.max_cluster_size) // min cluster size must be smaller than max cluster size
  {
    if (level & (1 << 1)) // min_cluster_size was changed
      new_config.max_cluster_size = new_config.min_cluster_size;
    else //if (level & (1 << 2)) / max_cluster_size was changed
      new_config.min_cluster_size = new_config.max_cluster_size;
  }

  if (!new_config.curvature_test_flag && !new_config.residual_test_flag) // either curvature or residual test must be activated
  {
    if (level & (1 << 9)) // residual_test_flag was changed
      new_config.curvature_test_flag = true;
    else if (level & (1 << 8)) // curvature_test_flag was changed
      new_config.residual_test_flag = true;
  }

  config = new_config;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // PCL visualization init
  boost::thread visualizeThread(visualizeLoop);

  dynamic_reconfigure::Server<roi_viewpoint_planner::EvaluatorConfig> server(nhp);
  server.setCallback(reconfigureCallback);

  std::string package_path = ros::package::getPath("roi_viewpoint_planner");

  double tree_resolution;
  if (!nh.getParam("/roi_viewpoint_planner/tree_resolution", tree_resolution))
  {
    ROS_ERROR("Planning tree resolution not found, make sure the planner is started");
    return -1;
  }
  std::stringstream resolution_sstr;
  resolution_sstr << tree_resolution;
  std::string resolution_str = resolution_sstr.str();

  ROS_INFO_STREAM("Resolution string: " << resolution_str);

  std::string world_name;
  if (!nh.getParam("/world_name", world_name))
  {
    ROS_ERROR("World name not specified; cannot load ground truth");
    return -1;
  }

  const std::vector<std::string> mode_list = {"idle", "automatic", "roi_centers", "roi_contours", "contours", "border", "roi_adjacent"};
  const std::string planning_mode_str = nhp.param<std::string>("planning_mode", "automatic");
  auto mode_it = std::find(mode_list.begin(), mode_list.end(), planning_mode_str);
  if (mode_it == mode_list.end())
  {
    ROS_ERROR("Specified planning mode not recognized");
    return -1;
  }
  const int planning_mode = mode_it - mode_list.begin();

  ROS_INFO_STREAM("Planning mode num: " << planning_mode);

  roiTree = new octomap_vpp::RoiOcTree(tree_resolution);

  std::string gt_file = package_path + "/cfg/world_roi_gts/" + world_name + "_roi_gt.yaml";

  ROS_INFO_STREAM("Reading ground truth");
  std::ifstream gt_ifs(gt_file);

  if (!gt_ifs.is_open())
  {
    ROS_INFO_STREAM("Could not open ground truth file " << gt_file);
    return -1;
  }

  std::vector<octomap::point3d> roi_locations;
  std::vector<octomap::point3d> roi_sizes;

  YAML::Node gt = YAML::Load(gt_ifs);
  for (const YAML::Node &roi : gt["rois"])
  {
    roi_locations.push_back(roi["location"].as<octomap::point3d>());
    roi_sizes.push_back(roi["size"].as<octomap::point3d>());
  }

  gt_ifs.close();

  // Compute GT-keys from bounding boxes

  octomap::KeySet gtRoiKeys;
  for (size_t i = 0; i < roi_locations.size(); i++)
  {
    octomap::OcTreeKey minKey = roiTree->coordToKey(roi_locations[i] - roi_sizes[i] * 0.5);
    octomap::OcTreeKey maxKey = roiTree->coordToKey(roi_locations[i] + roi_sizes[i] * 0.5);
    octomap::OcTreeKey curKey = minKey;
    for (curKey[0] = minKey[0]; curKey[0] <= maxKey[0]; curKey[0]++)
    {
      for (curKey[1] = minKey[1]; curKey[1] <= maxKey[1]; curKey[1]++)
      {
        for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; curKey[2]++)
        {
          gtRoiKeys.insert(curKey);
        }
      }
    }
  }
  ROS_INFO_STREAM("BB ROI key count: " << gtRoiKeys.size());

  // Read GT octree
  std::string gt_octree_file = package_path + "/cfg/world_gt_octrees/gt_tree_" + world_name + "_" + resolution_str + ".bt";
  octomap::OcTree gt_tree(gt_octree_file);
  gt_tree.expand(); // for key computations to work
  octomap::KeySet gt_tree_keys;

  for (auto it = gt_tree.begin_leafs(), end = gt_tree.end_leafs(); it != end; it++)
  {
    gt_tree_keys.insert(it.getKey());
  }

  ROS_INFO_STREAM("GT tree ROI key count: " << gt_tree_keys.size());

  auto isGtOcc = [](const octomap::OcTree &tree, const octomap::OcTreeNode *node) { return node->getLogOdds() > 0; };
  pcl::PointCloud<pcl::PointXYZ>::Ptr gt_pcl = octomap_vpp::octomapToPcl<octomap::OcTree, pcl::PointXYZ>(gt_tree, isGtOcc);

  std::vector<pcl::PointIndices> gt_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> gt_hulls_clouds;
  std::vector<std::vector<pcl::Vertices>> gt_hulls_polygons;
  std::vector<double> gt_cluster_volumes;

  clusterWithPCL(gt_pcl, gt_clusters);
  computeHullsAndVolumes(gt_pcl, gt_clusters, gt_hulls_clouds, gt_hulls_polygons, gt_cluster_volumes);

  while(!viewer_initialized.load())
  {
    ROS_INFO("Waiting for viewer initialization...");
    ros::Duration(0.1).sleep();
  }

  pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ> gt_color_handler(gt_pcl, gt_clusters);
  viewer_mtx.lock();
  viewer->addPointCloud<pcl::PointXYZ> (gt_pcl, gt_color_handler, "gt_cloud", vp1);
  for (size_t i = 0; i < gt_clusters.size(); i++)
  {
    std::string name = "gt_cluster_" + i;
    viewer->addPolygonMesh<pcl::PointXYZ>(gt_hulls_clouds[i], gt_hulls_polygons[i], name, vp1);
  }
  viewer_mtx.unlock();

  // Register publishers and subscribers

  ros::Publisher gt_pub = nhp.advertise<visualization_msgs::Marker>("roi_gt", 10, true);
  publishCubeVisualization(gt_pub, roi_locations, roi_sizes, COLOR_GREEN, "gt_rois");

  ros::Subscriber octomap_sub = nh.subscribe("/octomap", 10, octomapCallback);

  // Activate planner

  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> configClient("/roi_viewpoint_planner");

  std::ofstream resultsFile("planner_results.csv");
  resultsFile << "Time (s), Detected ROIs, ROI percentage, Average distance, Average volume accuracy, Covered ROI volume, False ROI volume, ROI key count, True ROI keys, False ROI keys" << std::endl;

  roi_viewpoint_planner::PlannerConfig planner_config;
  if (!configClient.getCurrentConfiguration(planner_config, ros::Duration(1)))
  {
    ROS_ERROR("Could not contact configuration server");
    return -1;
  }
  planner_config.activate_execution = true;
  planner_config.require_execution_confirmation = false;
  planner_config.mode = planning_mode;
  if (!configClient.setConfiguration(planner_config))
  {
    ROS_ERROR("Applying configuration not successful");
    return -1;
  }
  const double PLANNING_TIME = 180;
  ros::Time plannerStartTime = ros::Time::now();
  ROS_INFO("Planner activated");

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    std::vector<octomap::point3d> detected_locs, detected_sizes;

    tree_mtx.lock();
    roiTree->computeRoiKeys();
    octomap::KeySet roi_keys = roiTree->getRoiKeys();
    std::tie(detected_locs, detected_sizes) = roiTree->getClusterCentersWithVolume();

    // Mesh computations
    /*std::vector<octomap::point3d> vertices;
    std::vector<octomap_vpp::Triangle> faces;
    auto isRoi = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return tree.isNodeROI(node); };
    auto isOcc = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return node->getLogOdds() > 0; };
    octomap_vpp::polygonizeSubset<octomap_vpp::RoiOcTree>(*roiTree, roi_keys, vertices, faces, isRoi);

    // PCL Tests
    auto faceClusters = octomap_vpp::computeFaceClusters(faces);
    auto vertexClusters = octomap_vpp::computeClusterVertices(vertices, faceClusters);

    for (const octomap::point3d_collection &coll : vertexClusters)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = octomap_vpp::octomapPointCollectionToPcl<pcl::PointXYZ>(coll);
      ROS_INFO_STREAM("Cluster cloud size: " << cluster_cloud->size());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = octomap_vpp::octomapToPcl<octomap_vpp::RoiOcTree, pcl::PointXYZ>(*roiTree, isOcc);
    ROS_INFO_STREAM(pcl_cloud->size());*/

    auto isRoi = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return tree.isNodeROI(node); };
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_pcl = octomap_vpp::octomapToPcl<octomap_vpp::RoiOcTree, pcl::PointXYZ>(*roiTree, isRoi);

    tree_mtx.unlock();

    // PCL computations
    std::vector<pcl::PointIndices> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> hulls_clouds;
    std::vector<std::vector<pcl::Vertices>> hulls_polygons;
    std::vector<double> cluster_volumes;

    clusterWithPCL(roi_pcl, clusters);
    computeHullsAndVolumes(roi_pcl, clusters, hulls_clouds, hulls_polygons, cluster_volumes);

    ROS_INFO_STREAM("Number of clusters: " << clusters.size());

    // Debug view pointcloud
    if (viewer_initialized.load())
    {
      pcl::visualization::PointCloudColorHandlerClusters<pcl::PointXYZ> roi_color_handler(roi_pcl, clusters);
      viewer_mtx.lock();
      viewer->removeAllPointClouds(vp2);
      viewer->removeAllShapes(vp2);
      viewer->addPointCloud<pcl::PointXYZ> (roi_pcl, roi_color_handler, "roi_cloud", vp2);
      //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (roi_pcl, normal_cloud, 1, 0.02f, "roi_normals");
      for (size_t i = 0; i < clusters.size(); i++)
      {
        std::string name = "cluster_" + i;
        viewer->addPolygonMesh<pcl::PointXYZ>(hulls_clouds[i], hulls_polygons[i], name, vp2);
      }
      viewer_mtx.unlock();
    }

    ros::Time currentTime = ros::Time::now();

    ROS_INFO_STREAM("GT-Rois: " << roi_locations.size() << ", detected Rois: " << detected_locs.size());

    // Compute Pairs

    ublas::matrix<double> distances(roi_locations.size(), detected_locs.size());
    std::vector<IndexPair> indices;
    indices.reserve(roi_locations.size() * detected_locs.size());
    for (size_t i = 0; i < roi_locations.size(); i++)
    {
      for (size_t j = 0; j < detected_locs.size(); j++)
      {
        distances(i, j) = (roi_locations[i] - detected_locs[j]).norm();
        indices.push_back(IndexPair(i, j));
      }
    }

    auto indicesComp = [&distances](const IndexPair &a, const IndexPair &b)
    {
      return distances(a.gt_ind, a.det_ind) > distances(b.gt_ind, b.det_ind);
    };

    boost::dynamic_bitset<> usedGtPoints(roi_locations.size());
    boost::dynamic_bitset<> usedDetPoints(detected_locs.size());
    std::vector<IndexPair> roiPairs;
    roiPairs.reserve(std::min(roi_locations.size(), detected_locs.size()));

    double MAX_DISTANCE = 0.2; // Maximal distance to be considered as same ROI

    for (std::make_heap(indices.begin(), indices.end(), indicesComp); !usedGtPoints.all() && !usedDetPoints.all(); std::pop_heap(indices.begin(), indices.end(), indicesComp), indices.pop_back())
    {
      const IndexPair &pair = indices.front();

      if (distances(pair.gt_ind, pair.det_ind) > MAX_DISTANCE)
        break;

      if (usedGtPoints.test(pair.gt_ind) || usedDetPoints.test(pair.det_ind))
        continue;

      roiPairs.push_back(pair);
      usedGtPoints.set(pair.gt_ind);
      usedDetPoints.set(pair.det_ind);
    }
    /*while(!usedGtPoints.all() && !usedDetPoints.all())
    {
      double min_l = DBL_MAX;
      size_t ind_gt = -1, ind_det = -1;
      for (size_t i = 0; i < roi_locations.size(); i++)
      {
        if (usedGtPoints.test(i)) continue;
        for (size_t j = 0; j < detected_locs.size(); j++)
        {
          if (usedDetPoints.test(j)) continue;
          if (distances(i, j) < min_l)
          {
            min_l = distances(i, j);
            ind_gt = i;
            ind_det = j;
          }
        }
      }
      roiPairs.push_back(IndexPair(ind_gt, ind_det));
      usedGtPoints.set(ind_gt);
      usedDetPoints.set(ind_det);
    }*/
    double average_dist = 0;
    double average_vol_accuracy = 0;
    ROS_INFO_STREAM("Closest point pairs:");
    for (const IndexPair &pair : roiPairs)
    {
      ROS_INFO_STREAM(roi_locations[pair.gt_ind] << " and " << detected_locs[pair.det_ind] << "; distance: " << distances(pair.gt_ind, pair.det_ind));
      double gs = roi_sizes[pair.gt_ind].x() * roi_sizes[pair.gt_ind].y() * roi_sizes[pair.gt_ind].z();
      double ds = detected_sizes[pair.det_ind].x() * detected_sizes[pair.det_ind].y() * detected_sizes[pair.det_ind].z();

      double accuracy = 1 - std::abs(ds - gs) / gs;

      ROS_INFO_STREAM("Sizes: " << roi_sizes[pair.gt_ind] << " and " << detected_sizes[pair.det_ind] << "; factor: " << (ds / gs));

      average_dist += distances(pair.gt_ind, pair.det_ind);
      average_vol_accuracy += accuracy;
    }

    if (roiPairs.size() > 0)
    {
      average_dist /= roiPairs.size();
      average_vol_accuracy /= roiPairs.size();
    }

    // Compute volume overlap

    octomap::KeySet detectedRoiBBkeys;
    octomap::KeySet correctRoiBBkeys;
    octomap::KeySet falseRoiBBkeys;
    for (size_t i = 0; i < detected_locs.size(); i++)
    {
      octomap::OcTreeKey minKey = roiTree->coordToKey(detected_locs[i] - detected_sizes[i] * 0.5);
      octomap::OcTreeKey maxKey = roiTree->coordToKey(detected_locs[i] + detected_sizes[i] * 0.5);
      octomap::OcTreeKey curKey = minKey;
      for (curKey[0] = minKey[0]; curKey[0] <= maxKey[0]; curKey[0]++)
      {
        for (curKey[1] = minKey[1]; curKey[1] <= maxKey[1]; curKey[1]++)
        {
          for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; curKey[2]++)
          {
            detectedRoiBBkeys.insert(curKey);
            if (gtRoiKeys.find(curKey) != gtRoiKeys.end())
            {
              correctRoiBBkeys.insert(curKey);
            }
            else
            {
              falseRoiBBkeys.insert(curKey);
            }
          }
        }
      }
    }
    ROS_INFO_STREAM("Detected key count: " << detectedRoiBBkeys.size() << ", correct: " << correctRoiBBkeys.size() << ", false: " << falseRoiBBkeys.size());

    double coveredRoiVolumeRatio = (double)correctRoiBBkeys.size() / (double)gtRoiKeys.size();
    double falseRoiVolumeRatio = detectedRoiBBkeys.size() ? (double)falseRoiBBkeys.size() / (double)detectedRoiBBkeys.size() : 0;
    ROS_INFO_STREAM("Det vol ratio: " << coveredRoiVolumeRatio << ", False vol ratio: " << falseRoiVolumeRatio);

    // Computations directly with ROI cells
    octomap::KeySet true_roi_keys, false_roi_keys;
    for (const octomap::OcTreeKey &key : roi_keys)
    {
      if (gtRoiKeys.find(key) != gtRoiKeys.end())
      {
        true_roi_keys.insert(key);
      }
      else
      {
        false_roi_keys.insert(key);
      }
    }

    double passed_time = (currentTime - plannerStartTime).toSec();
    //resultsFile << "Time (s), Detected ROIs, ROI percentage, Average distance, Average volume accuracy, Covered ROI volume, False ROI volume" << std::endl;
    resultsFile << passed_time << ", " << roiPairs.size() << ", " << ((double)roiPairs.size() / (double)roi_locations.size()) << ", "
                << average_dist << ", " << average_vol_accuracy << ", " << coveredRoiVolumeRatio << ", " << falseRoiVolumeRatio << ", "
                << roi_keys.size() << ", " << true_roi_keys.size() << ", " << false_roi_keys.size() << std::endl;

    if (passed_time > PLANNING_TIME) // PLANNING_TIME s timeout
    {
      if (!configClient.getCurrentConfiguration(planner_config, ros::Duration(1)))
      {
        ROS_ERROR("Could not contact configuration server");
        break;
      }
      planner_config.activate_execution = false;
      planner_config.mode = roi_viewpoint_planner::Planner_IDLE;
      if (!configClient.setConfiguration(planner_config))
      {
        ROS_ERROR("Applying configuration not successful");
        break;
      }
      break;
    }
  }
  resultsFile.close();
}
