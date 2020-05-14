#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner/PlannerConfig.h>
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
#include "compute_cubes.h"

namespace ublas = boost::numeric::ublas;

octomap_vpp::RoiOcTree* roiTree = new octomap_vpp::RoiOcTree(0.02);
boost::mutex tree_mtx;

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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string package_path = ros::package::getPath("roi_viewpoint_planner");

  std::string gt_file = nhp.param<std::string>("gt", package_path + "/cfg/gt_w18.yaml");

  ROS_INFO_STREAM("Reading ground truth");
  std::ifstream gt_ifs(gt_file);

  if (!gt_ifs.is_open())
  {
    ROS_INFO_STREAM("Could not open file " << gt_file);
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
  ROS_INFO_STREAM("ROI key count: " << gtRoiKeys.size());

  ros::Publisher gt_pub = nhp.advertise<visualization_msgs::Marker>("roi_gt", 10, true);
  publishCubeVisualization(gt_pub, roi_locations, roi_sizes, COLOR_GREEN, "gt_rois");

  ros::Subscriber octomap_sub = nh.subscribe("/octomap", 10, octomapCallback);
  dynamic_reconfigure::Client<roi_viewpoint_planner::PlannerConfig> configClient("/roi_viewpoint_planner");

  std::ofstream resultsFile("planner_results.csv");
  resultsFile << "Time (s), Detected ROIs, ROI percentage, Average distance, Average volume accuracy, Covered ROI volume, False ROI volume, ROI key count, True ROI keys, False ROI keys" << std::endl;

  roi_viewpoint_planner::PlannerConfig config;
  if (!configClient.getCurrentConfiguration(config, ros::Duration(1)))
  {
    ROS_ERROR("Could not contact configuration server");
    return -1;
  }
  config.activate_execution = true;
  config.require_execution_confirmation = false;
  config.mode = roi_viewpoint_planner::Planner_SAMPLE_AUTOMATIC;
  if (!configClient.setConfiguration(config))
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
    std::vector<octomap::point3d> vertices;
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
    ROS_INFO_STREAM(pcl_cloud->size());

    tree_mtx.unlock();

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
      if (!configClient.getCurrentConfiguration(config, ros::Duration(1)))
      {
        ROS_ERROR("Could not contact configuration server");
        break;
      }
      config.activate_execution = false;
      config.mode = roi_viewpoint_planner::Planner_IDLE;
      if (!configClient.setConfiguration(config))
      {
        ROS_ERROR("Applying configuration not successful");
        break;
      }
      break;
    }
  }
  resultsFile.close();
}
