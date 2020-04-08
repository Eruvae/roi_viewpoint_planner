#include <ros/ros.h>
#include <ros/package.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/dynamic_bitset.hpp>

octomap_vpp::RoiOcTree* roiTree = new octomap_vpp::RoiOcTree(0.1);
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
  IndexPair(size_t gt_ind, size_t det_ind, double dist) : gt_ind(gt_ind), det_ind(det_ind), dist(dist) {}
  size_t gt_ind;
  size_t det_ind;
  double dist;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  std::string package_path = ros::package::getPath("roi_viewpoint_planner");

  std::string gt_file = nhp.param<std::string>("gt", package_path + "/cfg/gt_w11.yaml");

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

  ros::Subscriber octomap_sub = nh.subscribe("/octomap", 10, octomapCallback);

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    std::vector<octomap::point3d> detected_locs, detected_sizes;

    tree_mtx.lock();
    roiTree->computeRoiKeys();
    std::tie(detected_locs, detected_sizes) = roiTree->getClusterCentersWithVolume();
    tree_mtx.unlock();

    ROS_INFO_STREAM("GT-Rois: " << roi_locations.size() << ", detected Rois: " << detected_locs.size());

    boost::dynamic_bitset<> usedGtPoints(roi_locations.size());
    boost::dynamic_bitset<> usedDetPoints(detected_locs.size());
    std::vector<IndexPair> roiPairs;
    while(!usedGtPoints.all() && !usedDetPoints.all())
    {
      double min_l = DBL_MAX;
      size_t ind_gt = -1, ind_det = -1;
      for (size_t i = 0; i < roi_locations.size(); i++)
      {
        if (usedGtPoints.test(i)) continue;
        for (size_t j = 0; j < detected_locs.size(); j++)
        {
          if (usedDetPoints.test(j)) continue;
          double l = (roi_locations[i] - detected_locs[j]).norm_sq();
          if (l < min_l)
          {
            min_l = l;
            ind_gt = i;
            ind_det = j;
          }
        }
      }
      roiPairs.push_back(IndexPair(ind_gt, ind_det, min_l));
      usedGtPoints.set(ind_gt);
      usedDetPoints.set(ind_det);
    }
    ROS_INFO_STREAM("Closest point pairs:");
    for (const IndexPair &pair : roiPairs)
    {
      ROS_INFO_STREAM(roi_locations[pair.gt_ind] << " and " << detected_locs[pair.det_ind] << "; distance: " << pair.dist);
    }
  }
}