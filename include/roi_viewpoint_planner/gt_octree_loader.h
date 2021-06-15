#ifndef GT_OCTREE_LOADER_H
#define GT_OCTREE_LOADER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap_vpp/NearestRegionOcTree.h>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

class GtOctreeLoader
{
private:
  std::string package_path;

  std::vector<std::vector<octomap::OcTree>> plant_fruit_trees;
  std::vector<std::vector<octomap::KeySet>> plant_fruit_keys;
  std::unordered_map<std::string, size_t> plant_name_map;

  std::shared_ptr<std::vector<octomap::OcTree>> final_fruit_trees;
  std::shared_ptr<std::vector<octomap::KeySet>> final_fruit_keys;
  std::shared_ptr<octomap_vpp::CountingOcTree> indexed_fruit_tree;
  std::vector<size_t> fruit_cell_counts;

  pcl::PointCloud<pcl::PointXYZ>::Ptr gt_pcl;
  std::shared_ptr<std::vector<pcl::PointIndices>> gt_clusters;

  static inline octomap::OcTreeKey addKeys(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTreeKey &zero_key)
  {
    return octomap::OcTreeKey(k1[0] - zero_key[0] + k2[0], k1[1] - zero_key[1] + k2[1], k1[2] - zero_key[2] + k2[2]);
  }

  void loadPlantTrees(const std::string &name, const std::string &resolution_str, size_t num_fruits);

public:
  GtOctreeLoader(const std::string &world_name, double resolution);

  std::shared_ptr<const std::vector<octomap::OcTree>> getFruitTrees()
  {
    return std::const_pointer_cast<const std::vector<octomap::OcTree>>(final_fruit_trees);
  }

  std::shared_ptr<const octomap_vpp::CountingOcTree> getIndexedFruitTree()
  {
    return std::const_pointer_cast<const octomap_vpp::CountingOcTree>(indexed_fruit_tree);
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPclCloud()
  {
    return boost::const_pointer_cast<const pcl::PointCloud<pcl::PointXYZ>>(gt_pcl);
  }

  std::shared_ptr<const std::vector<pcl::PointIndices>> getPclClusters()
  {
    return std::const_pointer_cast<const std::vector<pcl::PointIndices>>(gt_clusters);
  }

  // return index of associated fruit, or 0 for no fruit
  unsigned int getFruitIndex(const octomap::OcTreeKey &key)
  {
    octomap_vpp::CountingOcTreeNode *node = indexed_fruit_tree->search(key);
    if (node)
    {
      return node->getCount();
    }
    return 0;
  }

  size_t getNumFruitCells(size_t ind)
  {
    return fruit_cell_counts[ind];
  }

  std::vector<size_t> getFruitCellsCounts()
  {
    return fruit_cell_counts;
  }

  size_t getTotalFruitCellCount()
  {
    size_t total_count = 0;
    for (size_t count : fruit_cell_counts)
      total_count += count;

    return total_count;
  }

  size_t getNumFruits()
  {
    return fruit_cell_counts.size();
  }
};

#endif // GT_OCTREE_LOADER_H
