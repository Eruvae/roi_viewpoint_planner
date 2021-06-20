#include "roi_viewpoint_planner/gt_octree_loader.h"
#include <yaml-cpp/yaml.h>
#include <gazebo_msgs/ModelStates.h>
#include <boost/algorithm/string.hpp>

namespace YAML
{
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
} // namespace YAML

namespace roi_viewpoint_planner
{

GtOctreeLoader::GtOctreeLoader(const std::string &world_name, double resolution) : package_path(ros::package::getPath("roi_viewpoint_planner")),
  final_fruit_trees(new std::vector<octomap::OcTree>), indexed_fruit_tree(new octomap_vpp::CountingOcTree(resolution)),
  gt_pcl(new pcl::PointCloud<pcl::PointXYZ>()), gt_clusters(new std::vector<pcl::PointIndices>()), final_fruit_keys(new std::vector<octomap::KeySet>())
{
  std::ostringstream oss;
  oss << std::setprecision(8) << std::noshowpoint << resolution;
  std::string resolution_str = oss.str();
  ROS_INFO_STREAM("Resolution str: " << resolution_str);

  loadPlantTrees("VG07_6", resolution_str, 7);
  readPlantPoses();

  const octomap::OcTreeKey ZERO_KEY = indexed_fruit_tree->coordToKey(0, 0, 0);

  unsigned int fruit_index = 1;
  for (const PlantInfo &plant : plant_list)
  {
    const std::string &model = plant.model;
    octomap::point3d loc(plant.pose.position.x, plant.pose.position.y, plant.pose.position.z);

    auto it = plant_name_map.find(model);
    if (it == plant_name_map.end())
    {
      ROS_ERROR_STREAM("Plant " << model << " is unknown");
      continue;
    }

    //std::vector<std::vector<octomap::OcTree>> plant_fruit_trees;
    //std::vector<std::vector<octomap::KeySet>> plant_fruit_keys;
    //std::unordered_map<std::string, size_t> plant_name_map;

    const std::vector<octomap::KeySet> &plant_keys = plant_fruit_keys[it->second];
    octomap::OcTreeKey plantBaseKey = indexed_fruit_tree->coordToKey(loc);
    for (const octomap::KeySet &fruit_keys : plant_keys)
    {
      octomap::OcTree fruit_tree(resolution);
      octomap::KeySet fruit_keys_global;
      pcl::PointIndices fruit_indices;
      for (const octomap::OcTreeKey &key : fruit_keys)
      {
        octomap::OcTreeKey resKey = addKeys(plantBaseKey, key, ZERO_KEY);
        octomap::point3d coord = indexed_fruit_tree->keyToCoord(resKey);
        fruit_tree.setNodeValue(resKey, fruit_tree.getClampingThresMaxLog());
        fruit_keys_global.insert(resKey);
        indexed_fruit_tree->setNodeCount(resKey, fruit_index);
        fruit_indices.indices.push_back(gt_pcl->size());
        gt_pcl->push_back(octomap_vpp::octomapPointToPcl<pcl::PointXYZ>(coord));
      }
      final_fruit_trees->push_back(fruit_tree);
      fruit_cell_counts.push_back(fruit_keys_global.size());
      final_fruit_keys->push_back(fruit_keys_global);
      gt_clusters->push_back(fruit_indices);
      fruit_index++;
    }
  }
}

void GtOctreeLoader::loadPlantTrees(const std::string &name, const std::string &resolution_str, size_t num_fruits)
{
  std::vector<octomap::OcTree> trees;
  std::vector<octomap::KeySet> keys;
  const std::string path = package_path + "/cfg/plant_files/individual_fruits/" + name + "/";
  for (size_t i=0; i < num_fruits; i++)
  {
    octomap::OcTree tree(path + resolution_str + "/" + name + "_fruit_" + std::to_string(i+1) + "_" + resolution_str + ".bt");
    tree.expand(); // for key computations to work
    octomap::KeySet tree_keys;
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; it++)
    {
      tree_keys.insert(it.getKey());
    }
    trees.push_back(tree);
    keys.push_back(tree_keys);
  }
  plant_fruit_trees.push_back(trees);
  plant_fruit_keys.push_back(keys);
  plant_name_map.insert(std::make_pair(name, plant_fruit_keys.size() - 1));
}

void GtOctreeLoader::readPlantPoses()
{
  gazebo_msgs::ModelStatesConstPtr model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
  if (!model_states)
  {
    ROS_ERROR_STREAM("Model states message not received; could not read plant poses");
    return;
  }
  plant_list.clear();
  for (size_t i=0; i < model_states->name.size(); i++)
  {
    const std::string &name = model_states->name[i];
    if (boost::algorithm::starts_with(name, "capsicum_plant_6_no_fruits"))
    {
      continue; // no fruits on plant
    }
    else if (boost::algorithm::starts_with(name, "capsicum_plant_6_one_fruit"))
    {
      continue; // one fruit plant; currently not supported
    }
    else if (boost::algorithm::starts_with(name, "capsicum_plant_6"))
    {
      plant_list.push_back(PlantInfo("VG07_6", model_states->pose[i]));
    }
  }
}

} // namespace roi_viewpoint_planner
