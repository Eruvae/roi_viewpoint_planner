#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <yaml-cpp/yaml.h>

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

std::string package_path;

inline octomap::OcTreeKey addKeys(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTreeKey &zero_key)
{
  return octomap::OcTreeKey(k1[0] - zero_key[0] + k2[0], k1[1] - zero_key[1] + k2[1], k1[2] - zero_key[2] + k2[2]);
}

std::vector<octomap::OcTree> plant_trees;
std::vector<octomap::KeySet> plant_keys;
std::unordered_map<std::string, size_t> plant_name_map;

void addTree(const std::string &name, octomap::OcTree &tree)
{
  tree.expand(); // for key computations to work
  octomap::KeySet tree_keys;
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; it++)
  {
    tree_keys.insert(it.getKey());
  }

  plant_trees.push_back(tree);
  plant_keys.push_back(tree_keys);
  plant_name_map.insert(std::make_pair(name, plant_trees.size() - 1));
}

void loadPlantTrees(const std::string &resolution_str)
{
  octomap::OcTree vg07_6_tree(package_path + "/cfg/plant_files/VG07_6_fruits_" + resolution_str + ".bt");
  addTree("VG07_6", vg07_6_tree);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_gt_octree");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  package_path = ros::package::getPath("roi_viewpoint_planner");

  if (argc < 3)
  {
    ROS_ERROR("World name and/or resolution not specified");
    return -1;
  }

  ROS_INFO_STREAM("Argv2: " << argv[2]);

  const std::string WORLD_NAME = argv[1];
  const std::string RESOLUTION_STR = argv[2];
  const double RESOLUTION = std::stod(RESOLUTION_STR);

  octomap::OcTree gt_tree(RESOLUTION);
  octomap::KeySet gt_keys;

  loadPlantTrees(RESOLUTION_STR);

  const octomap::OcTreeKey ZERO_KEY = gt_tree.coordToKey(0, 0, 0);

  ROS_INFO_STREAM("Zero key: " << ZERO_KEY[0] << ", " << ZERO_KEY[1] << ", " << ZERO_KEY[2]);

  YAML::Node plant_list = YAML::LoadFile(package_path + "/cfg/world_plant_locations/" + WORLD_NAME + "_plants.yaml");
  for (const YAML::Node &plant : plant_list)
  {
    octomap::point3d loc = plant["location"].as<octomap::point3d>();
    std::string model = plant["model"].as<std::string>();

    auto it = plant_name_map.find(model);
    if (it == plant_name_map.end())
    {
      ROS_ERROR_STREAM("Plant " << model << " is unknown");
      continue;
    }

    const octomap::KeySet &roi_keys = plant_keys[it->second];
    octomap::OcTreeKey plantBaseKey = gt_tree.coordToKey(loc);
    for (const octomap::OcTreeKey &key : roi_keys)
    {
      octomap::OcTreeKey resKey = addKeys(plantBaseKey, key, ZERO_KEY);
      gt_keys.insert(resKey);
      gt_tree.setNodeValue(resKey, gt_tree.getClampingThresMaxLog(), true);
    }
  }
  gt_tree.updateInnerOccupancy();

  ros::Publisher gt_tree_pub = nh.advertise<octomap_msgs::Octomap>("gt_tree", 1, true);

  octomap_msgs::Octomap gt_msg;
  gt_msg.header.frame_id = "world";
  gt_msg.header.stamp = ros::Time::now();
  bool msg_generated = octomap_msgs::fullMapToMsg(gt_tree, gt_msg);
  if (msg_generated)
  {
    gt_tree_pub.publish(gt_msg);
  }

  std::ofstream out_gt_tree(package_path + "/cfg/world_gt_octrees/gt_tree_" + WORLD_NAME + "_" + RESOLUTION_STR + ".bt");
  gt_tree.writeBinary(out_gt_tree);
  out_gt_tree.close();
  ROS_INFO_STREAM("Octree written to file, node keeps running to publish it. Close with CTRL+C.");

  ros::spin();

  // code below not necessary, resolution set when generating binvox
  /*const double DESIRED_RESOLUTION = 0.02;

  octomap::OcTree newResTree(DESIRED_RESOLUTION);
  double res_shift = std::log2(DESIRED_RESOLUTION / tree.getResolution());

  long shift_int = std::lround(res_shift);

  if (shift_int < 0) shift_int = 0;
  if (shift_int >= tree.getTreeDepth()) shift_int = tree.getTreeDepth() - 1;

  ROS_INFO_STREAM("Resolution: " << tree.getResolution() << ", Res-Factor: " << res_shift);

  for (auto it = tree.begin_leafs(tree.getTreeDepth() - shift_int), end = tree.end_leafs(); it != end; it++)
  {
    newResTree.updateNode(it.getCoordinate(), it->getLogOdds(), true);
  }
  newResTree.updateInnerOccupancy();
  newResTree.writeBinary("VG07_6_fruits_2cm.bt");*/
}
