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

inline octomap::OcTreeKey addKeys(const octomap::OcTreeKey &k1, const octomap::OcTreeKey &k2, const octomap::OcTreeKey &zero_key)
{
  return octomap::OcTreeKey(k1[0] - zero_key[0] + k2[0], k1[1] - zero_key[1] + k2[1], k1[2] - zero_key[2] + k2[2]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_gt_octree");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  std::string package_path = ros::package::getPath("roi_viewpoint_planner");

  octomap::OcTree gt_tree(0.02);
  octomap::KeySet gt_keys;

  octomap::OcTree vg07_6_tree("VG07_6_fruits_2cm.bt");
  vg07_6_tree.expand(); // for key computations to work
  octomap::KeySet vg07_6_gt_keys;

  octomap::OcTreeKey zero_key = gt_tree.coordToKey(0, 0, 0);

  ROS_INFO_STREAM("Zero key: " << zero_key[0] << ", " << zero_key[1] << ", " << zero_key[2]);

  size_t leaf_count = 0;
  for (auto it = vg07_6_tree.begin_leafs(), end = vg07_6_tree.end_leafs(); it != end; it++)
  {
    ROS_INFO_STREAM("Depth: " << it.getDepth());
    vg07_6_gt_keys.insert(it.getKey());
    leaf_count++;
  }

  ROS_INFO_STREAM("Leaf count: " << leaf_count);

  YAML::Node plant_list = YAML::LoadFile(package_path + "/cfg/world14_plants.yaml");
  for (const YAML::Node &plant : plant_list)
  {
    octomap::point3d loc = plant["location"].as<octomap::point3d>();
    std::string model = plant["model"].as<std::string>();
    octomap::OcTreeKey plantBaseKey = gt_tree.coordToKey(loc);
    for (const octomap::OcTreeKey &key : vg07_6_gt_keys)
    {
      octomap::OcTreeKey resKey = addKeys(plantBaseKey, key, zero_key);
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

  std::ofstream out_gt_tree("gt_tree_world14.bt");
  gt_tree.writeBinary(out_gt_tree);
  out_gt_tree.close();

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
