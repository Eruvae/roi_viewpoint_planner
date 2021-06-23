#include "roi_viewpoint_planner/gt_octree_loader.h"
#include <yaml-cpp/yaml.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

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

GtOctreeLoader::GtOctreeLoader(double resolution) : package_path(ros::package::getPath("roi_viewpoint_planner")),
  tree_resolution(resolution), final_fruit_trees(new std::vector<octomap::OcTree>), indexed_fruit_tree(new octomap_vpp::CountingOcTree(resolution)),
  gt_pcl(new pcl::PointCloud<pcl::PointXYZ>()), gt_clusters(new std::vector<pcl::PointIndices>()), final_fruit_keys(new std::vector<octomap::KeySet>()),
  fruit_locations(new std::vector<octomap::point3d>()), fruit_sizes(new std::vector<octomap::point3d>())
{
  ros::NodeHandle nh;
  pausePhysicsClient = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
  unpausePhysicsClient = nh.serviceClient<std_srvs::Empty>("gazebo/unpause_physics");
  setModelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  std::ostringstream oss;
  oss << std::setprecision(8) << std::noshowpoint << tree_resolution;
  std::string resolution_str = oss.str();
  ROS_INFO_STREAM("Resolution str: " << resolution_str);

  loadPlantTrees("VG07_6", resolution_str, 7);
  updateGroundtruth(true);
}

void GtOctreeLoader::updateGroundtruth(bool read_plant_poses)
{
  if (read_plant_poses)
    readPlantPoses();

  final_fruit_trees->clear();
  final_fruit_keys->clear();
  indexed_fruit_tree->clear();
  fruit_cell_counts.clear();
  gt_pcl->clear();
  gt_clusters->clear();
  fruit_locations->clear();
  fruit_sizes->clear();

  const octomap::OcTreeKey ZERO_KEY = indexed_fruit_tree->coordToKey(0, 0, 0);

  unsigned int fruit_index = 1;
  for (const PlantInfo &plant : plant_list)
  {
    const std::string &model = plant.model;
    octomap::point3d loc(plant.state.pose.position.x, plant.state.pose.position.y, plant.state.pose.position.z);

    auto it = plant_name_map.find(model);
    if (it == plant_name_map.end())
    {
      continue; // Plants with no fruits or unknown model are skipped
    }

    //std::vector<std::vector<octomap::OcTree>> plant_fruit_trees;
    //std::vector<std::vector<octomap::KeySet>> plant_fruit_keys;
    //std::unordered_map<std::string, size_t> plant_name_map;

    const std::vector<octomap::KeySet> &plant_keys = plant_fruit_keys[it->second];
    octomap::OcTreeKey plantBaseKey = indexed_fruit_tree->coordToKey(loc);
    for (const octomap::KeySet &fruit_keys : plant_keys)
    {
      octomap::point3d min_coord(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
      octomap::point3d max_coord(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
      octomap::OcTree fruit_tree(tree_resolution);
      octomap::KeySet fruit_keys_global;
      pcl::PointIndices fruit_indices;
      for (const octomap::OcTreeKey &key : fruit_keys)
      {
        octomap::OcTreeKey resKey = addKeys(plantBaseKey, key, ZERO_KEY);
        octomap::point3d coord = indexed_fruit_tree->keyToCoord(resKey);

        auto setMinMax = [](const float &c, float &min, float &max) -> void
        {
          if (c < min) min = c;
          if (c > max) max = c;
        };
        setMinMax(coord.x(), min_coord.x(), max_coord.x());
        setMinMax(coord.y(), min_coord.y(), max_coord.y());
        setMinMax(coord.z(), min_coord.z(), max_coord.z());

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
      fruit_locations->push_back((min_coord + max_coord)*0.5);
      fruit_sizes->push_back(max_coord - min_coord);
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
    gazebo_msgs::ModelState state;
    state.model_name = model_states->name[i];
    state.pose = model_states->pose[i];
    state.twist = model_states->twist[i];
    if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6_no_fruits"))
    {
      plant_list.push_back(PlantInfo("VG07_6_no_fruits", state)); // no fruits on plant
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6_one_fruit"))
    {
      plant_list.push_back(PlantInfo("VG07_6_one_fruit", state)); // one fruit plant; currently not supported (will have no fruits in GT)
    }
    else if (boost::algorithm::starts_with(state.model_name, "capsicum_plant_6"))
    {
      plant_list.push_back(PlantInfo("VG07_6", state));
    }
  }
}

double GtOctreeLoader::getMinDist(const octomap::point3d &point, const std::vector<octomap::point3d> &past_points)
{
  double min_dist = std::numeric_limits<double>::max();
  for (const octomap::point3d &other : past_points)
  {
    double dist = point.distance(other);
    if (dist < min_dist)
      min_dist = dist;
  }
  return min_dist;
}

void GtOctreeLoader::randomizePlantPositions(const octomap::point3d &min, const octomap::point3d &max, double min_dist, bool read_poses)
{
  if (read_poses)
    readPlantPoses();

  ROS_INFO_STREAM("Randomizing plant positions");

  std::uniform_real_distribution<float> x_dist(min.x(), max.x());
  std::uniform_real_distribution<float> y_dist(min.y(), max.y());
  std::uniform_real_distribution<float> z_dist(min.z(), max.z());

  std::vector<octomap::point3d> past_points;

  ROS_INFO_STREAM("Pausing physics to move plants");
  std_srvs::Empty empty_service;
  if (!pausePhysicsClient.call(empty_service))
    ROS_WARN("Couldn't pause physics to move plants");

  // A small sleep is necessary after pausing physics and changing each state,
  // because the gazebo set_model_state service is buggy.
  // Can't be a ROS sleep because clock is not published if gazebo is paused
  boost::this_thread::sleep_for(boost::chrono::milliseconds(100));

  for (PlantInfo &plant : plant_list)
  {
    octomap::point3d rand_point;
    size_t tries = 0;
    do
    {
      if (tries++ >= 1000) break; // prevent infinite loop if min_dist too high
      rand_point = octomap::point3d(x_dist(generator), y_dist(generator), z_dist(generator));
    } while(getMinDist(rand_point, past_points) < min_dist);

    ROS_INFO_STREAM("Sampled point: " << rand_point << ", tries: " << tries);

    gazebo_msgs::SetModelState set_model_state;
    set_model_state.request.model_state = plant.state;
    set_model_state.request.model_state.pose.position.x = rand_point.x();
    set_model_state.request.model_state.pose.position.y = rand_point.y();
    set_model_state.request.model_state.pose.position.z = rand_point.z();
    bool suc = setModelStateClient.call(set_model_state);
    if (!suc || !set_model_state.response.success)
    {
      ROS_WARN("Model state could not be set");
      continue;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    ROS_INFO_STREAM("Model state set successfully");
    plant.state = set_model_state.request.model_state;
    past_points.push_back(rand_point);
  }
  ROS_INFO_STREAM("Unpausing physics");
  if (!unpausePhysicsClient.call(empty_service))
    ROS_WARN("Couldn't unpause physics after moving plants");

  updateGroundtruth(true); // read plant poses again in case gazebo service doesn't work properly
}

} // namespace roi_viewpoint_planner
