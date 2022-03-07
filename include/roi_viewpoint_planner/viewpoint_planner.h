#ifndef VIEWPOINT_PLANNER_H
#define VIEWPOINT_PLANNER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <moveit_msgs/PlanningScene.h>
#include <sensor_msgs/CameraInfo.h>
//#include <image_geometry/pinhole_camera_model.h>

//#include <instance_segmentation_msgs/Detections.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/thread/mutex.hpp>
#include <random>
#include <vector>
#include <algorithm>
#include <unordered_set>

#if defined(__GNUC__ ) && (__GNUC__  < 7) // GCC < 7 has sample only in experimental namespace
#include <experimental/algorithm>
namespace std {
  using experimental::sample;
}
#endif

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseArray.h>
#include <std_srvs/Empty.h>

#include <octomap_vpp/RoiOcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap_vpp/WorkspaceOcTree.h>
#include <octomap_vpp/roioctree_utils.h>

#include <roi_viewpoint_planner_msgs/PlannerState.h>
#include "roi_viewpoint_planner/rvp_types.h"
#include "rvp_evaluation/compute_cubes.h"
#include "roi_viewpoint_planner/viewpoint_samplers.h"
#include "roi_viewpoint_planner/viewpoint_utilities.h"
#include "rvp_evaluation/evaluator.h"
#include "roi_viewpoint_planner/octree_provider_interfaces/direct_planner_interface.h"

#include "roi_viewpoint_planner/time_logger.h"

//#define PUBLISH_PLANNING_TIMES

#ifdef PUBLISH_PLANNING_TIMES
#include <roi_viewpoint_planner_msgs/PlanningTimes.h>
#endif

namespace roi_viewpoint_planner {

using rvp_evaluation::Evaluator;

// Constants

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

const std::string PC_TOPIC = "/move_group/filtered_cloud"; //"/camera/depth/points";
const std::string PC_GLOBAL = "/points_global";

class ViewpointPlanner
{
  // Sampler friends
  friend class SamplerBase;
  friend class RoiContourSampler;
  friend class RoiAdjacentSampler;
  friend class RoiCenterSampler;
  friend class ExplorationSampler;
  friend class ContourSampler;
  friend class BorderSampler;

  // Utility friends
  friend class UtilityBase;
  friend class SingleRayUtility;
  friend class MultiRayUtility;
  friend class RoiVicinityUtility;
  friend class RoiOcclusionUtility;

  // Planner interface friends
  friend class DirectPlannerInterface;

private:
  ros::NodeHandle &nh;
  ros::NodeHandle &nhp;
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  octomap_vpp::WorkspaceOcTree *workspaceTree;
  octomap_vpp::WorkspaceOcTree *samplingTree;
  Evaluator *evaluator;
  octomap::point3d wsMin, wsMax;
  octomap::point3d stMin, stMax;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::Publisher octomapPub;
  ros::Publisher inflatedOctomapPub;
  ros::Publisher pcGlobalPub;
  ros::Publisher pointVisPub;
  ros::Publisher viewArrowVisPub;
  ros::Publisher poseArrayPub;
  ros::Publisher workspaceTreePub;
  ros::Publisher samplingTreePub;
  ros::Publisher cubeVisPub;
  //ros::Publisher planningScenePub;

  ros::Publisher plannerStatePub;

  ros::ServiceClient requestExecutionConfirmation;
  ros::ServiceClient moveToSeeClient;

  ros::ServiceClient resetMoveitOctomapClient;
  std_srvs::Empty emptySrv;

  std::string bag_write_filename;
  std::string bag_final_filename;
  rosbag::Bag plannerBag;

  TimeLogger timeLogger;

  boost::mutex tree_mtx;

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub;
  tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter;
  //message_filters::Cache<sensor_msgs::PointCloud2> cloudCache(tfCloudFilter, 1);

  //message_filters::Subscriber<sensor_msgs::PointCloud2> pcGlobalSub(nh, PC_GLOBAL, 1);
  //message_filters::Subscriber<instance_segmentation_msgs::Detections> detectionsSub(nh, "/mask_rcnn/detections", 1);

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, instance_segmentation_msgs::Detections> DetsSyncPolicy;
  //message_filters::Synchronizer<DetsSyncPolicy> syncDets(DetsSyncPolicy(50), pcGlobalSub, detectionsSub);

  ros::Subscriber roiSub;

  moveit::planning_interface::MoveGroupInterface manipulator_group;

  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr kinematic_model;
  const robot_state::JointModelGroup* joint_model_group;
  robot_state::RobotStatePtr kinematic_state;

  std::atomic_bool robotIsMoving;
  std::atomic_bool scanInserted;

  roi_viewpoint_planner_msgs::PlannerState state;

  std::string map_frame;
  std::string ws_frame;

  #ifdef PUBLISH_PLANNING_TIMES
  boost::mutex times_mtx;
  ros::Publisher planningTimesPub;
  roi_viewpoint_planner_msgs::PlanningTimes times;
  #endif

  std::default_random_engine random_engine;

  // Evaluator variables
  bool eval_running;
  int eval_start_index;
  size_t eval_total_trials;
  EvalEpisodeEndParam eval_epEndParam;
  double eval_episode_duration;
  size_t eval_trial_num;
  std::ofstream eval_resultsFile;
  std::ofstream eval_resultsFileOld;
  std::ofstream eval_fruitCellPercFile;
  std::ofstream eval_volumeAccuracyFile;
  std::ofstream eval_distanceFile;
  ros::Time eval_plannerStartTime;
  double eval_accumulatedPlanDuration;
  double eval_accumulatedPlanLength;
  std::string eval_lastStep;
  bool eval_randomize;
  octomap::point3d eval_randomize_min;
  octomap::point3d eval_randomize_max;
  double eval_randomize_dist;

public:

  // Planner parameters

  enum PlannerMode
  {
    IDLE = 0,
    MAP_ONLY = 1,
    SAMPLE_AUTOMATIC = 2,
    SAMPLE_ROI_CONTOURS = 3,
    SAMPLE_ROI_ADJACENT = 4,
    SAMPLE_ROI_CENTERS = 5,
    SAMPLE_EXPLORATION = 6,
    SAMPLE_CONTOURS = 7,
    SAMPLE_BORDER = 8,
    NUM_MODES = 9 // Should be kept as last element if new modes are added
  } mode, roi_sample_mode, expl_sample_mode;

  static inline bool isRoiSampler(PlannerMode m)
  {
      return m >= SAMPLE_ROI_CONTOURS && m <= SAMPLE_ROI_CENTERS;
  }

  static inline bool isExplSampler(PlannerMode m)
  {
      return m >= SAMPLE_EXPLORATION && m <= SAMPLE_BORDER;
  }

  enum PlannerLoopState
  {
      NORMAL = 0,
      M2S = 1
  } loop_state;

  bool execute_plan;
  bool require_execution_confirmation;

  double sensor_min_range, sensor_max_range;
  //double sensor_hfov;
  //double sensor_vfov;

  bool insert_scan_if_not_moved;
  bool insert_scan_while_moving;
  bool wait_for_scan;
  bool publish_planning_state;

  std::string planner_id;
  double planning_time;

  bool use_cartesian_motion;
  bool compute_ik_when_sampling;

  bool record_map_updates;
  bool record_viewpoints;

  bool activate_move_to_see;
  bool move_to_see_exclusive;
  double m2s_delta_thresh;
  int m2s_max_steps;
  int m2s_current_steps;

  bool publish_cluster_visualization;
  size_t minimum_cluster_size;
  octomap_vpp::Neighborhood cluster_neighborhood;

  size_t roiMaxSamples, explMaxSamples;
  UtilityType roiUtil, explUtil;

  // Planner parameters end

  ViewpointPlanner(ros::NodeHandle &nh, ros::NodeHandle &nhp, const std::string &wstree_file, const std::string &sampling_tree_file, double tree_resolution,
                   const std::string &map_frame, const std::string &ws_frame, bool update_planning_tree=true, bool initialize_evaluator=true);

  ~ViewpointPlanner();

  bool initializeEvaluator(ros::NodeHandle &nh, ros::NodeHandle &nhp);
  bool startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                      bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist);
  void setEvaluatorStartParams();
  bool saveEvaluatorData(double plan_length, double traj_duration);
  bool resetEvaluator();

  std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree()
  {
    return planningTree;
  }

  boost::mutex& getTreeMutex()
  {
    return tree_mtx;
  }

  // Set planner parameters

  void setPoseReferenceFrame(const std::string& pose_reference_frame)
  {
    manipulator_group.setPoseReferenceFrame(pose_reference_frame);
  }

  void setPlannerId(const std::string& planner_id)
  {
    manipulator_group.setPlannerId(planner_id);
  }

  void setPlanningTime(double seconds)
  {
    manipulator_group.setPlanningTime(seconds);
  }

  void setMaxVelocityScalingFactor(double max_velocity_scaling_factor)
  {
    manipulator_group.setMaxVelocityScalingFactor(max_velocity_scaling_factor);
  }

  //void publishOctomapToPlanningScene(const octomap_msgs::Octomap &map_msg);
  void publishMap();

  void registerPointcloudWithRoi(const ros::MessageEvent<pointcloud_roi_msgs::PointcloudWithRoi const> &event);

  //void registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

  //void registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg);

  inline double sampleRandomSensorDistance()
  {
    if (sensor_min_range == sensor_max_range) return sensor_max_range;
    std::uniform_real_distribution<double> dist_distribution(sensor_min_range, sensor_max_range);
    return dist_distribution(random_engine);
  }

  octomap::point3d sampleRandomPointOnSphere(const octomap::point3d &center, double radius);

  //void getBorderPoints(const octomap::point3d &orig, double maxDist)

  /**
   * @brief dirVecToQuat computes quaternion closest to camQuat with x-axis aligned to dirVec
   * @param dirVec desired x-axis direction
   * @param camQuat camera orientation
   * @param viewDir camera view direction (x-axis)
   * @return computed quaternion
   */
  tf2::Quaternion dirVecToQuat(const octomath::Vector3 &dirVec, const tf2::Quaternion &camQuat, const tf2::Vector3 &viewDir);

  void publishViewpointVisualizations(const std::vector<Viewpoint> &viewpoints, const std::string &ns, const std_msgs::ColorRGBA &color = COLOR_RED);

  void saveViewpointsToBag(const std::vector<Viewpoint> &viewpoints, const std::string &sampling_method, const ros::Time &time);

  void sampleAroundROICenter(const octomap::point3d &center,  const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t roiID = 0);

  double computeExpectedRayIGinSamplingTree(const octomap::KeyRay &ray);

  double computeViewpointSamplingValue(const octomap::pose6d &viewpoint, const double &hfov, size_t x_steps, size_t y_steps, const double &maxRange, bool use_roi_weighting = true);

  bool hasDirectUnknownNeighbour(const octomap::OcTreeKey &key, unsigned int depth = 0);

  bool hasUnknownNeighbour18(const octomap::OcTreeKey &key, unsigned int depth = 0);

  bool hasUnknownAndOccupiedNeighbour6(const octomap::OcTreeKey &key, unsigned int depth = 0);

  bool hasUnknownAndOccupiedNeighbour18(const octomap::OcTreeKey &key, unsigned int depth = 0);

  void visualizeBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, unsigned int depth = 0);

  octomap::point3d transformToMapFrame(const octomap::point3d &p);
  geometry_msgs::Pose transformToMapFrame(const geometry_msgs::Pose &p);

  octomap::point3d transformToWorkspace(const octomap::point3d &p);
  geometry_msgs::Pose transformToWorkspace(const geometry_msgs::Pose &p);

  octomap::point3d computeSurfaceNormalDir(const octomap::OcTreeKey &key);

  octomap::point3d computeUnknownDir(const octomap::OcTreeKey &key);

  void getFreeNeighbours6(const octomap::OcTreeKey &key, octomap::KeySet &freeKeys);

  std::vector<Viewpoint> sampleAroundMultiROICenters(const std::vector<octomap::point3d> &centers, const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  std::vector<Viewpoint> sampleContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  std::vector<Viewpoint> sampleRoiContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  std::vector<Viewpoint> sampleRoiAdjecentCountours(const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  std::vector<Viewpoint> sampleExplorationPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  std::vector<Viewpoint> sampleBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, const octomap::point3d &camPos, const tf2::Quaternion &camQuat);

  robot_state::RobotStatePtr sampleNextRobotState(const robot_state::JointModelGroup *joint_model_group, const robot_state::RobotState &current_state);

  bool moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToPose(const geometry_msgs::Pose &goal_pose, bool async=false, bool safe=true);

  bool moveToState(const moveit::core::RobotStateConstPtr &goal_state, bool async=false, bool safe=true);

  bool moveToState(const std::vector<double> &joint_values, bool async=false, bool safe=true);

  bool planAndExecuteFromMoveGroup(bool async, bool safe);

  bool safeExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);

  bool executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async);

  bool saveTreeAsObj(const std::string &file_name);

  bool saveROIsAsObj(const std::string &file_name);

  std::string saveOctomap(const std::string &name = "planningTree", bool name_is_prefix = true);

  int loadOctomap(const std::string &filename);

  void resetOctomap();

  bool randomizePlantPositions(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist);

  void plannerLoop();

  bool plannerLoopOnce(); // returns true if moved
};

} // namespace roi_viewpoint_planner

#endif // VIEWPOINT_PLANNER_H
