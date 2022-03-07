#include "roi_viewpoint_planner/viewpoint_planner.h"

#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <roi_viewpoint_planner_msgs/GetGradient.h>
#include <roi_viewpoint_planner_msgs/ViewpointList.h>
#include <octomap_vpp/marching_cubes.h>
#include <octomap_vpp/octomap_transforms.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "roi_viewpoint_planner/octree_provider_interfaces/direct_planner_interface.h"
#include "rvp_evaluation/rvp_utils.h"

namespace roi_viewpoint_planner
{

ViewpointPlanner::ViewpointPlanner(ros::NodeHandle &nh, ros::NodeHandle &nhp, const std::string &wstree_file, const std::string &sampling_tree_file, double tree_resolution,
                                   const std::string &map_frame, const std::string &ws_frame, bool update_planning_tree, bool initialize_evaluator) :
  nh(nh), nhp(nhp),
  planningTree(new octomap_vpp::RoiOcTree(tree_resolution)),
  map_frame(map_frame),
  ws_frame(ws_frame),
  workspaceTree(nullptr),
  samplingTree(nullptr),
  evaluator(nullptr),
  wsMin(-FLT_MAX, -FLT_MAX, -FLT_MAX),
  wsMax(FLT_MAX, FLT_MAX, FLT_MAX),
  stMin(-FLT_MAX, -FLT_MAX, -FLT_MAX),
  stMax(FLT_MAX, FLT_MAX, FLT_MAX),
  tfBuffer(ros::Duration(30)),
  tfListener(tfBuffer),
  depthCloudSub(nh, PC_TOPIC, 1),
  tfCloudFilter(depthCloudSub, tfBuffer, map_frame, 100, nh),
  manipulator_group("manipulator"),
  robot_model_loader("robot_description"),
  kinematic_model(robot_model_loader.getModel()),
  joint_model_group(kinematic_model->getJointModelGroup("manipulator")),
  kinematic_state(new robot_state::RobotState(kinematic_model)),
  mode(IDLE),
  loop_state(NORMAL),
  execute_plan(false),
  robotIsMoving(false),
  scanInserted(false),
  m2s_current_steps(0),
  eval_running(false),
  eval_randomize(true),
  eval_randomize_min(-1, -1, -0.1),
  eval_randomize_max(1, 1, 0.1),
  eval_randomize_dist(0.4),
  random_engine(std::random_device{}())
{

  std::stringstream fDateTime;
  const boost::posix_time::ptime curDateTime = boost::posix_time::second_clock::local_time();
  boost::posix_time::time_facet *const timeFacet = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
  fDateTime.imbue(std::locale(fDateTime.getloc(), timeFacet));
  fDateTime << "plannerBag_" << curDateTime << ".bag";
  bag_final_filename = fDateTime.str();
  fDateTime << ".active";
  bag_write_filename = fDateTime.str();
  plannerBag.open(bag_write_filename, rosbag::bagmode::Write);

  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflated_octomap", 1);
  //pcGlobalPub = nh.advertise<sensor_msgs::PointCloud2>(PC_GLOBAL, 1);
  //planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  pointVisPub = nh.advertise<visualization_msgs::Marker>("border_marker", 1);
  viewArrowVisPub = nh.advertise<visualization_msgs::MarkerArray>("roi_vp_marker", 1);
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("vp_array", 1);
  workspaceTreePub = nh.advertise<octomap_msgs::Octomap>("workspace_tree", 1, true);
  samplingTreePub = nh.advertise<octomap_msgs::Octomap>("sampling_tree", 1, true);
  cubeVisPub = nh.advertise<visualization_msgs::Marker>("cube_vis", 1, true);

  plannerStatePub = nhp.advertise<roi_viewpoint_planner_msgs::PlannerState>("planner_state", 1, true);

  #ifdef PUBLISH_PLANNING_TIMES
  planningTimesPub = nhp.advertise<roi_viewpoint_planner_msgs::PlanningTimes>("planning_times", 1, true);
  #endif

  requestExecutionConfirmation = nhp.serviceClient<std_srvs::Trigger>("request_execution_confirmation");
  moveToSeeClient = nh.serviceClient<roi_viewpoint_planner_msgs::GetGradient>("move_to_see_srv/get_gradient");

  resetMoveitOctomapClient = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

  setPoseReferenceFrame(map_frame);

  // Load workspace

  octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(wstree_file);
  if (!tree)
  {
    ROS_ERROR_STREAM("Workspace tree file could not be loaded");
  }
  else
  {
    octomap_vpp::CountingOcTree *countingTree = dynamic_cast<octomap_vpp::CountingOcTree*>(tree);

    if (countingTree) // convert to workspace tree if counting tree loaded
    {
      workspaceTree = new octomap_vpp::WorkspaceOcTree(*countingTree);
      delete countingTree;
    }
    else
    {
      workspaceTree = dynamic_cast<octomap_vpp::WorkspaceOcTree*>(tree);
    }

    if (!workspaceTree)
    {
      ROS_ERROR("Workspace tree type not recognized; please load either CountingOcTree or WorkspaceOcTree");
      delete tree;
    }
    else
    {
      wsMin = octomap::point3d(FLT_MAX, FLT_MAX, FLT_MAX);
      wsMax = octomap::point3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);
      for (auto it = workspaceTree->begin_leafs(), end = workspaceTree->end_leafs(); it != end; it++)
      {
        octomap::point3d coord = it.getCoordinate();
        if (coord.x() < wsMin.x()) wsMin.x() = coord.x();
        if (coord.y() < wsMin.y()) wsMin.y() = coord.y();
        if (coord.z() < wsMin.z()) wsMin.z() = coord.z();
        if (coord.x() > wsMax.x()) wsMax.x() = coord.x();
        if (coord.y() > wsMax.y()) wsMax.y() = coord.y();
        if (coord.z() > wsMax.z()) wsMax.z() = coord.z();
      }

      octomap_msgs::Octomap ws_msg;
      ws_msg.header.frame_id = ws_frame;
      ws_msg.header.stamp = ros::Time(0);
      bool msg_generated = octomap_msgs::fullMapToMsg(*workspaceTree, ws_msg);
      if (msg_generated)
      {
        workspaceTreePub.publish(ws_msg);
      }
    }
  }

  tree = octomap::AbstractOcTree::read(sampling_tree_file);
  if (!tree)
  {
    ROS_ERROR_STREAM("Sampling tree file could not be loaded");
  }
  else
  {
    samplingTree = dynamic_cast<octomap_vpp::WorkspaceOcTree*>(tree);
    if (!samplingTree)
    {
      ROS_ERROR("Sampling tree must be of type WorkspaceOcTree");
      delete tree;
    }
  }

  if (!samplingTree) // if sampling tree not specified, use workspace octree
  {
    samplingTree = workspaceTree;
  }

  if (samplingTree)
  {
    stMin = octomap::point3d(FLT_MAX, FLT_MAX, FLT_MAX);
    stMax = octomap::point3d(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    for (auto it = samplingTree->begin_leafs(), end = samplingTree->end_leafs(); it != end; it++)
    {
      octomap::point3d coord = it.getCoordinate();
      if (coord.x() < stMin.x()) stMin.x() = coord.x();
      if (coord.y() < stMin.y()) stMin.y() = coord.y();
      if (coord.z() < stMin.z()) stMin.z() = coord.z();
      if (coord.x() > stMax.x()) stMax.x() = coord.x();
      if (coord.y() > stMax.y()) stMax.y() = coord.y();
      if (coord.z() > stMax.z()) stMax.z() = coord.z();
    }

    octomap_msgs::Octomap st_msg;
    st_msg.header.frame_id = ws_frame;
    st_msg.header.stamp = ros::Time(0);
    bool msg_generated = octomap_msgs::fullMapToMsg(*samplingTree, st_msg);
    if (msg_generated)
    {
      samplingTreePub.publish(st_msg);
    }
  }

  if (workspaceTree)
  {
    manipulator_group.setWorkspace(wsMin.x(), wsMin.y(), wsMin.z(), wsMax.x(), wsMax.y(), wsMax.z());

    visualization_msgs::Marker ws_cube;
    ws_cube.header.frame_id = ws_frame;
    ws_cube.header.stamp = ros::Time(0);
    ws_cube.ns = "ws_cube";
    ws_cube.id = 0;
    ws_cube.type = visualization_msgs::Marker::LINE_LIST;
    ws_cube.color.a = 1.0;
    ws_cube.color.r = 1.0;
    ws_cube.color.g = 0.0;
    ws_cube.color.b = 0.0;
    ws_cube.scale.x = 0.002;
    addCubeEdges(stMin, stMax, ws_cube.points);

    cubeVisPub.publish(ws_cube);
  }

  if (update_planning_tree)
    roiSub = nh.subscribe("/detect_roi/results", 1, &ViewpointPlanner::registerPointcloudWithRoi, this);

  if (initialize_evaluator)
  {
    ros::NodeHandle nh_eval("evaluator");
    initializeEvaluator(nh, nh_eval);
    evaluator->saveGtAsColoredCloud();
  }
}

ViewpointPlanner::~ViewpointPlanner()
{
  plannerBag.close();
  rename(bag_write_filename.c_str(), bag_final_filename.c_str());
}

bool ViewpointPlanner::initializeEvaluator(ros::NodeHandle &nh, ros::NodeHandle &nhp)
{
  std::shared_ptr<DirectPlannerInterface> interface(new DirectPlannerInterface(this));
  evaluator = new Evaluator(interface, nh, nhp, true);
  eval_trial_num = 0;
  eval_start_index = 0;
  return true;
}

bool ViewpointPlanner::startEvaluator(size_t numEvals, EvalEpisodeEndParam episodeEndParam, double episodeDuration, int start_index,
                                      bool randomize_plants, const octomap::point3d &min, const octomap::point3d &max, double min_dist)
{
  if (eval_running)
    return false;

  eval_trial_num = 0;
  eval_start_index = start_index;
  eval_total_trials = numEvals;
  eval_epEndParam = episodeEndParam;
  eval_episode_duration = episodeDuration;  
  eval_running = true;
  execute_plan = true;
  require_execution_confirmation = false;

  eval_randomize = randomize_plants;
  eval_randomize_min = min;
  eval_randomize_max = max;
  eval_randomize_dist = min_dist;

  if (eval_randomize)
  {
    evaluator->randomizePlantPositions(eval_randomize_min, eval_randomize_max, eval_randomize_dist);
  }
  resetOctomap();

  setEvaluatorStartParams();
  timeLogger.initNewFile(true, start_index);
  return true;
}

void ViewpointPlanner::setEvaluatorStartParams()
{
  std::string file_index_str = std::to_string(eval_start_index + eval_trial_num);
  eval_resultsFile = std::ofstream("planner_results_" + file_index_str + ".csv");
  eval_resultsFileOld = std::ofstream("planner_results_old" + file_index_str + ".csv");
  eval_fruitCellPercFile = std::ofstream("results_fruit_cells_" + file_index_str + ".csv");
  eval_volumeAccuracyFile = std::ofstream("results_volume_accuracy_" + file_index_str + ".csv");
  eval_distanceFile = std::ofstream("results_distances_" + file_index_str + ".csv");
  eval_resultsFile << "Time (s),Plan duration (s),Plan Length,";
  evaluator->writeHeader(eval_resultsFile) << ",Step" << std::endl;
  eval_resultsFileOld << "Time (s),Plan duration (s),Plan Length,";
  evaluator->writeHeaderOld(eval_resultsFileOld) << ",Step" << std::endl;
  eval_plannerStartTime = ros::Time::now();
  eval_accumulatedPlanDuration = 0;
  eval_accumulatedPlanLength = 0;
  mode = SAMPLE_AUTOMATIC;
}

template<typename T>
std::ostream& writeVector(std::ostream &os, double passed_time, const std::vector<T> &vec)
{
  os << passed_time << ",";
  for (size_t i = 0; i < vec.size(); i++)
  {
    os << vec[i];
    if (i < vec.size() - 1)
      os << ",";
  }
  return os;
}

bool ViewpointPlanner::saveEvaluatorData(double plan_length, double traj_duration)
{
  ros::Time currentTime = ros::Time::now();

  double passed_time = (currentTime - eval_plannerStartTime).toSec();

  eval_accumulatedPlanDuration += traj_duration;
  eval_accumulatedPlanLength += plan_length;

  rvp_evaluation::EvaluationParameters res = evaluator->processDetectedRois(true, eval_start_index + eval_trial_num, static_cast<size_t>(passed_time));
  rvp_evaluation::EvaluationParametersOld resOld = evaluator->processDetectedRoisOld();

  eval_resultsFile << passed_time << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  evaluator->writeParams(eval_resultsFile, res) << "," << eval_lastStep << std::endl;

  eval_resultsFileOld << passed_time << "," << eval_accumulatedPlanDuration << "," << eval_accumulatedPlanLength << ",";
  evaluator->writeParamsOld(eval_resultsFileOld, resOld) << "," << eval_lastStep << std::endl;

  writeVector(eval_fruitCellPercFile, passed_time, res.fruit_cell_percentages) << std::endl;
  writeVector(eval_volumeAccuracyFile, passed_time, res.volume_accuracies) << std::endl;
  writeVector(eval_distanceFile, passed_time, res.distances) << std::endl;

  switch (eval_epEndParam)
  {
  case EvalEpisodeEndParam::TIME:
  {
    if (passed_time > eval_episode_duration)
      resetEvaluator();
    break;
  }
  case EvalEpisodeEndParam::PLAN_DURATION:
  {
    if (eval_accumulatedPlanDuration > eval_episode_duration)
      resetEvaluator();
    break;
  }
  case EvalEpisodeEndParam::PLAN_LENGTH:
  {
    if (eval_accumulatedPlanLength > eval_episode_duration)
      resetEvaluator();
    break;
  }
  default:
  {
    ROS_ERROR_STREAM("Invalid episode end param");
    resetEvaluator();
    break;
  }
  }
  return true;
}

bool ViewpointPlanner::resetEvaluator()
{
  eval_resultsFile.close();
  eval_resultsFileOld.close();
  eval_fruitCellPercFile.close();
  eval_volumeAccuracyFile.close();
  eval_distanceFile.close();
  timeLogger.initNewFile();
  eval_trial_num++;

  mode = IDLE;
  resetOctomap();

  bool success = false;
  std::vector<double> joint_start_values;
  if(nhp.getParam("initial_joint_values", joint_start_values))
  {
    success = moveToState(joint_start_values, false, false);
  }
  else
  {
    ROS_WARN("No inital joint values set");
  }

  if (eval_trial_num < eval_total_trials)
  {
    setEvaluatorStartParams();
  }
  else
  {
    eval_running = false;
  }

  if (eval_randomize)
  {
    evaluator->randomizePlantPositions(eval_randomize_min, eval_randomize_max, eval_randomize_dist);
  }
  resetOctomap();

  return success;
}

/*void ViewpointPlanner::publishOctomapToPlanningScene(const octomap_msgs::Octomap &map_msg)
{
  moveit_msgs::PlanningScene scene;
  scene.world.octomap.header = map_msg.header;
  scene.world.octomap.octomap = map_msg;
  scene.world.octomap.octomap.id = "OcTree";
  scene.is_diff = true;
  planningScenePub.publish(scene);
}*/

void ViewpointPlanner::publishMap()
{
  octomap_msgs::Octomap map_msg;
  std::vector<octomap::point3d> centers, volumes;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();

  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(*planningTree, map_msg);
  if (publish_cluster_visualization)
  {
    std::tie(centers, volumes) = planningTree->getClusterCentersWithVolume(minimum_cluster_size, cluster_neighborhood);
  }
  tree_mtx.unlock();

  if (msg_generated)
  {
    octomapPub.publish(map_msg);
    //publishOctomapToPlanningScene(map_msg);
  }

  if (publish_cluster_visualization)
  {
    publishCubeVisualization(cubeVisPub, centers, volumes);
  }

  //if (octomap_msgs::binaryMapToMsg(testTree, map_msg))
  //    octomapPub.publish(map_msg);

  /*tree_mtx.lock();
  ros::Time inflationStart = ros::Time::now();
  std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflatedTree = planningTree->computeInflatedRois();
  ros::Time inflationDone = ros::Time::now();
  tree_mtx.unlock();
  ROS_INFO_STREAM("Time for inflation: " << inflationDone - inflationStart);*/

  std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflatedTree = planningTree->getInflatedRois();

  if (inflatedTree != nullptr)
  {
    octomap_msgs::Octomap inflated_map;
    inflated_map.header.frame_id = map_frame;
    inflated_map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*inflatedTree, inflated_map))
    {
      inflatedOctomapPub.publish(inflated_map);
    }
  }
}

void ViewpointPlanner::registerPointcloudWithRoi(const ros::MessageEvent<pointcloud_roi_msgs::PointcloudWithRoi const> &event)
{
  if (mode == IDLE || (!insert_scan_while_moving && robotIsMoving.load()))
  {
    //ROS_INFO_STREAM("Robot is currently moving, not inserting ROI");
    return;
  }
  const pointcloud_roi_msgs::PointcloudWithRoi &roi = *event.getMessage();

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time cbStartTime = ros::Time::now();
  #endif

  geometry_msgs::TransformStamped pcFrameTf;
  bool cloud_in_map_frame = (roi.cloud.header.frame_id == map_frame);
  if (cloud_in_map_frame)
  {
    ROS_INFO_STREAM("Incoming cloud already in target frame");
    //pcFrameTf.header = roi.cloud.header;
    pcFrameTf.transform = roi.transform;
  }
  else
  {
    ROS_INFO_STREAM("Convert incoming cloud (" << roi.cloud.header.frame_id << ") to map frame (" << map_frame << "), assuming no transform in incoming data");
    try
    {
      pcFrameTf = tfBuffer.lookupTransform(map_frame, roi.cloud.header.frame_id, roi.cloud.header.stamp);
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame in registerRoiOCL: " << e.what());
      return;
    }
  }

  if (!insert_scan_if_not_moved) // check if robot has moved since last update
  {
    Eigen::Affine3d camPoseEigen = tf2::transformToEigen(pcFrameTf.transform);
    static Eigen::Affine3d lastCamPose;
    if (camPoseEigen.isApprox(lastCamPose, 1e-2))
    {
        ROS_INFO("Arm has not moved, not inserting ROI");
        scanInserted.store(true); // to prevent getting stuck
        return;
    }
    lastCamPose = camPoseEigen;
  }

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  //sensor_msgs::PointCloud2 pc_glob;
  //tf2::doTransform(roi.cloud, pc_glob, pcFrameTf);

  octomap::Pointcloud inlierCloud, outlierCloud, fullCloud;
  if (cloud_in_map_frame)
    octomap_vpp::pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, inlierCloud, outlierCloud, fullCloud);
  else
    octomap_vpp::pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, pcFrameTf.transform, inlierCloud, outlierCloud, fullCloud);

  tree_mtx.lock();
  planningTree->insertPointCloud(fullCloud, scan_orig);
  planningTree->insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  scanInserted.store(true);
  if (publish_planning_state)
  {
    state.scan_inserted = true;
    plannerStatePub.publish(state);
  }

  if (record_map_updates)
  {
    plannerBag.write("/detect_roi/results", event);
  }

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time insertTime = ros::Time::now();
  times_mtx.lock();
  times.insert_roi_latest = insertTime - cbStartTime;
  times.num_rois++;
  times.insert_roi_avg += (times.insert_roi_latest - times.insert_roi_avg) * (1.0 / times.num_rois);
  planningTimesPub.publish(times);
  times_mtx.unlock();
  #endif

  //std::vector<octomap::OcTreeKey> roi_keys = testTree.getRoiKeys();
  //ROS_INFO_STREAM("Found " << testTree.getRoiSize() << " ROI keys (" << testTree.getAddedRoiSize() << " added, " << testTree.getDeletedRoiSize() << " removed)");
}

/*void ViewpointPlanner::registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  ROS_INFO_STREAM("REGISTER_NEW_SCAN");
  if (!insert_occ_while_moving && robotIsMoving.load())
  {
    //ROS_INFO_STREAM("Robot is currently moving, not inserting occupancy");
    return;
  }

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time cbStartTime = ros::Time::now();
  #endif

  geometry_msgs::TransformStamped pcFrameTf;

  try
  {
    pcFrameTf = tfBuffer.lookupTransform(map_frame, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame in registerNewScan: " << e.what());
    return;
  }
  //ROS_INFO_STREAM("Transform for time " << pc_msg->header.stamp << " successful");

  if (!insert_occ_if_not_moved) // check if robot has moved since last update
  {
    Eigen::Affine3d camPoseEigen = tf2::transformToEigen(pcFrameTf.transform);
    static Eigen::Affine3d lastCamPose;
    if (camPoseEigen.isApprox(lastCamPose, 1e-2))
    {
        ROS_INFO("Arm has not moved, not inserting occupancy");
        return;
    }
    lastCamPose = camPoseEigen;
 }

  const geometry_msgs::Vector3 &pcfOrig = pcFrameTf.transform.translation;
  octomap::point3d scan_orig(pcfOrig.x, pcfOrig.y, pcfOrig.z);

  //ros::Time tfTime = ros::Time::now();

  sensor_msgs::PointCloud2 pc_glob;
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  //pcGlobalPub.publish(pc_glob);

  //ros::Time doTFTime = ros::Time::now();

  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(pc_glob, pc);

  //ros::Time toOctoTime = ros::Time::now();

  tree_mtx.lock();
  planningTree->insertPointCloud(pc, scan_orig);
  tree_mtx.unlock();

  occupancyScanned.store(true);
  if (publish_planning_state)
  {
    state.occupancy_scanned = true;
    plannerStatePub.publish(state);
  }

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time insertTime = ros::Time::now();
  times_mtx.lock();
  times.insert_scan_latest = insertTime - cbStartTime;
  times.num_scans++;
  times.insert_scan_avg += (times.insert_scan_latest - times.insert_scan_avg) * (1.0 / times.num_scans);
  planningTimesPub.publish(times);
  times_mtx.unlock();
  #endif

  //ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}*/

/*void ViewpointPlanner::registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg)
{
  if (!insert_roi_while_moving && robotIsMoving.load())
  {
    //ROS_INFO_STREAM("Robot is currently moving, not inserting ROI");
    return;
  }

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time cbStartTime = ros::Time::now();
  #endif

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tfBuffer.lookupTransform(map_frame, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame in registerRoi: " << e.what());
    return;
  }

  if (!insert_roi_if_not_moved) // check if robot has moved since last update
  {
    Eigen::Affine3d camPoseEigen = tf2::transformToEigen(pcFrameTf.transform);
    static Eigen::Affine3d lastCamPose;
    if (camPoseEigen.isApprox(lastCamPose, 1e-2))
    {
        ROS_INFO("Arm has not moved, not inserting ROI");
        return;
    }
    lastCamPose = camPoseEigen;
  }

  sensor_msgs::PointCloud2 pc_glob;
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  //ROS_INFO_STREAM("Register ROI called, " << dets_msg->detections.size() << " detections");
  std::unordered_set<size_t> inlier_indices;
  for (const auto &det : dets_msg->detections)
  {
    if (det.class_name != "capsicum") // not region of interest
      continue;

    for (int y = det.box.y1; y < det.box.y2; y++)
    {
      for (int x = det.box.x1; x < det.box.x2; x++)
      {
        if (det.mask.mask[(y - det.box.y1) * det.mask.width + (x - det.box.x1)])
        {
          inlier_indices.insert(y * pc_msg->width + x);
        }
      }
    }
  }

  //ROS_INFO_STREAM("Number of inliers: " << inlier_indices.size());

  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(pc_glob, inlier_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  planningTree->insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  roiScanned.store(true);
  if (publish_planning_state)
  {
    state.roi_scanned = true;
    plannerStatePub.publish(state);
  }

  #ifdef PUBLISH_PLANNING_TIMES
  ros::Time insertTime = ros::Time::now();
  times_mtx.lock();
  times.insert_roi_latest = insertTime - cbStartTime;
  times.num_rois++;
  times.insert_roi_avg += (times.insert_roi_latest - times.insert_roi_avg) * (1.0 / times.num_rois);
  planningTimesPub.publish(times);
  times_mtx.unlock();
  #endif
}*/

octomap::point3d ViewpointPlanner::sampleRandomPointOnSphere(const octomap::point3d &center, double radius)
{
  static std::normal_distribution<double> distribution(0.0, 1.0);
  octomap::point3d p;
  for (size_t i = 0; i < 3; i++)
    p(i) = distribution(random_engine);

  p.normalize();
  p *= radius;
  p += center;
  return p;
}

/*void ViewpointPlanner::getBorderPoints(const octomap::point3d &orig, double maxDist)
{
  std::vector<octomap::pose6d> sampledPoses;
  octomap::KeyRay ray;
  testTree.computeRayKeys(orig, sampleRandomPointOnSphere(orig, maxDist), ray);
  for (const octomap::OcTreeKey &key : ray)
  {
    RoiOcTreeNode *node = testTree.search(key);
    if (node == NULL)

    if (node != NULL)
    {
      double occ = node->getOccupancy();
    }

  }
}*/

/**
 * @brief dirVecToQuat computes quaternion closest to camQuat with x-axis aligned to dirVec
 * @param dirVec desired x-axis direction
 * @param camQuat camera orientation
 * @param viewDir camera view direction (x-axis)
 * @return computed quaternion
 */
tf2::Quaternion ViewpointPlanner::dirVecToQuat(const octomath::Vector3 &dirVec, const tf2::Quaternion &camQuat, const tf2::Vector3 &viewDir)
{
  tf2::Vector3 dirVecTf = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
  tf2::Vector3 rotAx = viewDir.cross(dirVecTf);
  double rotAng = viewDir.angle(dirVecTf);
  tf2::Quaternion toRot(rotAx, rotAng);
  tf2::Quaternion viewQuat = toRot * camQuat;
  return viewQuat;
}

void ViewpointPlanner::publishViewpointVisualizations(const std::vector<Viewpoint> &viewpoints, const std::string &ns, const std_msgs::ColorRGBA &color)
{
  static std::unordered_map<std::string, size_t> last_marker_counts;
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = map_frame;
  poseArr.header.stamp = ros::Time::now();
  bool first = true;
  for (size_t i = 0; i < viewpoints.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    if (first && ns == "roiContourPoints")
    {
      marker.color = COLOR_RED;
      first = false;
    }
    else {
      marker.color = color;
    }
    //marker.lifetime = ros::Duration(2);
    marker.points.push_back(viewpoints[i].pose.position);
    marker.points.push_back(octomap::pointOctomapToMsg(viewpoints[i].target));
    markers.markers.push_back(marker);

    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position = viewpoints[i].pose.position;
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.z = 0.05;
    textMarker.color = color;
    //textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(viewpoints[i].infoGain) + ", " + std::to_string(viewpoints[i].distance) + ", " + std::to_string(viewpoints[i].utility);
    markers.markers.push_back(textMarker);*/

    poseArr.poses.push_back(viewpoints[i].pose);
  }
  size_t last_marker_count = last_marker_counts[ns];
  ROS_INFO_STREAM("Last Marker count namespace: " << ns << ": " << last_marker_count << "; current: " << viewpoints.size());
  for (size_t i = viewpoints.size(); i < last_marker_count; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
    /*visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(textMarker);*/
  }
  last_marker_counts[ns] = viewpoints.size();
  viewArrowVisPub.publish(markers);
  poseArrayPub.publish(poseArr);
}

void ViewpointPlanner::saveViewpointsToBag(const std::vector<Viewpoint> &viewpoints, const std::string &sampling_method, const ros::Time &time)
{
  roi_viewpoint_planner_msgs::ViewpointList list;
  for (const Viewpoint & vp : viewpoints)
  {
    roi_viewpoint_planner_msgs::Viewpoint vp_msg;
    vp_msg.pose = vp.pose;
    vp_msg.target = octomap::pointOctomapToMsg(vp.target);
    vp_msg.infoGain = vp.infoGain;
    vp_msg.distance = vp.distance;
    vp_msg.utility = vp.utility;
    vp_msg.isFree = vp.isFree;
    list.viewpoints.push_back(vp_msg);
  }
  list.header.stamp = time;
  list.sampling_method = sampling_method;
  plannerBag.write("/roi_viewpoint_planner/viewpoints", time, list);
}

void ViewpointPlanner::sampleAroundROICenter(const octomap::point3d &center, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t roiID)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);

  std::vector<octomap::point3d> sampledPoints;
  std::vector<size_t> infoGain;
  std::vector<double> distances;
  std::vector<bool> vpIsFree;
  std::vector<tf2::Quaternion> vpOrientations;
  for (size_t i = 0; i < 10; i++)
  {
    octomap::KeyRay ray;
    octomap::point3d spherePoint = sampleRandomPointOnSphere(center, sampleRandomSensorDistance());
    planningTree->computeRayKeys(center, spherePoint, ray);
    bool view_occluded = false;
    bool has_left_roi = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
      if (node == NULL)
      {
        unknown_nodes++;
        has_left_roi = true;
        last_node_free = false;
        continue;
      }
      if (!has_left_roi && planningTree->isNodeROI(node))
      {
        continue;
      }
      has_left_roi = true;
      double occ = node->getOccupancy();
      if (occ > 0.6) // View is blocked
      {
        view_occluded = true;
        break;
      }
      else if (occ > 0.4) // unknown
      {
        unknown_nodes++;
        last_node_free = false;
      }
      else //free
      {
        last_node_free = true;
      }
    }
    if (!view_occluded && unknown_nodes > 0)
    {
      octomath::Vector3 dirVec = center - spherePoint;
      dirVec.normalize();

      sampledPoints.push_back(spherePoint);
      infoGain.push_back(unknown_nodes);
      distances.push_back((spherePoint - camPos).norm());
      vpIsFree.push_back(last_node_free);

      tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
      vpOrientations.push_back(viewQuat);
    }
  }

  visualization_msgs::MarkerArray markers;
  for (size_t i = 0; i < sampledPoints.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "roiPoints" + roiID;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.02;
    marker.color.a = 1.0;
    size_t colID = roiID % 3;
    marker.color.r = colID == 0 ? 1.0 : 0.0;
    marker.color.g = colID == 1 ? 1.0 : 0.0;
    marker.color.b = colID == 2 ? 1.0 : 0.0;
    marker.lifetime = ros::Duration(2);
    marker.points.push_back(octomap::pointOctomapToMsg(sampledPoints[i]));
    marker.points.push_back(octomap::pointOctomapToMsg(center));
    markers.markers.push_back(marker);

    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = map_frame;
    textMarker.header.stamp = ros::Time();
    textMarker.ns = "roiPoints_texts" + roiID;
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position.x = sampledPoints[i].x();
    textMarker.pose.position.y = sampledPoints[i].y();
    textMarker.pose.position.z = sampledPoints[i].z();
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.z = 0.05;
    textMarker.color.a = 1.0;
    textMarker.color.r = colID == 0 ? 1.0 : 0.0;
    textMarker.color.g = colID == 1 ? 1.0 : 0.0;
    textMarker.color.b = colID == 2 ? 1.0 : 0.0;
    textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(infoGain[i]) + ", " + std::to_string(distances[i]) + ", " + std::to_string(vpIsFree[i]);
    markers.markers.push_back(textMarker);
  }
  viewArrowVisPub.publish(markers);

  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = map_frame;
  poseArr.header.stamp = ros::Time::now();
  for (size_t i = 0; i < sampledPoints.size(); i++)
  {
    geometry_msgs::Pose pose;
    pose.position = octomap::pointOctomapToMsg(sampledPoints[i]);
    pose.orientation = tf2::toMsg(vpOrientations[i]);
    poseArr.poses.push_back(pose);
  }
  poseArrayPub.publish(poseArr);

}

double ViewpointPlanner::computeExpectedRayIGinSamplingTree(const octomap::KeyRay &ray)
{
  double expected_gain = 0;
  //double curProb = 1;
  for (const octomap::OcTreeKey &key : ray)
  {
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
    float reachability = samplingTree ? samplingTree->getReachability(transformToWorkspace(planningTree->keyToCoord(key))) : 1.f;
    if (reachability > 0) reachability = 1.f; // Test: binarize reachability
    if (node == NULL)
    {
      expected_gain += reachability;
      continue;
    }
    double occ = node->getOccupancy();
    if (occ > 0.6) // View is blocked
    {
      break;
    }
    else if (occ > 0.4) // unknown
    {
      expected_gain += reachability;
    }
    /*float logOdds = testTree.keyToLogOdds(key);
    double gain = testTree.computeExpectedInformationGain(logOdds);
    double reachability = samplingTree ? samplingTree->getReachability(transformToWorkspace(planningTree->keyToCoord(key))) : 1.0; // default to 1 if reachability not specified
    //const double RB_WEIGHT = 0.5;
    double weightedGain = reachability * gain;//RB_WEIGHT * reachability + (1 - RB_WEIGHT) * gain;
    expected_gain += weightedGain; // * curProb
    //curProb *= 1 - octomap::probability(logOdds);
    //if (curProb < 0.05) // stop checking cells if probability of being hit is low
    //  break;
    if (logOdds > octomap::logodds(0.9)) // stop if cell is likely occupied
      break;*/
  }
  if (ray.size() > 0)
    expected_gain /= ray.size();

  return expected_gain;
}

double ViewpointPlanner::computeViewpointSamplingValue(const octomap::pose6d &viewpoint, const double &hfov, size_t x_steps, size_t y_steps, const double &maxRange, bool use_roi_weighting)
{
  double f_rec =  2 * tan(hfov / 2) / (double)x_steps;
  double cx = (double)x_steps / 2.0;
  double cy = (double)y_steps / 2.0;
  double value = 0;
  for (size_t i = 0; i < x_steps; i++)
  {
    for(size_t j = 0; j < y_steps; j++)
    {
      double x = (i + 0.5 - cx) * f_rec;
      double y = (j + 0.5 - cy) * f_rec;
      octomap::point3d dir(x, y, 1.0);
      octomap::point3d end = dir * maxRange;
      end = viewpoint.transform(end);
      octomap::KeyRay ray;
      planningTree->computeRayKeys(viewpoint.trans(), end, ray);
      double gain = computeExpectedRayIGinSamplingTree(ray);
      //ROS_INFO_STREAM("Ray (" << i << ", " << j << ") from " << viewpoint.trans() << " to " << end << ": " << ray.size() << " Keys, Gain: " << gain);
      value += gain;
    }
  }
  value /= x_steps * y_steps;
  return value;
}

bool ViewpointPlanner::hasDirectUnknownNeighbour(const octomap::OcTreeKey &key, unsigned int depth)
{
  for (int i = 0; i < 6; i++)
  {
    octomap::OcTreeKey neighbour_key(key);
    neighbour_key[i/2] += i%2 ? 1 : -1;
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) return true;
  }
  return false;
}

bool ViewpointPlanner::hasUnknownNeighbour18(const octomap::OcTreeKey &key, unsigned int depth)
{
  for (int i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) return true;
  }
  return false;
}

bool ViewpointPlanner::hasUnknownAndOccupiedNeighbour6(const octomap::OcTreeKey &key, unsigned int depth)
{
  bool unknown = false, occupied = false;
  for (int i = 0; i < 6 && !(unknown && occupied); i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb6Lut[i][0], key[1] + octomap_vpp::nb6Lut[i][1], key[2] + octomap_vpp::nb6Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) unknown = true;
    else if (node->getLogOdds() > 0) occupied = true;
  }
  return unknown && occupied;
}

bool ViewpointPlanner::hasUnknownAndOccupiedNeighbour18(const octomap::OcTreeKey &key, unsigned int depth)
{
  bool unknown = false, occupied = false;
  for (int i = 0; i < 18 && !(unknown && occupied); i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) unknown = true;
    else if (node->getLogOdds() > 0) occupied = true;
  }
  return unknown && occupied;
}

void ViewpointPlanner::visualizeBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, unsigned int depth)
{
  assert(depth <= planningTree->getTreeDepth());
  if (depth == 0)
    depth = planningTree->getTreeDepth();

  /*octomap::OcTreeKey key_min = testTree.coordToKey(pmin, depth);
  octomap::OcTreeKey key_max = testTree.coordToKey(pmax, depth);

  for (size_t i = 0; i < 3; i++)
  {
    if (key_min[i] > key_max[i]) std::swap(key_min[i], key_max[i]);
  }*/

  std::vector<octomap::point3d> sampledPoints;

  tree_mtx.lock();
  /*octomap::OcTreeKey cur_key;
  for (cur_key[0] = key_min[0]; cur_key[0] <= key_max[0]; cur_key[0]++)
  {
    for (cur_key[1] = key_min[1]; cur_key[1] <= key_max[1]; cur_key[1]++)
    {
      for (cur_key[2] = key_min[2]; cur_key[2] <= key_max[2]; cur_key[2]++)
      {
        RoiOcTreeNode *node = testTree.search(cur_key, depth);
        if (node != NULL && node->getLogOdds() < 0) // is node free; TODO: replace with bounds later
        {
          if (hasDirectUnkownNeighbour(cur_key, depth))
          {
            sampledPoints.push_back(testTree.keyToCoord(cur_key, depth));// add node to border list
          }
        }
      }
    }
  }*/

  for (auto it = planningTree->begin_leafs_bbx(pmin, pmax, depth), end = planningTree->end_leafs_bbx(); it != end; it++)
  {
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasDirectUnknownNeighbour(it.getKey(), depth))
      {
        sampledPoints.push_back(it.getCoordinate());// add node to border list
      }
    }
  }

  tree_mtx.unlock();

  visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame;
  marker.header.stamp = ros::Time();
  marker.ns = "borderPoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  for (const octomap::point3d &point : sampledPoints)
  {
    marker.points.push_back(octomap::pointOctomapToMsg(point));
  }
  pointVisPub.publish( marker );
}

octomap::point3d ViewpointPlanner::transformToMapFrame(const octomap::point3d &p)
{
  if (map_frame == ws_frame)
    return p;

  geometry_msgs::TransformStamped trans;
  try
  {
    trans = tfBuffer.lookupTransform(map_frame, ws_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToWorkspace: " << e.what());
    return p;
  }

  octomap::point3d pt;
  tf2::doTransform(p, pt, trans);
  return pt;
}

geometry_msgs::Pose ViewpointPlanner::transformToMapFrame(const geometry_msgs::Pose &p)
{
  if (map_frame == ws_frame)
    return p;

  geometry_msgs::TransformStamped trans;
  try
  {
    trans = tfBuffer.lookupTransform(map_frame, ws_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToWorkspace: " << e.what());
    return p;
  }

  geometry_msgs::Pose pt;
  tf2::doTransform(p, pt, trans);
  return pt;
}

octomap::point3d ViewpointPlanner::transformToWorkspace(const octomap::point3d &p)
{
  if (map_frame == ws_frame)
    return p;

  geometry_msgs::TransformStamped trans;
  try
  {
    trans = tfBuffer.lookupTransform(ws_frame, map_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToWorkspace: " << e.what());
    return p;
  }

  octomap::point3d pt;
  tf2::doTransform(p, pt, trans);
  return pt;
}

geometry_msgs::Pose ViewpointPlanner::transformToWorkspace(const geometry_msgs::Pose &p)
{
  if (map_frame == ws_frame)
    return p;

  geometry_msgs::TransformStamped trans;
  try
  {
    trans = tfBuffer.lookupTransform(ws_frame, map_frame, ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to ws frame in transformToWorkspace: " << e.what());
    return p;
  }

  geometry_msgs::Pose pt;
  tf2::doTransform(p, pt, trans);
  return pt;
}

octomap::point3d ViewpointPlanner::computeSurfaceNormalDir(const octomap::OcTreeKey &key)
{
  octomap::point3d normalDir;
  for (size_t i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key);
    if (node != NULL && node->getLogOdds() > 0) // occupied
    {
      normalDir -= octomap::point3d(octomap_vpp::nb18Lut[i][0], octomap_vpp::nb18Lut[i][1], octomap_vpp::nb18Lut[i][2]);
    }
  }
  normalDir.normalize();
  return normalDir;
}

octomap::point3d ViewpointPlanner::computeUnknownDir(const octomap::OcTreeKey &key)
{
  octomap::point3d averageUnknownDir;
  for (size_t i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key);
    if (node == NULL || node->getLogOdds() == 0)
    {
      averageUnknownDir += octomap::point3d(octomap_vpp::nb18Lut[i][0], octomap_vpp::nb18Lut[i][1], octomap_vpp::nb18Lut[i][2]);
    }
  }
  averageUnknownDir.normalize();
  return averageUnknownDir;
}

void ViewpointPlanner::getFreeNeighbours6(const octomap::OcTreeKey &key, octomap::KeySet &freeKeys)
{
  for (int i = 0; i < 6; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb6Lut[i][0], key[1] + octomap_vpp::nb6Lut[i][1], key[2] + octomap_vpp::nb6Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(neighbour_key);
    if (node != NULL && node->getLogOdds() < 0) freeKeys.insert(neighbour_key);
  }
}

std::vector<Viewpoint> ViewpointPlanner::sampleAroundMultiROICenters(const std::vector<octomap::point3d> &centers, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);

  boost::mutex::scoped_lock lock(tree_mtx);

  std::vector<Viewpoint> sampledPoints;
  if (centers.empty()) return sampledPoints;
  ros::Time samplingStartTime = ros::Time::now();
  const size_t TOTAL_SAMPLE_TRIES = 100;
  size_t samples_per_center = TOTAL_SAMPLE_TRIES / centers.size();
  for (const octomap::point3d &center : centers)
  {
    for (size_t i = 0; i < samples_per_center; i++)
    {
      Viewpoint vp;
      octomap::KeyRay ray;
      octomap::point3d spherePoint = sampleRandomPointOnSphere(center, sampleRandomSensorDistance());
      octomath::Vector3 dirVec = center - spherePoint;
      dirVec.normalize();
      tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(spherePoint);
      vp.pose.orientation = tf2::toMsg(viewQuat);

      if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(spherePoint)) == NULL) // workspace specified and sampled point not in workspace
      {
        continue;
      }
      if (compute_ik_when_sampling)
      {
        if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
      #else
        manipulator_group.getJointValueTarget(vp.joint_target);
      #endif
      }

      planningTree->computeRayKeys(center, spherePoint, ray);
      bool view_occluded = false;
      bool first_roi_passed = false;
      bool has_left_roi = false;
      bool last_node_free = false;
      size_t unknown_nodes = 0;
      for (const octomap::OcTreeKey &key : ray)
      {
        octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
        if (node == NULL)
        {
          unknown_nodes++;
          if (first_roi_passed) has_left_roi = true;
          last_node_free = false;
          continue;
        }
        if (!has_left_roi && planningTree->isNodeROI(node))
        {
          first_roi_passed = true;
          continue;
        }
        if (first_roi_passed) has_left_roi = true;
        double occ = node->getOccupancy();
        if (has_left_roi && occ > 0.6) // View is blocked
        {
          view_occluded = true;
          break;
        }
        else if (occ > 0.4) // unknown
        {
          unknown_nodes++;
          last_node_free = false;
        }
        else //free
        {
          last_node_free = true;
        }
      }
      if (!view_occluded)
      {
        vp.target = center;
        vp.infoGain = (double)unknown_nodes / (double)ray.size();
        vp.distance = (spherePoint - camPos).norm();
        vp.utility = vp.infoGain - 0.2 * vp.distance;
        vp.isFree = last_node_free;
        sampledPoints.push_back(vp);
      }
    }
  }
  ROS_INFO_STREAM("Time for sampling: " << (ros::Time::now() - samplingStartTime));
  return sampledPoints;
}

std::vector<Viewpoint> ViewpointPlanner::sampleContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);

  boost::mutex::scoped_lock lock(tree_mtx);

  std::vector<octomap::OcTreeKey> contourKeys;
  std::vector<Viewpoint> sampled_vps;
  //size_t total_nodes = 0, jumped_nodes = 0, rejected_nodes = 0, free_nodes = 0;
  octomap::point3d stMin_tf = transformToMapFrame(stMin), stMax_tf = transformToMapFrame(stMax);
  for (size_t i = 0; i < 3; i++)
  {
    if (stMin_tf(i) > stMax_tf(i))
      std::swap(stMin_tf(i), stMax_tf(i));
  }
  for (auto it = planningTree->begin_leafs_bbx(stMin_tf, stMax_tf), end = planningTree->end_leafs_bbx(); it != end; it++)
  {
    //total_nodes++;
    if (samplingTree != NULL && samplingTree->search(transformToWorkspace(it.getCoordinate())) == NULL) // sampling tree specified and sampled point not in sampling tree
    {
      //jumped_nodes++;
      continue;
    }
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      //free_nodes++;
      if (hasUnknownAndOccupiedNeighbour6(it.getKey()))
      {
        contourKeys.push_back(it.getKey());
      }
      //else
      //  rejected_nodes++;
    }
  }

  //ROS_INFO_STREAM("Nodes (T, J, R, F): " << total_nodes << ", " << jumped_nodes << ", " << rejected_nodes << ", " << free_nodes);

  if (contourKeys.empty())
  {
    ROS_INFO_STREAM("No occ-unknown border found");
    return sampled_vps;
  }

  const size_t MAX_SAMPLES = 100;

  std::vector<octomap::OcTreeKey> selected_keys;
  std::sample(contourKeys.begin(), contourKeys.end(), std::back_inserter(selected_keys),
         MAX_SAMPLES, std::mt19937{std::random_device{}()});

  for (const octomap::OcTreeKey &key : selected_keys)
  {
    Viewpoint vp;
    octomath::Vector3 normalDir = computeSurfaceNormalDir(key);
    octomap::point3d contourPoint = planningTree->keyToCoord(key);
    octomap::point3d vpOrig = contourPoint + normalDir * sampleRandomSensorDistance();
    tf2::Quaternion viewQuat = dirVecToQuat(-normalDir, camQuat, viewDir);
    vp.pose.position = octomap::pointOctomapToMsg(vpOrig);
    vp.pose.orientation = tf2::toMsg(viewQuat);
    if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(vpOrig)) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (compute_ik_when_sampling)
    {
      if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
        continue;

    #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
      vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
    #else
      manipulator_group.getJointValueTarget(vp.joint_target);
    #endif
    }
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(vpOrig);
    if (node != NULL && node->getLogOdds() > 0) // Node is occupied
    {
      continue;
    }

    vp.target = contourPoint;
    octomap::pose6d viewpose(vpOrig, octomath::Quaternion(viewQuat.w(), viewQuat.x(), viewQuat.y(), viewQuat.z()));
    vp.infoGain = computeViewpointSamplingValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, sensor_max_range); // planningTree->computeViewpointValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, sensor_max_range, false);
    vp.distance = (vpOrig - camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    sampled_vps.push_back(vp);
  }

  /*visualization_msgs::Marker marker;
  marker.header.frame_id = map_frame;
  marker.header.stamp = ros::Time();
  marker.ns = "borderPoints";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  for (const octomap::OcTreeKey &key : contourKeys)
  {
    marker.points.push_back(octomap::pointOctomapToMsg(planningTree->keyToCoord(key)));
  }
  pointVisPub.publish(marker);*/

  ROS_INFO_STREAM("Border point count: " << contourKeys.size());

  return sampled_vps;
}

std::vector<Viewpoint> ViewpointPlanner::sampleRoiContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  boost::mutex::scoped_lock lock(tree_mtx);

  octomap::KeySet roi = planningTree->getRoiKeys();
  octomap::KeySet freeNeighbours;
  for (const octomap::OcTreeKey &key : roi)
  {
    planningTree->getNeighborsInState(key, freeNeighbours, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, octomap_vpp::NB_6);
    //getFreeNeighbours6(key, freeNeighbours);
  }
  std::vector<octomap::point3d> selectedPoints;
  for (const octomap::OcTreeKey &key : freeNeighbours)
  {
    //if (hasDirectUnknownNeighbour(key))
    if (planningTree->hasNeighborInState(key, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, octomap_vpp::NB_6))
    {
      selectedPoints.push_back(planningTree->keyToCoord(key));
    }
  }

  ROS_INFO_STREAM("Number of ROI contour points: " << selectedPoints.size());

  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  if (selectedPoints.size() == 0)
    return sampledPoints;

  const size_t TOTAL_SAMPLE_TRIES = 100;

  std::uniform_int_distribution<int> distribution(0, selectedPoints.size() - 1);
  for (size_t i = 0; i < TOTAL_SAMPLE_TRIES; i++)
  {
    Viewpoint vp;
    octomap::point3d target = selectedPoints[distribution(random_engine)];
    octomap::KeyRay ray;
    octomap::point3d spherePoint = sampleRandomPointOnSphere(target, sampleRandomSensorDistance());
    octomath::Vector3 dirVec = target - spherePoint;
    dirVec.normalize();
    tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
    vp.pose.position = octomap::pointOctomapToMsg(spherePoint);
    vp.pose.orientation = tf2::toMsg(viewQuat);

    if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(spherePoint)) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (compute_ik_when_sampling)
    {
      if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
        continue;

    #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
      vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
    #else
      manipulator_group.getJointValueTarget(vp.joint_target);
    #endif
    }

    planningTree->computeRayKeys(target, spherePoint, ray);
    bool view_occluded = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
      if (node == NULL)
      {
        unknown_nodes++;
        last_node_free = false;
        continue;
      }
      double occ = node->getOccupancy();
      if (occ > 0.6) // View is blocked
      {
        view_occluded = true;
        break;
      }
      else if (occ > 0.4) // unknown
      {
        unknown_nodes++;
        last_node_free = false;
      }
      else //free
      {
        last_node_free = true;
      }
    }
    if (!view_occluded)
    {
      vp.target = target;
      vp.infoGain = (double)unknown_nodes / (double)ray.size();
      vp.distance = (spherePoint - camPos).norm();
      vp.utility = vp.infoGain - 0.2 * vp.distance;
      vp.isFree = last_node_free;
      sampledPoints.push_back(vp);
    }
  }
  return sampledPoints;
}

std::vector<Viewpoint> ViewpointPlanner::sampleRoiAdjecentCountours(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  boost::mutex::scoped_lock lock(tree_mtx);

  ros::Time inflationBegin = ros::Time::now();
  planningTree->computeInflatedRois(planningTree->getResolution(), 0.1);
  ros::Time inflationEnd = ros::Time::now();
  ROS_INFO_STREAM("Inflation time: " << (inflationEnd - inflationBegin));
  octomap::KeySet roi_keys = planningTree->getRoiKeys();
  octomap::KeySet inflated_roi_keys = planningTree->getInflatedRoiKeys();
  std::vector<octomap::point3d> inflated_contours;

  for (const octomap::OcTreeKey &key : inflated_roi_keys)
  {
    if (samplingTree != NULL && samplingTree->search(transformToWorkspace(planningTree->keyToCoord(key))) == NULL) // sampling tree specified and sampled point not in sampling tree
    {
      continue;
    }
    octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
    if (node != NULL && node->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasUnknownAndOccupiedNeighbour6(key))
      {
        inflated_contours.push_back(planningTree->keyToCoord(key));
      }
    }
  }

  ROS_INFO_STREAM("Roi keys: " << roi_keys.size() << ", Inflated: " << inflated_roi_keys.size() << ", Contours: " << inflated_contours.size());

  if (inflated_contours.size() == 0)
    return sampledPoints;

  const size_t TOTAL_SAMPLE_TRIES = 100;

  std::uniform_int_distribution<int> distribution(0, inflated_contours.size() - 1);
  for (size_t i = 0; i < TOTAL_SAMPLE_TRIES; i++)
  {
    Viewpoint vp;
    octomap::point3d target = inflated_contours[distribution(random_engine)];
    octomap::KeyRay ray;
    octomap::point3d spherePoint = sampleRandomPointOnSphere(target, sampleRandomSensorDistance());
    octomath::Vector3 dirVec = target - spherePoint;
    dirVec.normalize();
    tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
    vp.pose.position = octomap::pointOctomapToMsg(spherePoint);
    vp.pose.orientation = tf2::toMsg(viewQuat);

    if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(spherePoint)) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (compute_ik_when_sampling)
    {
      if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
        continue;

    #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
      vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
    #else
      manipulator_group.getJointValueTarget(vp.joint_target);
    #endif
    }

    planningTree->computeRayKeys(target, spherePoint, ray);
    bool view_occluded = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree->search(key);
      if (node == NULL)
      {
        unknown_nodes++;
        last_node_free = false;
        continue;
      }
      double occ = node->getOccupancy();
      if (occ > 0.6) // View is blocked
      {
        view_occluded = true;
        break;
      }
      else if (occ > 0.4) // unknown
      {
        unknown_nodes++;
        last_node_free = false;
      }
      else //free
      {
        last_node_free = true;
      }
    }
    if (!view_occluded)
    {
      vp.target = target;
      vp.infoGain = (double)unknown_nodes / (double)ray.size();
      vp.distance = (spherePoint - camPos).norm();
      vp.utility = vp.infoGain - 0.2 * vp.distance;
      vp.isFree = last_node_free;
      sampledPoints.push_back(vp);
    }
  }
  return sampledPoints;

}

std::vector<Viewpoint> ViewpointPlanner::sampleExplorationPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  octomap::point3d wsMin_tf = transformToMapFrame(wsMin), wsMax_tf = transformToMapFrame(wsMax);
  octomap::point3d stMin_tf = transformToMapFrame(stMin), stMax_tf = transformToMapFrame(stMax);

  for (size_t i = 0; i < 3; i++)
  {
    if (wsMin_tf(i) > wsMax_tf(i))
      std::swap(wsMin_tf(i), wsMax_tf(i));

    if (stMin_tf(i) > stMax_tf(i))
      std::swap(stMin_tf(i), stMax_tf(i));
  }

  std::uniform_real_distribution<double> wsxDist(wsMin_tf.x(), wsMax_tf.x());
  std::uniform_real_distribution<double> wsyDist(wsMin_tf.y(), wsMax_tf.y());
  std::uniform_real_distribution<double> wszDist(wsMin_tf.z(), wsMax_tf.z());

  std::uniform_real_distribution<double> stxDist(stMin_tf.x(), stMax_tf.x());
  std::uniform_real_distribution<double> styDist(stMin_tf.y(), stMax_tf.y());
  std::uniform_real_distribution<double> stzDist(stMin_tf.z(), stMax_tf.z());

  const size_t TOTAL_SAMPLE_TRIES = 30;

  boost::mutex::scoped_lock lock(tree_mtx);

  for (size_t i = 0; i < TOTAL_SAMPLE_TRIES; i++)
  {
    Viewpoint vp;
    octomap::point3d source(wsxDist(random_engine), wsyDist(random_engine), wszDist(random_engine));
    octomap::point3d target(stxDist(random_engine), styDist(random_engine), stzDist(random_engine));
    octomath::Vector3 dirVec = target - source;
    dirVec.normalize();
    tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
    vp.pose.position = octomap::pointOctomapToMsg(source);
    vp.pose.orientation = tf2::toMsg(viewQuat);

    if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(source)) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (compute_ik_when_sampling)
    {
      if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
        continue;

    #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
      vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
    #else
      manipulator_group.getJointValueTarget(vp.joint_target);
    #endif
    }

    vp.target = target;
    octomap::pose6d viewpose(source, octomath::Quaternion(viewQuat.w(), viewQuat.x(), viewQuat.y(), viewQuat.z()));
    vp.infoGain = computeViewpointSamplingValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    vp.distance = (source - camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    sampledPoints.push_back(vp);
  }

  return sampledPoints;
}

std::vector<Viewpoint> ViewpointPlanner::sampleBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  boost::mutex::scoped_lock lock(tree_mtx);

  std::vector<octomap::OcTreeKey> viewpoint_candidates;

  for (auto it = planningTree->begin_leafs_bbx(pmin, pmax), end = planningTree->end_leafs_bbx(); it != end; it++)
  {
    if (workspaceTree != NULL && workspaceTree->search(transformToWorkspace(it.getCoordinate())) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasDirectUnknownNeighbour(it.getKey()))
      {
        viewpoint_candidates.push_back(it.getKey());
      }
    }
  }

  const size_t MAX_SAMPLES = 30;

  if (viewpoint_candidates.empty())
  {
    ROS_INFO_STREAM("No occ-unknown border found");
    return sampledPoints;
  }

  std::vector<octomap::OcTreeKey> selected_viewpoints;
  std::sample(viewpoint_candidates.begin(), viewpoint_candidates.end(), std::back_inserter(selected_viewpoints),
         MAX_SAMPLES, std::mt19937{std::random_device{}()});

  for (const octomap::OcTreeKey &key : selected_viewpoints)
  {
    Viewpoint vp;

    octomap::point3d point = planningTree->keyToCoord(key);
    octomath::Vector3 dirVec = computeUnknownDir(key);
    tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);
    vp.pose.position = octomap::pointOctomapToMsg(point);
    vp.pose.orientation = tf2::toMsg(viewQuat);

    if (compute_ik_when_sampling)
    {
      if (!manipulator_group.setJointValueTarget(transformToWorkspace(vp.pose), "camera_link"))
        continue;

    #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
      vp.joint_target.reset(new robot_state::RobotState(manipulator_group.getJointValueTarget()));
    #else
      manipulator_group.getJointValueTarget(vp.joint_target);
    #endif
    }


    vp.target = point + dirVec;
    octomap::pose6d viewpose(point, octomath::Quaternion(viewQuat.w(), viewQuat.x(), viewQuat.y(), viewQuat.z()));
    vp.infoGain = computeViewpointSamplingValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    vp.distance = (point - camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    sampledPoints.push_back(vp);// add node to border list
  }

  return sampledPoints;
}



robot_state::RobotStatePtr ViewpointPlanner::sampleNextRobotState(const robot_state::JointModelGroup *joint_model_group, const robot_state::RobotState &current_state)
{
  double maxValue = 0;
  robot_state::RobotStatePtr maxState(new robot_state::RobotState(current_state));
  robot_state::RobotStatePtr curSample(new robot_state::RobotState(current_state));
  for (size_t i = 0; i < 10; i++)
  {
    curSample->setToRandomPositionsNearBy(joint_model_group, current_state, 0.2);
    const Eigen::Affine3d& stateTf = curSample->getGlobalLinkTransform("camera_link");
    auto translation = stateTf.translation();
    auto quaternion = Eigen::Quaterniond(stateTf.linear()).coeffs();
    octomap::pose6d viewpoint(octomap::point3d(translation[0], translation[1], translation[2]), octomath::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));
    tree_mtx.lock();
    double curValue = planningTree->computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    tree_mtx.unlock();
    if (curValue > maxValue)
    {
      *maxState = *curSample;
      maxValue = curValue;
    }
  }
  return maxState;
}

bool ViewpointPlanner::moveToPoseCartesian(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(goal_pose);
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;
  double fraction = manipulator_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction > 0.95) // execute plan if at least 95% of goal reached
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_= trajectory;
    if (safe)
      return safeExecutePlan(plan, async);
    else
      return executePlan(plan, async);
  }
  return false;
}

bool ViewpointPlanner::moveToPose(const geometry_msgs::Pose &goal_pose, bool async, bool safe)
{
  ros::Time setTargetTime = ros::Time::now();
  if (!manipulator_group.setJointValueTarget(goal_pose, "camera_link"))
  {
    ROS_INFO_STREAM("Could not find IK for specified pose (Timeout: " << (ros::Time::now() - setTargetTime) << ")");
    return false;
  }
  ROS_INFO_STREAM("IK solve time: " << (ros::Time::now() - setTargetTime));

  return planAndExecuteFromMoveGroup(async, safe);
}

bool ViewpointPlanner::moveToState(const robot_state::RobotStateConstPtr &goal_state, bool async, bool safe)
{
  if (!manipulator_group.setJointValueTarget(*goal_state))
  {
    ROS_INFO_STREAM("Couldn't set joint target, make sure values are in bounds");
    return false;
  }

  return planAndExecuteFromMoveGroup(async, safe);
}

bool ViewpointPlanner::moveToState(const std::vector<double> &joint_values, bool async, bool safe)
{
  if (!manipulator_group.setJointValueTarget(joint_values))
  {
    ROS_INFO_STREAM("Couldn't set joint target, make sure values are in bounds");
    return false;
  }

  return planAndExecuteFromMoveGroup(async, safe);
}

bool ViewpointPlanner::planAndExecuteFromMoveGroup(bool async, bool safe)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::Time planStartTime = ros::Time::now();
  moveit::core::MoveItErrorCode res = manipulator_group.plan(plan);
  ROS_INFO_STREAM("Planning duration: " << (ros::Time::now() - planStartTime));
  if (res != moveit::core::MoveItErrorCode::SUCCESS)
  {
    ROS_INFO("Could not find plan");
    return false;
  }
  if (safe)
    return safeExecutePlan(plan, async);
  else
    return executePlan(plan, async);
}

bool ViewpointPlanner::safeExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async)
{
  if (require_execution_confirmation)
  {
    std_srvs::Trigger requestExecution;
    if (!requestExecutionConfirmation.call(requestExecution) || !requestExecution.response.success)
    {
      ROS_INFO_STREAM("Plan execution denied");
      return false;
    }
  }

  robotIsMoving.store(true);
  scanInserted.store(false);
  if (publish_planning_state)
  {
    state.robot_is_moving = true;
    state.scan_inserted = false;
    plannerStatePub.publish(state);
  }

  bool res = executePlan(plan, async);

  robotIsMoving.store(false);
  if (publish_planning_state)
  {
    state.robot_is_moving = false;
    plannerStatePub.publish(state);
  }

  timeLogger.saveTime(TimeLogger::PLAN_EXECUTED);

  /*if (!res)
  {
    ROS_INFO("Could not execute plan");
    return false;
  }*/

  if (eval_running)
  {
    for (ros::Rate r(100); !scanInserted; r.sleep()); // wait for scan
    timeLogger.saveTime(TimeLogger::WAITED_FOR_SCAN);
    saveEvaluatorData(rvp_evaluation::computeTrajectoryLength(plan), rvp_evaluation::getTrajectoryDuration(plan));
    timeLogger.saveTime(TimeLogger::EVALUATED);
  }

  return true;
}

bool ViewpointPlanner::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan, bool async)
{
  moveit::core::MoveItErrorCode res;
  if (async)
    res = manipulator_group.asyncExecute(plan);
  else
    res = manipulator_group.execute(plan);

  return (res == moveit::core::MoveItErrorCode::SUCCESS);
}

bool ViewpointPlanner::saveTreeAsObj(const std::string &file_name)
{
  std::vector<octomap::point3d> vertices;
  std::vector<octomap_vpp::Triangle> faces;
  tree_mtx.lock();
  auto isOcc = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return node->getLogOdds() > 0; };
  octomap_vpp::polygonize<octomap_vpp::RoiOcTree>(*planningTree, vertices, faces, isOcc);
  tree_mtx.unlock();
  std::ofstream tree_out(file_name);
  octomap_vpp::generateObj(tree_out, vertices, faces);
  tree_out.close();
  return true;
}

bool ViewpointPlanner::saveROIsAsObj(const std::string &file_name)
{
  std::vector<octomap::point3d> vertices;
  std::vector<octomap_vpp::Triangle> faces;
  tree_mtx.lock();
  auto isOcc = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return tree.isNodeROI(node); };
  octomap_vpp::polygonizeSubset<octomap_vpp::RoiOcTree>(*planningTree, planningTree->getRoiKeys(), vertices, faces, isOcc);
  tree_mtx.unlock();
  std::ofstream tree_out(file_name);
  octomap_vpp::generateObj(tree_out, vertices, faces);
  tree_out.close();
  return true;
}

std::string ViewpointPlanner::saveOctomap(const std::string &name, bool name_is_prefix)
{
  std::stringstream fName;
  fName << name;
  if (name_is_prefix)
  {
    const boost::posix_time::ptime curDateTime = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_facet *const timeFacet = new boost::posix_time::time_facet("%Y-%m-%d-%H-%M-%S");
    fName.imbue(std::locale(fName.getloc(), timeFacet));
    fName << "_" << curDateTime;
  }
  fName << ".ot";
  tree_mtx.lock();
  bool result = planningTree->write(fName.str());
  tree_mtx.unlock();
  return result ? fName.str() : "";
}

int ViewpointPlanner::loadOctomap(const std::string &filename)
{
  octomap_vpp::RoiOcTree *map = nullptr;
  if (boost::algorithm::ends_with(filename, ".bt"))
  {
    map = new octomap_vpp::RoiOcTree(0.1);
    bool success = map->readBinary(filename);
    if (!success)
    {
      delete map;
      return -1;
    }
  }
  else
  {
    octomap::AbstractOcTree *tree =  octomap::AbstractOcTree::read(filename);
    if (!tree)
      return -1;

    map = dynamic_cast<octomap_vpp::RoiOcTree*>(tree);
    if(!map)
    {
      delete tree;
      return -2;
    }
  }
  tree_mtx.lock();
  planningTree.reset(map);
  planningTree->computeRoiKeys();
  tree_mtx.unlock();
  publishMap();
  return 0;
}

void ViewpointPlanner::resetOctomap()
{
  tree_mtx.lock();
  planningTree->clear();
  planningTree->clearRoiKeys();
  tree_mtx.unlock();
  if (!resetMoveitOctomapClient.call(emptySrv))
  {
      ROS_ERROR("Failed to reset moveit octomap");
  }

  publishMap();
}

bool ViewpointPlanner::randomizePlantPositions(const geometry_msgs::Point &min, const geometry_msgs::Point &max, double min_dist)
{
  if (!evaluator)
    return false;

  evaluator->randomizePlantPositions(octomap_vpp::pointToOctomath(min), octomap_vpp::pointToOctomath(max), min_dist);
  return true;
}

void ViewpointPlanner::plannerLoop()
{
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    plannerLoopOnce();
  }
}

bool ViewpointPlanner::plannerLoopOnce()
{
  timeLogger.startLoop();

  if (mode == IDLE || (wait_for_scan && !scanInserted.load()))
  {
    timeLogger.endLoop();
    return false;
  }

  publishMap();

  timeLogger.saveTime(TimeLogger::MAP_PUBLISHED);

  if (mode == MAP_ONLY)
  {
    timeLogger.endLoop();
    return false;
  }

  geometry_msgs::TransformStamped camFrameTf;

  try
  {
    camFrameTf = tfBuffer.lookupTransform(map_frame, "camera_link", ros::Time(0));
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame in plannerLoop: " << e.what());
    timeLogger.endLoop();
    return false;
  }

  const geometry_msgs::Vector3 &camOrigTf = camFrameTf.transform.translation;
  octomap::point3d camOrig(camOrigTf.x, camOrigTf.y, camOrigTf.z);
  octomap::point3d box_min(camOrig.x() - 0.5, camOrig.y() - 0.5, camOrig.z() - 0.5);
  octomap::point3d box_max(camOrig.x() + 0.5, camOrig.y() + 0.5, camOrig.z() + 0.5);
  //getBorderPoints(box_min, box_max);

  tf2::Quaternion camQuat;
  tf2::fromMsg(camFrameTf.transform.rotation, camQuat);

  timeLogger.saveTime(TimeLogger::CAM_POS_COMPUTED);

  if (activate_move_to_see && loop_state == M2S)
  {
    bool m2s_success = false;
    roi_viewpoint_planner_msgs::GetGradient gradSrv;
    if (moveToSeeClient.call(gradSrv))
    {
      const auto &resp = gradSrv.response;
      ROS_INFO_STREAM("Gradient: " << resp.gradient.position.x << ", " << resp.gradient.position.y << ", " << resp.gradient.position.z);
      ROS_INFO_STREAM("Delta: " << resp.delta);

      if (resp.delta > m2s_delta_thresh)
      {
          geometry_msgs::Pose commandPose;
          tf2::Quaternion q_rot;
          tf2::fromMsg(resp.gradient.orientation, q_rot);
          tf2::Quaternion q_new = q_rot * camQuat;
          q_new.normalize();

          commandPose.position.x = camFrameTf.transform.translation.x + resp.gradient.position.x;
          commandPose.position.y = camFrameTf.transform.translation.y + resp.gradient.position.y;
          commandPose.position.z = camFrameTf.transform.translation.z + resp.gradient.position.z;
          commandPose.orientation = tf2::toMsg(q_new);

          eval_lastStep = "M2S";
          timeLogger.saveTime(TimeLogger::MOVE_TO_SEE_APPLIED);
          if (execute_plan && moveToPose(transformToWorkspace(commandPose)))
          {
            ROS_INFO_STREAM("Succesfully moving with move to see");
            m2s_success = true;
            m2s_current_steps++;
          }
      }
      else
      {
        timeLogger.saveTime(TimeLogger::MOVE_TO_SEE_APPLIED);
      }
    }
    else
    {
      ROS_INFO_STREAM("Move to see service not found");
    }
    if (!m2s_success || m2s_current_steps >= m2s_max_steps)
    {
      if (!move_to_see_exclusive)
        loop_state = NORMAL;

      m2s_current_steps = 0;
    }
    timeLogger.endLoop();
    return m2s_success;
  }

  std::vector<Viewpoint> roiSamplingVps, explSamplingVps;
  auto vpComp = [](const Viewpoint &a, const Viewpoint &b)
  {
    //return a.distance > b.distance;
    return a.utility < b.utility;
  };

  // ROI sampling
  if (mode == SAMPLE_ROI_CONTOURS || (mode == SAMPLE_AUTOMATIC && roi_sample_mode == SAMPLE_ROI_CONTOURS))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::ROI_CONTOUR_SAMPLER, this, camOrig, camQuat, roiMaxSamples, roiUtil);
    roiSamplingVps = sampler->sampleViewpoints();
    std::make_heap(roiSamplingVps.begin(), roiSamplingVps.end(), vpComp);
    publishViewpointVisualizations(roiSamplingVps, "roiContourPoints", COLOR_ORANGE);
    if (record_viewpoints)
      saveViewpointsToBag(roiSamplingVps, "roiContourPoints", ros::Time::now());
  }
  else if (mode == SAMPLE_ROI_ADJACENT || (mode == SAMPLE_AUTOMATIC && roi_sample_mode == SAMPLE_ROI_ADJACENT))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::ROI_ADJACENT_SAMPLER, this, camOrig, camQuat, roiMaxSamples, roiUtil);
    roiSamplingVps = sampler->sampleViewpoints();
    std::make_heap(roiSamplingVps.begin(), roiSamplingVps.end(), vpComp);
    publishViewpointVisualizations(roiSamplingVps, "roiAdjPoints", COLOR_RED);
    if (record_viewpoints)
      saveViewpointsToBag(roiSamplingVps, "roiAdjPoints", ros::Time::now());
  }
  else if (mode == SAMPLE_ROI_CENTERS || (mode == SAMPLE_AUTOMATIC && roi_sample_mode == SAMPLE_ROI_CENTERS))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::ROI_CENTER_SAMPLER, this, camOrig, camQuat, roiMaxSamples, roiUtil);
    roiSamplingVps = sampler->sampleViewpoints();
    std::make_heap(roiSamplingVps.begin(), roiSamplingVps.end(), vpComp);
    publishViewpointVisualizations(roiSamplingVps, "roiPoints", COLOR_BROWN);
    if (record_viewpoints)
      saveViewpointsToBag(roiSamplingVps, "roiPoints", ros::Time::now());
  }

  if (mode == SAMPLE_CONTOURS || (mode == SAMPLE_AUTOMATIC && expl_sample_mode == SAMPLE_CONTOURS))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::CONTOUR_SAMPLER, this, camOrig, camQuat, explMaxSamples, explUtil);
    explSamplingVps = sampler->sampleViewpoints();
    std::make_heap(explSamplingVps.begin(), explSamplingVps.end(), vpComp);
    publishViewpointVisualizations(explSamplingVps, "contourPoints", COLOR_GREEN);
    if (record_viewpoints)
      saveViewpointsToBag(explSamplingVps, "contourPoints", ros::Time::now());
  }
  else if (mode == SAMPLE_BORDER || (mode == SAMPLE_AUTOMATIC && expl_sample_mode == SAMPLE_BORDER))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::BORDER_SAMPLER, this, camOrig, camQuat, explMaxSamples, explUtil);
    explSamplingVps = sampler->sampleViewpoints();
    std::make_heap(explSamplingVps.begin(), explSamplingVps.end(), vpComp);
    publishViewpointVisualizations(explSamplingVps, "borderPoints", COLOR_BLUE);
    if (record_viewpoints)
      saveViewpointsToBag(explSamplingVps, "borderPoints", ros::Time::now());
  }
  else if (mode == SAMPLE_EXPLORATION || (mode == SAMPLE_AUTOMATIC && expl_sample_mode == SAMPLE_EXPLORATION))
  {
    std::unique_ptr<SamplerBase> sampler = createSampler(SamplerType::EXPLORATION_SAMPLER, this, camOrig, camQuat, explMaxSamples, explUtil);
    explSamplingVps = sampler->sampleViewpoints();
    std::make_heap(explSamplingVps.begin(), explSamplingVps.end(), vpComp);
    publishViewpointVisualizations(explSamplingVps, "explorationPoints", COLOR_VIOLET);
    if (record_viewpoints)
      saveViewpointsToBag(explSamplingVps, "explorationPoints", ros::Time::now());
  }

  timeLogger.saveTime(TimeLogger::VIEWPOINTS_SAMPLED);

  //std::make_heap(roiViewpoints.begin(), roiViewpoints.end(), vpComp);
  //std::make_heap(borderVps.begin(), borderVps.end(), vpComp);
  //std::make_heap(contourVps.begin(), contourVps.end(), vpComp);

  if (!execute_plan)
  {
    timeLogger.endLoop();
    return false;
  }

  std::vector<Viewpoint> &nextViewpoints = [&]() -> std::vector<Viewpoint>&
  {
    if (mode == SAMPLE_ROI_CENTERS || mode == SAMPLE_ROI_CONTOURS || mode == SAMPLE_ROI_ADJACENT) return roiSamplingVps;
    else if (mode == SAMPLE_CONTOURS || mode == SAMPLE_ROI_ADJACENT || mode == SAMPLE_EXPLORATION) return explSamplingVps;

    if (roiSamplingVps.size() > 0 && roiSamplingVps.front().utility > 0.2)
    {
      ROS_INFO_STREAM("Using ROI viewpoint targeting");
      eval_lastStep = "ROI sampling";
      return roiSamplingVps;
    }
    else// if (!contourVps.empty())
    {
      ROS_INFO_STREAM("Using exploration viewpoints");
      eval_lastStep = "Expl sampling";
      return explSamplingVps;
    }
    /*else
    {
      ROS_INFO_STREAM("Using border viewpoints");
      return borderVps;
    }*/
  }();

  timeLogger.saveTime(TimeLogger::VIEWPOINTS_SELECTED);

  bool move_success = false;
  for (/*std::make_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp)*/; !nextViewpoints.empty(); std::pop_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp), nextViewpoints.pop_back())
  {
    const Viewpoint &vp = nextViewpoints.front();
    /*if (vp.utility <= 0)
    {
      ROS_INFO_STREAM("No viewpoint with sufficient utility found.");
      break;
    }*/
    if (use_cartesian_motion)
    {
      if (moveToPoseCartesian(transformToWorkspace(vp.pose)))
      {
        move_success = true;
        break;
      }
    }
    else
    {
      if (compute_ik_when_sampling)
      {
        if (moveToState(vp.joint_target))
        {
          move_success = true;
          break;
        }
      }
      else
      {
        if (moveToPose(transformToWorkspace(vp.pose)))
        {
          move_success = true;
          break;
        }
      }
    }
  }
  if (move_success && activate_move_to_see)
  {
      loop_state = M2S;
      m2s_current_steps = 0;
  }
  timeLogger.endLoop();
  return move_success;
}


} // namespace roi_viewpoint_planner
