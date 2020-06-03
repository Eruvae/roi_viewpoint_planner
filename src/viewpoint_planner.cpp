#include "viewpoint_planner.h"

#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include "octomap_vpp/marching_cubes.h"

ViewpointPlanner::ViewpointPlanner(ros::NodeHandle &nh, ros::NodeHandle &nhp, const std::string &wstree_file, double tree_resolution) :
  planningTree(tree_resolution),
  workspaceTree(NULL),
  wsMin(-FLT_MAX, -FLT_MAX, -FLT_MAX),
  wsMax(FLT_MAX, FLT_MAX, FLT_MAX),
  depthCloudSub(nh, PC_TOPIC, 1),
  tfBuffer(ros::Duration(30)),
  tfListener(tfBuffer),
  manipulator_group("manipulator"),
  robot_model_loader("robot_description"),
  kinematic_model(robot_model_loader.getModel()),
  joint_model_group(kinematic_model->getJointModelGroup("manipulator")),
  kinematic_state(new robot_state::RobotState(kinematic_model)),
  mode(IDLE),
  execute_plan(false),
  robotIsMoving(false),
  occupancyScanned(false),
  roiScanned(false)
{
  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflated_octomap", 1);
  //pcGlobalPub = nh.advertise<sensor_msgs::PointCloud2>(PC_GLOBAL, 1);
  //planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  pointVisPub = nh.advertise<visualization_msgs::Marker>("border_marker", 1);
  viewArrowVisPub = nh.advertise<visualization_msgs::MarkerArray>("roi_vp_marker", 1);
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("vp_array", 1);
  workspaceTreePub = nh.advertise<octomap_msgs::Octomap>("workspace_tree", 1, true);
  cubeVisPub = nh.advertise<visualization_msgs::Marker>("cube_vis", 1);

  plannerStatePub = nhp.advertise<roi_viewpoint_planner::PlannerState>("planner_state", 1, true);

  requestExecutionConfirmation = nhp.serviceClient<std_srvs::Trigger>("request_execution_confirmation");

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
      ws_msg.header.frame_id = "world";
      ws_msg.header.stamp = ros::Time::now();
      bool msg_generated = octomap_msgs::fullMapToMsg(*workspaceTree, ws_msg);
      if (msg_generated)
      {
        workspaceTreePub.publish(ws_msg);
      }
    }
  }


  depthCloudSub.registerCallback(&ViewpointPlanner::registerNewScan, this);
  //tfCloudFilter.registerCallback(registerNewScan);
  //cloudCache.registerCallback(registerNewScan);

  //syncDets.registerCallback(registerRoi);

  roiSub = nh.subscribe("/pointcloud_roi", 1, &ViewpointPlanner::registerRoiPCL, this);

  if (workspaceTree)
  {
    manipulator_group.setWorkspace(wsMin.x(), wsMin.y(), wsMin.z(), wsMax.x(), wsMax.y(), wsMax.z());

    visualization_msgs::Marker ws_cube;
    ws_cube.header.frame_id = "world";
    ws_cube.header.stamp = ros::Time::now();
    ws_cube.ns = "ws_cube";
    ws_cube.id = 0;
    ws_cube.type = visualization_msgs::Marker::LINE_LIST;
    ws_cube.color.a = 1.0;
    ws_cube.color.r = 1.0;
    ws_cube.color.g = 0.0;
    ws_cube.color.b = 0.0;
    ws_cube.scale.x = 0.002;
    addCubeEdges(wsMin, wsMax, ws_cube.points);

    cubeVisPub.publish(ws_cube);
  }
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
  map_msg.header.frame_id = MAP_FRAME;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(planningTree, map_msg);
  tree_mtx.unlock();
  if (msg_generated)
  {
    octomapPub.publish(map_msg);
    //publishOctomapToPlanningScene(map_msg);
  }

  //if (octomap_msgs::binaryMapToMsg(testTree, map_msg))
  //    octomapPub.publish(map_msg);

  /*tree_mtx.lock();
  ros::Time inflationStart = ros::Time::now();
  std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflatedTree = planningTree.computeInflatedRois();
  ros::Time inflationDone = ros::Time::now();
  tree_mtx.unlock();
  ROS_INFO_STREAM("Time for inflation: " << inflationDone - inflationStart);*/

  std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflatedTree = planningTree.getInflatedRois();

  if (inflatedTree != nullptr)
  {
    octomap_msgs::Octomap inflated_map;
    inflated_map.header.frame_id = MAP_FRAME;
    inflated_map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*inflatedTree, inflated_map))
    {
      inflatedOctomapPub.publish(inflated_map);
    }
  }
}

void ViewpointPlanner::registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  if (!insert_occ_while_moving && robotIsMoving.load())
  {
    ROS_INFO_STREAM("Robot is currently moving, not inserting occupancy");
    return;
  }

  ros::Time cbStartTime = ros::Time::now();
  geometry_msgs::TransformStamped pcFrameTf;

  try
  {
    pcFrameTf = tfBuffer.lookupTransform(MAP_FRAME, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
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

  ros::Time tfTime = ros::Time::now();

  sensor_msgs::PointCloud2 pc_glob;
  tf2::doTransform(*pc_msg, pc_glob, pcFrameTf);

  //pcGlobalPub.publish(pc_glob);

  ros::Time doTFTime = ros::Time::now();

  octomap::Pointcloud pc;
  octomap::pointCloud2ToOctomap(pc_glob, pc);

  ros::Time toOctoTime = ros::Time::now();

  tree_mtx.lock();
  planningTree.insertPointCloud(pc, scan_orig);
  tree_mtx.unlock();

  occupancyScanned.store(true);
  if (publish_planning_state)
  {
    state.occupancy_scanned = true;
    plannerStatePub.publish(state);
  }

  ros::Time insertTime = ros::Time::now();

  ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}

void ViewpointPlanner::pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (indices.find(i) != indices.end())
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       else
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
     }
     else
       invalid_points++;
   }
   //ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

// indices must be ordered!
void ViewpointPlanner::pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<uint32_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
{
   inlierCloud.reserve(indices.size());
   outlierCloud.reserve(cloud.data.size() / cloud.point_step - indices.size());

   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

   size_t invalid_points = 0;
   std::vector<uint32_t>::const_iterator it = indices.begin();
   for (uint32_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i){
     // Check if the point is invalid
     if (std::isfinite (*iter_x) && std::isfinite (*iter_y) && std::isfinite (*iter_z))
     {
       if (it != indices.end() && i == *it)
       {
         inlierCloud.push_back(*iter_x, *iter_y, *iter_z);
         it++;
       }
       else
       {
         outlierCloud.push_back(*iter_x, *iter_y, *iter_z);
       }
     }
     else
       invalid_points++;
   }
   //ROS_INFO_STREAM("Number of invalid points: " << invalid_points);
}

void ViewpointPlanner::registerRoiPCL(const pointcloud_roi_msgs::PointcloudWithRoi &roi)
{
  if (!insert_roi_while_moving && robotIsMoving.load())
  {
    ROS_INFO_STREAM("Robot is currently moving, not inserting ROI");
    return;
  }

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tfBuffer.lookupTransform(MAP_FRAME, roi.cloud.header.frame_id, roi.cloud.header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
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
  tf2::doTransform(roi.cloud, pc_glob, pcFrameTf);

  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(pc_glob, roi.roi_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  planningTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  roiScanned.store(true);
  if (publish_planning_state)
  {
    state.roi_scanned = true;
    plannerStatePub.publish(state);
  }

  //std::vector<octomap::OcTreeKey> roi_keys = testTree.getRoiKeys();
  //ROS_INFO_STREAM("Found " << testTree.getRoiSize() << " ROI keys (" << testTree.getAddedRoiSize() << " added, " << testTree.getDeletedRoiSize() << " removed)");
}

void ViewpointPlanner::registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg)
{
  if (!insert_roi_while_moving && robotIsMoving.load())
  {
    ROS_INFO_STREAM("Robot is currently moving, not inserting ROI");
    return;
  }

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tfBuffer.lookupTransform(MAP_FRAME, pc_msg->header.frame_id, pc_msg->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
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
  planningTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  roiScanned.store(true);
  if (publish_planning_state)
  {
    state.roi_scanned = true;
    plannerStatePub.publish(state);
  }
}

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
tf2::Quaternion ViewpointPlanner::dirVecToQuat(octomath::Vector3 dirVec, const tf2::Quaternion &camQuat, const tf2::Vector3 &viewDir)
{
  tf2::Vector3 dirVecTf = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
  tf2::Vector3 rotAx = viewDir.cross(dirVecTf);
  double rotAng = viewDir.angle(dirVecTf);
  tf2::Quaternion toRot(rotAx, rotAng);
  tf2::Quaternion viewQuat = toRot * camQuat;
  return viewQuat;
}

std::vector<ViewpointPlanner::Viewpoint> ViewpointPlanner::sampleAroundMultiROICenters(const std::vector<octomap::point3d> &centers, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);

  std::vector<Viewpoint> sampledPoints;
  if (centers.empty()) return sampledPoints;
  ros::Time samplingStartTime = ros::Time::now();
  const size_t TOTAL_SAMPLE_TRIES = 100;
  size_t samples_per_center = TOTAL_SAMPLE_TRIES / centers.size();
  for (const octomap::point3d &center : centers)
  {
    for (size_t i = 0; i < samples_per_center; i++)
    {
      octomap::KeyRay ray;
      octomap::point3d spherePoint = sampleRandomPointOnSphere(center, sampleRandomSensorDistance());

      if (workspaceTree != NULL && workspaceTree->search(spherePoint) == NULL) // workspace specified and sampled point not in workspace
      {
        continue;
      }

      planningTree.computeRayKeys(center, spherePoint, ray);
      bool view_occluded = false;
      bool first_roi_passed = false;
      bool has_left_roi = false;
      bool last_node_free = false;
      size_t unknown_nodes = 0;
      for (const octomap::OcTreeKey &key : ray)
      {
        octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
        if (node == NULL)
        {
          unknown_nodes++;
          if (first_roi_passed) has_left_roi = true;
          last_node_free = false;
          continue;
        }
        if (!has_left_roi && planningTree.isNodeROI(node))
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
        octomath::Vector3 dirVec = center - spherePoint;
        dirVec.normalize();
        tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);

        Viewpoint vp;
        vp.point = spherePoint;
        vp.target = center;
        vp.orientation = viewQuat;
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

void ViewpointPlanner::publishViewpointVisualizations(const std::vector<ViewpointPlanner::Viewpoint> &viewpoints, const std::string &ns, const std_msgs::ColorRGBA &color)
{
  static std::unordered_map<std::string, size_t> last_marker_counts;
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = "world";
  poseArr.header.stamp = ros::Time::now();
  for (size_t i = 0; i < viewpoints.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
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
    marker.color = color;
    //marker.lifetime = ros::Duration(2);
    marker.points.push_back(octomap::pointOctomapToMsg(viewpoints[i].point));
    marker.points.push_back(octomap::pointOctomapToMsg(viewpoints[i].target));
    markers.markers.push_back(marker);

    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::ADD;
    textMarker.pose.position.x = viewpoints[i].point.x();
    textMarker.pose.position.y = viewpoints[i].point.y();
    textMarker.pose.position.z = viewpoints[i].point.z();
    textMarker.pose.orientation.x = 0.0;
    textMarker.pose.orientation.y = 0.0;
    textMarker.pose.orientation.z = 0.0;
    textMarker.pose.orientation.w = 1.0;
    textMarker.scale.z = 0.05;
    textMarker.color = color;
    //textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(viewpoints[i].infoGain) + ", " + std::to_string(viewpoints[i].distance) + ", " + std::to_string(viewpoints[i].utility);
    markers.markers.push_back(textMarker);

    geometry_msgs::Pose pose;
    pose.position = octomap::pointOctomapToMsg(viewpoints[i].point);
    pose.orientation = tf2::toMsg(viewpoints[i].orientation);
    poseArr.poses.push_back(pose);
  }
  size_t last_marker_count = last_marker_counts[ns];
  ROS_INFO_STREAM("Last Marker count namespace: " << ns << ": " << last_marker_count << "; current: " << viewpoints.size());
  for (size_t i = viewpoints.size(); i < last_marker_count; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(marker);
    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp = ros::Time();
    textMarker.ns = ns + "_texts";
    textMarker.id = i;
    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    textMarker.action = visualization_msgs::Marker::DELETE;
    markers.markers.push_back(textMarker);
  }
  last_marker_counts[ns] = viewpoints.size();
  viewArrowVisPub.publish(markers);
  poseArrayPub.publish(poseArr);
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
    planningTree.computeRayKeys(center, spherePoint, ray);
    bool view_occluded = false;
    bool has_left_roi = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
      if (node == NULL)
      {
        unknown_nodes++;
        has_left_roi = true;
        last_node_free = false;
        continue;
      }
      if (!has_left_roi && planningTree.isNodeROI(node))
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
    marker.header.frame_id = "world";
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
    textMarker.header.frame_id = "world";
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
  poseArr.header.frame_id = "world";
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

double ViewpointPlanner::computeExpectedRayIGinWorkspace(const octomap::KeyRay &ray)
{
  double expected_gain = 0;
  //double curProb = 1;
  for (const octomap::OcTreeKey &key : ray)
  {
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
    float reachability = workspaceTree ? workspaceTree->getReachability(planningTree.keyToCoord(key)) : 1.f;
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
    double reachability = workspaceTree ? workspaceTree->getReachability(planningTree.keyToCoord(key)) : 1.0; // default to 1 if reachability not specified
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

double ViewpointPlanner::computeViewpointWorkspaceValue(const octomap::pose6d &viewpoint, const double &hfov, size_t x_steps, size_t y_steps, const double &maxRange, bool use_roi_weighting)
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
      planningTree.computeRayKeys(viewpoint.trans(), end, ray);
      double gain = computeExpectedRayIGinWorkspace(ray);
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
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) return true;
  }
  return false;
}

bool ViewpointPlanner::hasUnknownNeighbour18(const octomap::OcTreeKey &key, unsigned int depth)
{
  for (int i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key, depth);
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
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key, depth);
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
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) unknown = true;
    else if (node->getLogOdds() > 0) occupied = true;
  }
  return unknown && occupied;
}

void ViewpointPlanner::visualizeBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, unsigned int depth)
{
  assert(depth <= planningTree.getTreeDepth());
  if (depth == 0)
    depth = planningTree.getTreeDepth();

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

  for (auto it = planningTree.begin_leafs_bbx(pmin, pmax, depth), end = planningTree.end_leafs_bbx(); it != end; it++)
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
  marker.header.frame_id = "world";
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

octomap::point3d ViewpointPlanner::computeSurfaceNormalDir(const octomap::OcTreeKey &key)
{
  octomap::point3d normalDir;
  for (size_t i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key);
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
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key);
    if (node == NULL || node->getLogOdds() == 0)
    {
      averageUnknownDir += octomap::point3d(octomap_vpp::nb18Lut[i][0], octomap_vpp::nb18Lut[i][1], octomap_vpp::nb18Lut[i][2]);
    }
  }
  averageUnknownDir.normalize();
  return averageUnknownDir;
}

std::vector<ViewpointPlanner::Viewpoint> ViewpointPlanner::sampleContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<octomap::OcTreeKey> contourKeys;
  std::vector<Viewpoint> sampled_vps;
  for (auto it = planningTree.begin_leafs_bbx(wsMin, wsMax), end = planningTree.end_leafs_bbx(); it != end; it++)
  {
    if (workspaceTree != NULL && workspaceTree->search(it.getCoordinate()) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasUnknownAndOccupiedNeighbour6(it.getKey()))
      {
        contourKeys.push_back(it.getKey());
      }
    }
  }

  if (contourKeys.empty())
  {
    ROS_INFO_STREAM("No occ-unknown border found");
    return sampled_vps;
  }

  const size_t MAX_SAMPLES = 30;

  std::vector<octomap::OcTreeKey> selected_keys;
  std::sample(contourKeys.begin(), contourKeys.end(), std::back_inserter(selected_keys),
         MAX_SAMPLES, std::mt19937{std::random_device{}()});

  for (const octomap::OcTreeKey &key : selected_keys)
  {
    octomath::Vector3 normalDir = computeSurfaceNormalDir(key);
    octomap::point3d contourPoint = planningTree.keyToCoord(key);
    octomap::point3d vpOrig = contourPoint + normalDir * sampleRandomSensorDistance();
    if (workspaceTree != NULL && workspaceTree->search(vpOrig) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(vpOrig);
    if (node != NULL && node->getLogOdds() > 0) // Node is occupied
    {
      continue;
    }
    Viewpoint vp;
    vp.point = vpOrig;
    vp.target = contourPoint;
    vp.orientation = dirVecToQuat(-normalDir, camQuat, viewDir);
    octomap::pose6d viewpose(vp.point, octomath::Quaternion(vp.orientation.w(), vp.orientation.x(), vp.orientation.y(), vp.orientation.z()));
    vp.infoGain = computeViewpointWorkspaceValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, sensor_max_range); // planningTree.computeViewpointValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, sensor_max_range, false);
    vp.distance = (vp.point - camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    sampled_vps.push_back(vp);
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
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
    marker.points.push_back(octomap::pointOctomapToMsg(planningTree.keyToCoord(key)));
  }
  pointVisPub.publish(marker);

  ROS_INFO_STREAM("Border point count: " << contourKeys.size());

  return sampled_vps;
}

void ViewpointPlanner::getFreeNeighbours6(const octomap::OcTreeKey &key, octomap::KeySet &freeKeys)
{
  for (int i = 0; i < 6; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb6Lut[i][0], key[1] + octomap_vpp::nb6Lut[i][1], key[2] + octomap_vpp::nb6Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(neighbour_key);
    if (node != NULL && node->getLogOdds() < 0) freeKeys.insert(neighbour_key);
  }
}

std::vector<ViewpointPlanner::Viewpoint> ViewpointPlanner::sampleRoiContourPoints(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  octomap::KeySet roi = planningTree.getRoiKeys();
  octomap::KeySet freeNeighbours;
  for (const octomap::OcTreeKey &key : roi)
  {
    getFreeNeighbours6(key, freeNeighbours);
  }
  std::vector<octomap::point3d> selectedPoints;
  for (const octomap::OcTreeKey &key : freeNeighbours)
  {
    if (hasDirectUnknownNeighbour(key))
    {
      selectedPoints.push_back(planningTree.keyToCoord(key));
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
    octomap::point3d target = selectedPoints[distribution(random_engine)];
    octomap::KeyRay ray;
    octomap::point3d spherePoint = sampleRandomPointOnSphere(target, sampleRandomSensorDistance());

    if (workspaceTree != NULL && workspaceTree->search(spherePoint) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }

    planningTree.computeRayKeys(target, spherePoint, ray);
    bool view_occluded = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
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
      octomath::Vector3 dirVec = target - spherePoint;
      dirVec.normalize();
      tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);

      Viewpoint vp;
      vp.point = spherePoint;
      vp.target = target;
      vp.orientation = viewQuat;
      vp.infoGain = (double)unknown_nodes / (double)ray.size();
      vp.distance = (spherePoint - camPos).norm();
      vp.utility = vp.infoGain - 0.2 * vp.distance;
      vp.isFree = last_node_free;
      sampledPoints.push_back(vp);
    }
  }
  return sampledPoints;
}

std::vector<ViewpointPlanner::Viewpoint> ViewpointPlanner::sampleRoiAdjecentCountours(const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  tree_mtx.lock();

  ros::Time inflationBegin = ros::Time::now();
  planningTree.computeInflatedRois(planningTree.getResolution(), 0.1);
  ros::Time inflationEnd = ros::Time::now();
  ROS_INFO_STREAM("Inflation time: " << (inflationEnd - inflationBegin));
  octomap::KeySet roi_keys = planningTree.getRoiKeys();
  octomap::KeySet inflated_roi_keys = planningTree.getInflatedRoiKeys();
  std::vector<octomap::point3d> inflated_contours;

  for (const octomap::OcTreeKey &key : inflated_roi_keys)
  {
    if (workspaceTree != NULL && workspaceTree->search(planningTree.keyToCoord(key)) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }
    octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
    if (node != NULL && node->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasUnknownAndOccupiedNeighbour6(key))
      {
        inflated_contours.push_back(planningTree.keyToCoord(key));
      }
    }
  }

  tree_mtx.unlock();

  ROS_INFO_STREAM("Roi keys: " << roi_keys.size() << ", Inflated: " << inflated_roi_keys.size() << ", Contours: " << inflated_contours.size());

  if (inflated_contours.size() == 0)
    return sampledPoints;

  const size_t TOTAL_SAMPLE_TRIES = 100;

  std::uniform_int_distribution<int> distribution(0, inflated_contours.size() - 1);
  for (size_t i = 0; i < TOTAL_SAMPLE_TRIES; i++)
  {
    octomap::point3d target = inflated_contours[distribution(random_engine)];
    octomap::KeyRay ray;
    octomap::point3d spherePoint = sampleRandomPointOnSphere(target, sampleRandomSensorDistance());

    if (workspaceTree != NULL && workspaceTree->search(spherePoint) == NULL) // workspace specified and sampled point not in workspace
    {
      continue;
    }

    planningTree.computeRayKeys(target, spherePoint, ray);
    bool view_occluded = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planningTree.search(key);
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
      octomath::Vector3 dirVec = target - spherePoint;
      dirVec.normalize();
      tf2::Quaternion viewQuat = dirVecToQuat(dirVec, camQuat, viewDir);

      Viewpoint vp;
      vp.point = spherePoint;
      vp.target = target;
      vp.orientation = viewQuat;
      vp.infoGain = (double)unknown_nodes / (double)ray.size();
      vp.distance = (spherePoint - camPos).norm();
      vp.utility = vp.infoGain - 0.2 * vp.distance;
      vp.isFree = last_node_free;
      sampledPoints.push_back(vp);
    }
  }
  return sampledPoints;

}

std::vector<ViewpointPlanner::Viewpoint> ViewpointPlanner::sampleBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  tree_mtx.lock();

  std::vector<octomap::OcTreeKey> viewpoint_candidates;

  for (auto it = planningTree.begin_leafs_bbx(pmin, pmax), end = planningTree.end_leafs_bbx(); it != end; it++)
  {
    if (workspaceTree != NULL && workspaceTree->search(it.getCoordinate()) == NULL) // workspace specified and sampled point not in workspace
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
    tree_mtx.unlock();
    return sampledPoints;
  }

  std::vector<octomap::OcTreeKey> selected_viewpoints;
  std::sample(viewpoint_candidates.begin(), viewpoint_candidates.end(), std::back_inserter(selected_viewpoints),
         MAX_SAMPLES, std::mt19937{std::random_device{}()});

  for (const octomap::OcTreeKey &key : selected_viewpoints)
  {
    Viewpoint vp;
    vp.point = planningTree.keyToCoord(key);
    octomath::Vector3 dirVec = computeUnknownDir(key);
    vp.target = vp.point + dirVec;
    vp.orientation = dirVecToQuat(dirVec, camQuat, viewDir);
    octomap::pose6d viewpose(vp.point, octomath::Quaternion(vp.orientation.w(), vp.orientation.x(), vp.orientation.y(), vp.orientation.z()));
    vp.infoGain = computeViewpointWorkspaceValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    vp.distance = (vp.point - camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    sampledPoints.push_back(vp);// add node to border list
  }

  tree_mtx.unlock();
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
    double curValue = planningTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    tree_mtx.unlock();
    if (curValue > maxValue)
    {
      *maxState = *curSample;
      maxValue = curValue;
    }
  }
  return maxState;
}

bool ViewpointPlanner::moveToPoseCartesian(moveit::planning_interface::MoveGroupInterface &manipulator_group, const geometry_msgs::Pose &goal_pose)
{
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(goal_pose);
  moveit_msgs::RobotTrajectory trajectory;
  const double eef_step = 0.005;
  const double jump_threshold = 0.0;
  double fraction = manipulator_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  if (fraction > 0.95) // execute plan if at least 95% of goal reached
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
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_= trajectory;
    robotIsMoving.store(true);
    occupancyScanned.store(false);
    roiScanned.store(false);
    if (publish_planning_state)
    {
      state.robot_is_moving = true;
      state.occupancy_scanned = false;
      state.roi_scanned = false;
      plannerStatePub.publish(state);
    }

    manipulator_group.execute(plan);
    robotIsMoving.store(false);
    if (publish_planning_state)
    {
      state.robot_is_moving = false;
      plannerStatePub.publish(state);
    }

  }
  return true;
}

bool ViewpointPlanner::moveToPose(moveit::planning_interface::MoveGroupInterface &manipulator_group, const geometry_msgs::Pose &goal_pose)
{
  manipulator_group.setPoseReferenceFrame(MAP_FRAME);
  manipulator_group.setPlannerId(planner_id);
  manipulator_group.setPlanningTime(planning_time);
  ros::Time setTargetTime = ros::Time::now();
  if (!manipulator_group.setJointValueTarget(goal_pose, "camera_link"))
  {
    ROS_INFO_STREAM("Could not find IK for specified pose (Timeout: " << (ros::Time::now() - setTargetTime) << ")");
    return false;
  }
  ROS_INFO_STREAM("IK solve time: " << (ros::Time::now() - setTargetTime));

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  ros::Time planStartTime = ros::Time::now();
  bool success = (manipulator_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_STREAM("Planning duration: " << (ros::Time::now() - planStartTime));

  if (success)
  {
    //manipulator_group.asyncExecute(plan);
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
    occupancyScanned.store(false);
    roiScanned.store(false);
    if (publish_planning_state)
    {
      state.robot_is_moving = true;
      state.occupancy_scanned = false;
      state.roi_scanned = false;
      plannerStatePub.publish(state);
    }
    manipulator_group.execute(plan);
    robotIsMoving.store(false);
    if (publish_planning_state)
    {
      state.robot_is_moving = false;
      plannerStatePub.publish(state);
    }
  }
  else
  {
    ROS_INFO("Could not find plan");
  }
  return success;
}

bool ViewpointPlanner::moveToState(const robot_state::RobotState &goal_state, bool async)
{
  manipulator_group.setJointValueTarget(goal_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (manipulator_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    if (async)
      manipulator_group.asyncExecute(plan);
    else
      manipulator_group.execute(plan);
  }
  else
  {
    ROS_INFO("Can't execute plan");
  }
  return success;
}

bool ViewpointPlanner::moveToState(const std::vector<double> &joint_values, bool async)
{
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  manipulator_group.setJointValueTarget(*kinematic_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (manipulator_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    if (async)
      manipulator_group.asyncExecute(plan);
    else
      manipulator_group.execute(plan);
  }
  else
  {
    ROS_INFO("Can't execute plan");
  }
  return success;
}

bool ViewpointPlanner::saveTreeAsObj(const std::string &file_name)
{
  std::vector<octomap::point3d> vertices;
  std::vector<octomap_vpp::Triangle> faces;
  tree_mtx.lock();
  auto isOcc = [](const octomap_vpp::RoiOcTree &tree, const octomap_vpp::RoiOcTreeNode *node) { return node->getLogOdds() > 0; };
  octomap_vpp::polygonize<octomap_vpp::RoiOcTree>(planningTree, vertices, faces, isOcc);
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
  octomap_vpp::polygonizeSubset<octomap_vpp::RoiOcTree>(planningTree, planningTree.getRoiKeys(), vertices, faces, isOcc);
  tree_mtx.unlock();
  std::ofstream tree_out(file_name);
  octomap_vpp::generateObj(tree_out, vertices, faces);
  tree_out.close();
  return true;
}

void ViewpointPlanner::plannerLoop()
{
  for (ros::Rate rate(100); ros::ok(); rate.sleep())
  {
    /*std::vector<double> mg_joint_values = manipulator_group.getCurrentJointValues();
    kinematic_state->setJointGroupPositions(joint_model_group, mg_joint_values);
    kinematic_state->setToRandomPositionsNearBy(joint_model_group, *kinematic_state, 0.2);
    const Eigen::Affine3d& stateTf = kinematic_state->getGlobalLinkTransform("camera_link");
    auto translation = stateTf.translation();
    auto quaternion = Eigen::Quaterniond(stateTf.linear()).coeffs();
    octomap::pose6d viewpoint(octomap::point3d(translation[0], translation[1], translation[2]), octomath::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));*/

    if ((wait_for_occ_scan && !occupancyScanned.load()) || (wait_for_roi_scan && !roiScanned.load()))
      continue;

    publishMap();

    if (mode == IDLE)
      continue;

    /*tree_mtx.lock();
    ros::Time vpEvalStart = ros::Time::now();
    double value = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    double value_no_weighting = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0, false) / 2;
    ros::Time vpEvalDone = ros::Time::now();
    tree_mtx.unlock();

    ROS_INFO_STREAM("Viewpoint value: " << value << " / " << value_no_weighting << "; Computation time: " << (vpEvalDone - vpEvalStart));*/

    /*std::vector<double> mg_joint_values = manipulator_group.getCurrentJointValues();
    kinematic_state->setJointGroupPositions(joint_model_group, mg_joint_values);
    kinematic_state = sampleNextRobotState(joint_model_group, *kinematic_state);
    moveToState(manipulator_group, *kinematic_state);*/

    geometry_msgs::TransformStamped camFrameTf;

    try
    {
      camFrameTf = tfBuffer.lookupTransform(MAP_FRAME, "camera_link", ros::Time(0));
    }
    catch (const tf2::TransformException &e)
    {
      ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
      continue;
    }

    const geometry_msgs::Vector3 &camOrigTf = camFrameTf.transform.translation;
    octomap::point3d camOrig(camOrigTf.x, camOrigTf.y, camOrigTf.z);
    octomap::point3d box_min(camOrig.x() - 0.5, camOrig.y() - 0.5, camOrig.z() - 0.5);
    octomap::point3d box_max(camOrig.x() + 0.5, camOrig.y() + 0.5, camOrig.z() + 0.5);
    //getBorderPoints(box_min, box_max);

    tf2::Quaternion camQuat;
    tf2::fromMsg(camFrameTf.transform.rotation, camQuat);

    std::vector<Viewpoint> contourVps, borderVps, roiContourVps, roiCenterVps, roiAdjacentVps;
    auto vpComp = [](const Viewpoint &a, const Viewpoint &b)
    {
      //return a.distance > b.distance;
      return a.utility < b.utility;
    };

    if (mode == SAMPLE_CONTOURS || mode == SAMPLE_AUTOMATIC)
    {
      contourVps = sampleContourPoints(camOrig, camQuat);
      std::make_heap(contourVps.begin(), contourVps.end(), vpComp);
      publishViewpointVisualizations(contourVps, "contourPoints", COLOR_GREEN);
    }

    if (mode == SAMPLE_ROI_CONTOURS/* || mode == SAMPLE_AUTOMATIC*/)
    {
      roiContourVps = sampleRoiContourPoints(camOrig, camQuat);
      std::make_heap(roiContourVps.begin(), roiContourVps.end(), vpComp);
      publishViewpointVisualizations(roiContourVps, "roiContourPoints", COLOR_RED);
    }

    if (mode == SAMPLE_BORDER || ((mode == SAMPLE_AUTOMATIC || mode == SAMPLE_CONTOURS) && contourVps.empty() && roiContourVps.empty()))
    {
      borderVps = sampleBorderPoints(box_min, box_max, camOrig, camQuat);
      std::make_heap(borderVps.begin(), borderVps.end(), vpComp);
      publishViewpointVisualizations(borderVps, "borderPoints", COLOR_BLUE);
    }

    if (mode == SAMPLE_ROI_ADJACENT || mode == SAMPLE_AUTOMATIC)
    {
      roiAdjacentVps = sampleRoiAdjecentCountours(camOrig, camQuat);
      std::make_heap(roiAdjacentVps.begin(), roiAdjacentVps.end(), vpComp);
      publishViewpointVisualizations(roiAdjacentVps, "roiAdjPoints", COLOR_RED);
    }

    std::pair<std::vector<octomap::point3d>, std::vector<octomap::point3d>> clusterCentersWithVol = planningTree.getClusterCentersWithVolume();
    std::vector<octomap::point3d> &clusterCenters = clusterCentersWithVol.first;
    std::vector<octomap::point3d> &clusterVolumes = clusterCentersWithVol.second;
    publishCubeVisualization(cubeVisPub, clusterCenters, clusterVolumes);
    /*ROS_INFO_STREAM("Found " << clusterCenters.size() << " clusters for " << planningTree.getRoiSize() << " ROI cells");
    for(size_t i = 0; i < clusterCenters.size(); i++)
    {
      octomap::point3d &dims = clusterVolumes[i];
      dims *= 100; // Convert m to cm
      double vol = dims.x() * dims.y() * dims.z();
      const octomap::point3d BBX_DIFF(0.2, 0.2, 0.2);
      double disRatio = planningTree.getDiscoveredRatio(clusterCenters[i] - BBX_DIFF, clusterCenters[i] + BBX_DIFF);
      ROS_INFO_STREAM("Cluster at " << clusterCenters[i] << " with volume " << dims.x() << "*" << dims.y() << "*" << dims.z() << " (" << vol << ") cm3; "
                      << "DisRatio: " << disRatio);
    }*/

    //std::vector<octomap::point3d> clusterCenters = testTree.getClusterCenters();
    //ROS_INFO_STREAM("ROI count: " << testTree.getRoiSize() << "; Clusters: " << clusterCenters.size());
    /*for (size_t i = 0; i < clusterCenters.size(); i++)
    {
      ROS_INFO_STREAM("Cluster center: " << clusterCenters[i]);
      sampleAroundROICenter(clusterCenters[i], 0.5, octomap::point3d(camOrig.x, camOrig.y, camOrig.z), camQuat, i);
    }*/

    if (mode == SAMPLE_ROI_CENTERS /*|| mode == SAMPLE_AUTOMATIC*/)
    {
      std::vector<Viewpoint> roiViewpoints = sampleAroundMultiROICenters(clusterCenters, camOrig, camQuat);
      std::make_heap(roiViewpoints.begin(), roiViewpoints.end(), vpComp);
      publishViewpointVisualizations(roiViewpoints, "roiPoints", COLOR_RED);
    }

    //std::make_heap(roiViewpoints.begin(), roiViewpoints.end(), vpComp);
    //std::make_heap(borderVps.begin(), borderVps.end(), vpComp);
    //std::make_heap(contourVps.begin(), contourVps.end(), vpComp);

    if (!execute_plan)
      continue;

    std::vector<Viewpoint> &nextViewpoints = [&]() -> std::vector<Viewpoint>&
    {
      if (mode == SAMPLE_ROI_CENTERS) return roiCenterVps;
      else if (mode == SAMPLE_ROI_CONTOURS) return roiContourVps;
      else if (mode == SAMPLE_CONTOURS) return contourVps.empty() ? borderVps : contourVps;
      else if (mode == SAMPLE_BORDER) return borderVps;
      else if (mode == SAMPLE_ROI_ADJACENT) return roiAdjacentVps;

      if (roiAdjacentVps.size() > 0 && roiAdjacentVps.front().utility > 0.2)
      {
        ROS_INFO_STREAM("Using ROI viewpoint targeting");
        return roiAdjacentVps; //roiContourVps;
      }
      else if (!contourVps.empty())
      {
        ROS_INFO_STREAM("Using exploration viewpoints");
        return contourVps;
      }
      else
      {
        ROS_INFO_STREAM("Using border viewpoints");
        return borderVps;
      }
    }();

    for (/*std::make_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp)*/; !nextViewpoints.empty(); std::pop_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp), nextViewpoints.pop_back())
    {
      const Viewpoint &vp = nextViewpoints.front();
      if (vp.utility <= 0)
      {
        ROS_INFO_STREAM("No viewpoint with sufficient utility found.");
        break;
      }
      geometry_msgs::Pose pose;
      pose.position = octomap::pointOctomapToMsg(vp.point);
      pose.orientation = tf2::toMsg(vp.orientation);
      if (use_cartesian_motion)
      {
        if (moveToPoseCartesian(manipulator_group, pose))
          break;
      }
      else
      {
        if (moveToPose(manipulator_group, pose))
          break;
      }
    }
  }
}
