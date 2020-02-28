#include <ros/ros.h>
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
#include <image_geometry/pinhole_camera_model.h>

#include <instance_segmentation_msgs/Detections.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <boost/thread/mutex.hpp>
#include <random>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseArray.h>

#include "octomap_vpp/RoiOcTree.h"
#include "octomap_vpp/roioctree_utils.h"

octomap_vpp::RoiOcTree testTree(0.02);
tf2_ros::Buffer tfBuffer(ros::Duration(30));
ros::Publisher octomapPub;
ros::Publisher inflatedOctomapPub;
ros::Publisher pcGlobalPub;
ros::Publisher pointVisPub;
ros::Publisher viewArrowVisPub;
ros::Publisher poseArrayPub;
//ros::Publisher planningScenePub;

boost::mutex tree_mtx;

const double OCCUPANCY_THRESH = 0.7;
const double FREE_THRESH = 0.3;

const std::string MAP_FRAME = "world";
const std::string PC_TOPIC = "/move_group/filtered_cloud"; //"/camera/depth/points";
const std::string PC_GLOBAL = "/points_global";

/*void publishOctomapToPlanningScene(const octomap_msgs::Octomap &map_msg)
{
  moveit_msgs::PlanningScene scene;
  scene.world.octomap.header = map_msg.header;
  scene.world.octomap.octomap = map_msg;
  scene.world.octomap.octomap.id = "OcTree";
  scene.is_diff = true;
  planningScenePub.publish(scene);
}*/

void publishMap()
{
  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = MAP_FRAME;
  map_msg.header.stamp = ros::Time::now();
  tree_mtx.lock();
  bool msg_generated = octomap_msgs::fullMapToMsg(testTree, map_msg);
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
  std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflatedTree = testTree.computeInflatedRois();
  ros::Time inflationDone = ros::Time::now();
  tree_mtx.unlock();

  ROS_INFO_STREAM("Time for inflation: " << inflationDone - inflationStart);
  if (inflatedTree != nullptr)
  {
    octomap_msgs::Octomap inflated_map;
    inflated_map.header.frame_id = MAP_FRAME;
    inflated_map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(*inflatedTree, inflated_map))
    {
      inflatedOctomapPub.publish(inflated_map);
    }
  }*/
}

void registerNewScan(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
  ros::Time cbStartTime = ros::Time::now();
  geometry_msgs::TransformStamped pcFrameTf;
  static Eigen::Affine3d lastCamPose;

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

  Eigen::Affine3d camPoseEigen = tf2::transformToEigen(pcFrameTf.transform);

  if (camPoseEigen.isApprox(lastCamPose, 1e-2))
  {
      ROS_INFO("Arm has not moved, not processing pointcloud");
      return;
  }

  lastCamPose = camPoseEigen;

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
  testTree.insertPointCloud(pc, scan_orig);
  tree_mtx.unlock();

  ros::Time insertTime = ros::Time::now();

  ROS_INFO_STREAM("Timings - TF: " << tfTime - cbStartTime << "; doTF: " << doTFTime - tfTime << "; toOct: " << toOctoTime - doTFTime << "; insert: " << insertTime - toOctoTime);
}

void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::unordered_set<size_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
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
void pointCloud2ToOctomapByIndices(const sensor_msgs::PointCloud2 &cloud, const std::vector<uint32_t> &indices,  octomap::Pointcloud &inlierCloud, octomap::Pointcloud &outlierCloud)
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
       if (i == *it)
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

void registerRoiPCL(const pointcloud_roi_msgs::PointcloudWithRoi &roi)
{
  octomap::Pointcloud inlierCloud, outlierCloud;
  pointCloud2ToOctomapByIndices(roi.cloud, roi.roi_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();

  //std::vector<octomap::OcTreeKey> roi_keys = testTree.getRoiKeys();
  //ROS_INFO_STREAM("Found " << testTree.getRoiSize() << " ROI keys (" << testTree.getAddedRoiSize() << " added, " << testTree.getDeletedRoiSize() << " removed)");
}

void registerRoi(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const instance_segmentation_msgs::DetectionsConstPtr &dets_msg)
{
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
  pointCloud2ToOctomapByIndices(*pc_msg, inlier_indices, inlierCloud, outlierCloud);
  //ROS_INFO_STREAM("Cloud sizes: " << inlierCloud.size() << ",  " << outlierCloud.size());

  tree_mtx.lock();
  testTree.insertRegionScan(inlierCloud, outlierCloud);
  tree_mtx.unlock();
}

void testRayTrace(const octomap::point3d &orig, const octomap::point3d &end)
{
  octomap::KeyRay ray;
  testTree.computeRayKeys(orig, end, ray);

  for (const octomap::OcTreeKey &key : ray)
  {
    octomap::OcTreeNode *node = testTree.search(key);
    if (node != NULL)
    {
      double occ = node->getOccupancy();
    }

  }
}

octomap::point3d sampleRandomPointOnSphere(const octomap::point3d &center, double radius)
{
  static std::default_random_engine generator;
  static std::normal_distribution<double> distribution(0.0, 1.0);
  octomap::point3d p;
  for (size_t i = 0; i < 3; i++)
    p(i) = distribution(generator);

  p.normalize();
  p *= radius;
  p += center;
  return p;
}

/*void getBorderPoints(const octomap::point3d &orig, double maxDist)
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

struct Viewpoint
{
  octomap::point3d point;
  octomap::point3d target;
  tf2::Quaternion orientation;
  double infoGain;
  double distance;
  double utility;
  bool isFree;
};

std::vector<Viewpoint> sampleAroundMultiROICenters(const std::vector<octomap::point3d> &centers, const double &dist, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);

  std::vector<Viewpoint> sampledPoints;
  if (centers.empty()) return sampledPoints;
  ros::Time samplingStartTime = ros::Time::now();
  const size_t TOTAL_SAMPLE_TRIES = 500;
  size_t samples_per_center = TOTAL_SAMPLE_TRIES / centers.size();
  for (const octomap::point3d &center : centers)
  {
    for (size_t i = 0; i < samples_per_center; i++)
    {
      octomap::KeyRay ray;
      octomap::point3d spherePoint = sampleRandomPointOnSphere(center, dist);
      testTree.computeRayKeys(center, spherePoint, ray);
      bool view_occluded = false;
      bool has_left_roi = false;
      bool last_node_free = false;
      size_t unknown_nodes = 0;
      for (const octomap::OcTreeKey &key : ray)
      {
        octomap_vpp::RoiOcTreeNode *node = testTree.search(key);
        if (node == NULL)
        {
          unknown_nodes++;
          has_left_roi = true;
          last_node_free = false;
          continue;
        }
        if (!has_left_roi && testTree.isNodeROI(node))
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
      if (!view_occluded)
      {
        octomath::Vector3 dirVec = center - spherePoint;
        dirVec.normalize();
        tf2::Vector3 dirVecTf = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
        tf2::Vector3 rotAx = viewDir.cross(dirVecTf);
        double rotAng = viewDir.angle(dirVecTf);
        tf2::Quaternion toRot(rotAx, rotAng);
        tf2::Quaternion viewQuat = toRot * camQuat;

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

void publishViewpointVisualizations(const std::vector<Viewpoint> &viewpoints)
{
  visualization_msgs::MarkerArray markers;
  geometry_msgs::PoseArray poseArr;
  poseArr.header.frame_id = "world";
  poseArr.header.stamp = ros::Time::now();
  for (size_t i = 0; i < viewpoints.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "roiPoints";
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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(2);
    marker.points.push_back(octomap::pointOctomapToMsg(viewpoints[i].point));
    marker.points.push_back(octomap::pointOctomapToMsg(viewpoints[i].target));
    markers.markers.push_back(marker);

    visualization_msgs::Marker textMarker;
    textMarker.header.frame_id = "world";
    textMarker.header.stamp = ros::Time();
    textMarker.ns = "roiPoints_texts";
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
    textMarker.color.a = 1.0;
    textMarker.color.r = 1.0;
    textMarker.color.g = 0.0;
    textMarker.color.b = 0.0;
    textMarker.lifetime = ros::Duration(2);
    textMarker.text = std::to_string(viewpoints[i].infoGain) + ", " + std::to_string(viewpoints[i].distance) + ", " + std::to_string(viewpoints[i].isFree);
    markers.markers.push_back(textMarker);

    geometry_msgs::Pose pose;
    pose.position = octomap::pointOctomapToMsg(viewpoints[i].point);
    pose.orientation = tf2::toMsg(viewpoints[i].orientation);
    poseArr.poses.push_back(pose);
  }
  viewArrowVisPub.publish(markers);
  poseArrayPub.publish(poseArr);
}

void sampleAroundROICenter(const octomap::point3d &center, const double &dist,  const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t roiID = 0)
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
    octomap::point3d spherePoint = sampleRandomPointOnSphere(center, dist);
    testTree.computeRayKeys(center, spherePoint, ray);
    bool view_occluded = false;
    bool has_left_roi = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = testTree.search(key);
      if (node == NULL)
      {
        unknown_nodes++;
        has_left_roi = true;
        last_node_free = false;
        continue;
      }
      if (!has_left_roi && testTree.isNodeROI(node))
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
      tf2::Vector3 dirVecTf = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
      sampledPoints.push_back(spherePoint);
      infoGain.push_back(unknown_nodes);
      distances.push_back((spherePoint - camPos).norm());
      vpIsFree.push_back(last_node_free);

      tf2::Vector3 rotAx = viewDir.cross(dirVecTf);
      double rotAng = viewDir.angle(dirVecTf);
      tf2::Quaternion toRot(rotAx, rotAng);
      tf2::Quaternion viewQuat = toRot * camQuat;
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

bool hasDirectUnkownNeighbour(const octomap::OcTreeKey &key, unsigned int depth = 0)
{
  for (int i = 0; i < 6; i++)
  {
    octomap::OcTreeKey neighbour_key(key);
    neighbour_key[i/2] += i%2 ? 1 : -1;
    octomap_vpp::RoiOcTreeNode *node = testTree.search(neighbour_key, depth);
    if (node == NULL || node->getLogOdds() == 0) return true;
  }
  return false;
}

void getBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, unsigned int depth = 0)
{
  assert(depth <= testTree.getTreeDepth());
  if (depth == 0)
    depth = testTree.getTreeDepth();

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

  for (auto it = testTree.begin_leafs_bbx(pmin, pmax, depth), end = testTree.end_leafs_bbx(); it != end; it++)
  {
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasDirectUnkownNeighbour(it.getKey(), depth))
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

octomap::point3d computeUnknownDir(const octomap::OcTreeKey &key)
{
  octomap::point3d averageUnknownDir;
  for (size_t i = 0; i < 18; i++)
  {
    octomap::OcTreeKey neighbour_key(key[0] + octomap_vpp::nb18Lut[i][0], key[1] + octomap_vpp::nb18Lut[i][1], key[2] + octomap_vpp::nb18Lut[i][2]);
    octomap_vpp::RoiOcTreeNode *node = testTree.search(neighbour_key);
    if (node == NULL || node->getLogOdds() == 0)
    {
      averageUnknownDir += octomap::point3d(octomap_vpp::nb18Lut[i][0], octomap_vpp::nb18Lut[i][1], octomap_vpp::nb18Lut[i][2]);
    }
  }
  averageUnknownDir.normalize();
  return averageUnknownDir;
}

std::vector<Viewpoint> getBorderPoints(const octomap::point3d &pmin, const octomap::point3d &pmax, const octomap::point3d &camPos, const tf2::Quaternion &camQuat)
{
  tf2::Matrix3x3 camMat(camQuat);
  tf2::Vector3 viewDir = camMat.getColumn(0);
  std::vector<Viewpoint> sampledPoints;

  tree_mtx.lock();

  for (auto it = testTree.begin_leafs_bbx(pmin, pmax), end = testTree.end_leafs_bbx(); it != end; it++)
  {
    if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
    {
      if (hasDirectUnkownNeighbour(it.getKey()))
      {
        Viewpoint vp;
        vp.point = it.getCoordinate();
        octomath::Vector3 dirVec = computeUnknownDir(it.getKey());
        vp.target = vp.point + dirVec;
        tf2::Vector3 dirVecTf = tf2::Vector3(dirVec.x(), dirVec.y(), dirVec.z());
        tf2::Vector3 rotAx = viewDir.cross(dirVecTf);
        double rotAng = viewDir.angle(dirVecTf);
        tf2::Quaternion toRot(rotAx, rotAng);
        vp.orientation = toRot * camQuat;
        sampledPoints.push_back(vp);// add node to border list
      }
    }
  }

  tree_mtx.unlock();
  return sampledPoints;
}



robot_state::RobotStatePtr sampleNextRobotState(const robot_state::JointModelGroup *joint_model_group, const robot_state::RobotState &current_state)
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
    double curValue = testTree.computeViewpointValue(viewpoint, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    tree_mtx.unlock();
    if (curValue > maxValue)
    {
      *maxState = *curSample;
      maxValue = curValue;
    }
  }
  return maxState;
}

bool moveToPose(moveit::planning_interface::MoveGroupInterface &manipulator_group, const geometry_msgs::Pose &goal_pose)
{
  manipulator_group.setPoseReferenceFrame("world");
  manipulator_group.setPlannerId("TRRTkConfigDefault");
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
    manipulator_group.execute(plan);
  }
  else
  {
    ROS_INFO("Could not find plan");
  }
  return success;
}

bool moveToState(moveit::planning_interface::MoveGroupInterface &manipulator_group, const robot_state::RobotState &goal_state)
{
  manipulator_group.setJointValueTarget(goal_state);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (manipulator_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    manipulator_group.asyncExecute(plan);
  }
  else
  {
    ROS_INFO("Can't execute plan");
  }
  return success;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tf2_ros::TransformListener tfListener(tfBuffer);

  octomapPub = nh.advertise<octomap_msgs::Octomap>("octomap", 1);
  inflatedOctomapPub = nh.advertise<octomap_msgs::Octomap>("inflated_octomap", 1);
  //pcGlobalPub = nh.advertise<sensor_msgs::PointCloud2>(PC_GLOBAL, 1);
  //planningScenePub = nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
  pointVisPub = nh.advertise<visualization_msgs::Marker>("border_marker", 1);
  viewArrowVisPub = nh.advertise<visualization_msgs::MarkerArray>("roi_vp_marker", 1);
  poseArrayPub = nh.advertise<geometry_msgs::PoseArray>("vp_array", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> depthCloudSub(nh, PC_TOPIC, 1);
  //tf2_ros::MessageFilter<sensor_msgs::PointCloud2> tfCloudFilter(depthCloudSub, tfBuffer, MAP_FRAME, 1, nh);
  //message_filters::Cache<sensor_msgs::PointCloud2> cloudCache(tfCloudFilter, 1);

  depthCloudSub.registerCallback(registerNewScan);
  //tfCloudFilter.registerCallback(registerNewScan);
  //cloudCache.registerCallback(registerNewScan);

  //message_filters::Subscriber<sensor_msgs::PointCloud2> pcGlobalSub(nh, PC_GLOBAL, 1);
  //message_filters::Subscriber<instance_segmentation_msgs::Detections> detectionsSub(nh, "/mask_rcnn/detections", 1);

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, instance_segmentation_msgs::Detections> DetsSyncPolicy;
  //message_filters::Synchronizer<DetsSyncPolicy> syncDets(DetsSyncPolicy(50), pcGlobalSub, detectionsSub);

  //syncDets.registerCallback(registerRoi);

  ros::Subscriber roiSub = nh.subscribe("/pointcloud_roi", 1, registerRoiPCL);

  moveit::planning_interface::MoveGroupInterface manipulator_group("manipulator");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    /*std::vector<double> mg_joint_values = manipulator_group.getCurrentJointValues();
    kinematic_state->setJointGroupPositions(joint_model_group, mg_joint_values);
    kinematic_state->setToRandomPositionsNearBy(joint_model_group, *kinematic_state, 0.2);
    const Eigen::Affine3d& stateTf = kinematic_state->getGlobalLinkTransform("camera_link");
    auto translation = stateTf.translation();
    auto quaternion = Eigen::Quaterniond(stateTf.linear()).coeffs();
    octomap::pose6d viewpoint(octomap::point3d(translation[0], translation[1], translation[2]), octomath::Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));*/

    publishMap();

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

    const geometry_msgs::Vector3 &camOrig = camFrameTf.transform.translation;
    octomap::point3d box_min(camOrig.x - 0.2, camOrig.y - 0.2, camOrig.z - 0.2);
    octomap::point3d box_max(camOrig.x + 0.2, camOrig.y + 0.2, camOrig.z + 0.2);
    //getBorderPoints(box_min, box_max);

    tf2::Quaternion camQuat;
    tf2::fromMsg(camFrameTf.transform.rotation, camQuat);

    //std::vector<Viewpoint> borderVps = getBorderPoints(box_min, box_max, octomap::point3d(camOrig.x, camOrig.y, camOrig.z), camQuat);
    //publishViewpointVisualizations(borderVps);

    std::vector<octomap::point3d> clusterCenters = testTree.getClusterCenters();
    ROS_INFO_STREAM("ROI count: " << testTree.getRoiSize() << "; Clusters: " << clusterCenters.size());
    /*for (size_t i = 0; i < clusterCenters.size(); i++)
    {
      ROS_INFO_STREAM("Cluster center: " << clusterCenters[i]);
      sampleAroundROICenter(clusterCenters[i], 0.5, octomap::point3d(camOrig.x, camOrig.y, camOrig.z), camQuat, i);
    }*/
    std::vector<Viewpoint> nextViewpoints = sampleAroundMultiROICenters(clusterCenters, 0.5, octomap::point3d(camOrig.x, camOrig.y, camOrig.z), camQuat);
    publishViewpointVisualizations(nextViewpoints);

    auto vpComp = [](const Viewpoint &a, const Viewpoint &b)
    {
      //return a.distance > b.distance;
      return a.utility < b.utility;
    };
    for (std::make_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp); !nextViewpoints.empty(); std::pop_heap(nextViewpoints.begin(), nextViewpoints.end(), vpComp), nextViewpoints.pop_back())
    {
      const Viewpoint &vp = nextViewpoints.front();
      if (vp.infoGain > 0)
      {
        geometry_msgs::Pose pose;
        pose.position = octomap::pointOctomapToMsg(vp.point);
        pose.orientation = tf2::toMsg(vp.orientation);
        if (moveToPose(manipulator_group, pose))
        {
          break;
        }
      }
    }
  }
}
