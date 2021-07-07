#include "roi_viewpoint_planner/viewpoint_samplers.h"
#include "roi_viewpoint_planner/viewpoint_planner.h"

namespace roi_viewpoint_planner
{

SamplerBase::SamplerBase(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
    planner(planner), camPos(camPos), camQuat(camQuat), camMat(camQuat), viewDir(camMat.getColumn(0)), maxSamples(maxSamples)
{
    utility = createUtility(util_type, this, planner);
}

SamplerBase::~SamplerBase()
{
    delete utility;
}

std::vector<Viewpoint> RoiContourSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);
    std::vector<Viewpoint> sampledPoints;

    octomap::KeySet roi = planner->planningTree->getRoiKeys();
    octomap::KeySet freeNeighbours;
    for (const octomap::OcTreeKey &key : roi)
    {
      planner->planningTree->getNeighborsInState(key, freeNeighbours, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::FREE_NONROI, octomap_vpp::NB_6);
    }
    std::vector<octomap::point3d> selectedPoints;
    for (const octomap::OcTreeKey &key : freeNeighbours)
    {
      if (planner->planningTree->hasNeighborInState(key, octomap_vpp::NodeProperty::OCCUPANCY, octomap_vpp::NodeState::UNKNOWN, octomap_vpp::NB_6))
      {
        selectedPoints.push_back(planner->planningTree->keyToCoord(key));
      }
    }

    if (selectedPoints.size() == 0)
      return sampledPoints;

    std::uniform_int_distribution<size_t> distribution(0, selectedPoints.size() - 1);
    for (size_t i = 0; i < maxSamples; i++)
    {
      Viewpoint vp;
      octomap::point3d target = selectedPoints[distribution(planner->random_engine)];
      octomap::point3d origin = planner->sampleRandomPointOnSphere(target, planner->sampleRandomSensorDistance());
      octomath::Vector3 dirVec = target - origin;
      dirVec.normalize();
      tf2::Quaternion viewQuat = planner->dirVecToQuat(dirVec, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(origin);
      vp.pose.orientation = tf2::toMsg(viewQuat);
      vp.target = target;

      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(origin)) == nullptr) // workspace specified and sampled point not in workspace
      {
        continue;
      }

      if (planner->compute_ik_when_sampling)
      {
        if (!planner->manipulator_group.setJointValueTarget(planner->transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(planner->manipulator_group.getJointValueTarget()));
      #else
        planner->manipulator_group.getJointValueTarget(vp.joint_target);
      #endif

      }

      if (utility->computeUtility(vp, origin, target, viewQuat))
      {
        sampledPoints.push_back(vp);
      }
    }
    return sampledPoints;
}

std::vector<Viewpoint> RoiAdjacentSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);

    std::vector<Viewpoint> sampledPoints;

    tf2::Matrix3x3 camMat(camQuat);

    ros::Time inflationBegin = ros::Time::now();
    planner->planningTree->computeInflatedRois(planner->planningTree->getResolution(), 0.1);
    ros::Time inflationEnd = ros::Time::now();
    ROS_INFO_STREAM("Inflation time: " << (inflationEnd - inflationBegin));
    octomap::KeySet roi_keys = planner->planningTree->getRoiKeys();
    octomap::KeySet inflated_roi_keys = planner->planningTree->getInflatedRoiKeys();
    std::vector<octomap::point3d> inflated_contours;

    for (const octomap::OcTreeKey &key : inflated_roi_keys)
    {
      if (planner->samplingTree != nullptr && planner->samplingTree->search(planner->transformToWorkspace(planner->planningTree->keyToCoord(key))) == nullptr)
      {
        continue; // sampling tree specified and sampled point not in sampling tree
      }
      octomap_vpp::RoiOcTreeNode *node = planner->planningTree->search(key);
      if (node != nullptr && node->getLogOdds() < 0) // is node free; TODO: replace with bounds later
      {
        if (planner->hasUnknownAndOccupiedNeighbour6(key))
        {
          inflated_contours.push_back(planner->planningTree->keyToCoord(key));
        }
      }
    }

    ROS_INFO_STREAM("Roi keys: " << roi_keys.size() << ", Inflated: " << inflated_roi_keys.size() << ", Contours: " << inflated_contours.size());

    if (inflated_contours.size() == 0)
      return sampledPoints;

    std::uniform_int_distribution<size_t> distribution(0, inflated_contours.size() - 1);
    for (size_t i = 0; i < maxSamples; i++)
    {
      Viewpoint vp;
      octomap::point3d target = inflated_contours[distribution(planner->random_engine)];
      octomap::KeyRay ray;
      octomap::point3d origin = planner->sampleRandomPointOnSphere(target, planner->sampleRandomSensorDistance());
      octomath::Vector3 dirVec = target - origin;
      dirVec.normalize();
      tf2::Quaternion viewQuat = planner->dirVecToQuat(dirVec, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(origin);
      vp.pose.orientation = tf2::toMsg(viewQuat);
      vp.target = target;

      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(origin)) == nullptr)
      {
        continue; // workspace specified and sampled point not in workspace
      }
      if (planner->compute_ik_when_sampling)
      {
        if (!planner->manipulator_group.setJointValueTarget(planner->transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(planner->manipulator_group.getJointValueTarget()));
      #else
        planner->manipulator_group.getJointValueTarget(vp.joint_target);
      #endif
      }

      if (utility->computeUtility(vp, origin, target, viewQuat))
      {
        sampledPoints.push_back(vp);
      }
    }
    return sampledPoints;
}

std::vector<Viewpoint> RoiCenterSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);
    std::vector<Viewpoint> sampledPoints;
    return sampledPoints;
}

std::vector<Viewpoint> ExplorationSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);

    std::vector<Viewpoint> sampledPoints;

    octomap::point3d wsMin_tf = planner->transformToMapFrame(planner->wsMin), wsMax_tf = planner->transformToMapFrame(planner->wsMax);
    octomap::point3d stMin_tf = planner->transformToMapFrame(planner->stMin), stMax_tf = planner->transformToMapFrame(planner->stMax);

    for (unsigned int i = 0; i < 3; i++)
    {
      if (wsMin_tf(i) > wsMax_tf(i))
        std::swap(wsMin_tf(i), wsMax_tf(i));

      if (stMin_tf(i) > stMax_tf(i))
        std::swap(stMin_tf(i), stMax_tf(i));
    }

    std::uniform_real_distribution<float> wsxDist(wsMin_tf.x(), wsMax_tf.x());
    std::uniform_real_distribution<float> wsyDist(wsMin_tf.y(), wsMax_tf.y());
    std::uniform_real_distribution<float> wszDist(wsMin_tf.z(), wsMax_tf.z());

    std::uniform_real_distribution<float> stxDist(stMin_tf.x(), stMax_tf.x());
    std::uniform_real_distribution<float> styDist(stMin_tf.y(), stMax_tf.y());
    std::uniform_real_distribution<float> stzDist(stMin_tf.z(), stMax_tf.z());

    for (size_t i = 0; i < maxSamples; i++)
    {
      Viewpoint vp;
      octomap::point3d origin(wsxDist(planner->random_engine), wsyDist(planner->random_engine), wszDist(planner->random_engine));
      octomap::point3d target(stxDist(planner->random_engine), styDist(planner->random_engine), stzDist(planner->random_engine));
      octomath::Vector3 dirVec = target - origin;
      dirVec.normalize();
      tf2::Quaternion viewQuat = planner->dirVecToQuat(dirVec, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(origin);
      vp.pose.orientation = tf2::toMsg(viewQuat);

      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(origin)) == nullptr)
      {
        continue; // workspace specified and sampled point not in workspace
      }
      if (planner->compute_ik_when_sampling)
      {
        if (!planner->manipulator_group.setJointValueTarget(planner->transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(planner->manipulator_group.getJointValueTarget()));
      #else
        planner->manipulator_group.getJointValueTarget(vp.joint_target);
      #endif
      }

      vp.target = target;

      if (utility->computeUtility(vp, origin, target, viewQuat))
      {
        sampledPoints.push_back(vp);
      }

    }

    return sampledPoints;
}

std::vector<Viewpoint> ContourSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);

    std::vector<Viewpoint> sampledPoints;

    std::vector<octomap::OcTreeKey> contourKeys;
    octomap::point3d stMin_tf = planner->transformToMapFrame(planner->stMin), stMax_tf = planner->transformToMapFrame(planner->stMax);
    for (unsigned int i = 0; i < 3; i++)
    {
      if (stMin_tf(i) > stMax_tf(i))
        std::swap(stMin_tf(i), stMax_tf(i));
    }
    for (auto it = planner->planningTree->begin_leafs_bbx(stMin_tf, stMax_tf), end = planner->planningTree->end_leafs_bbx(); it != end; it++)
    {
      if (planner->samplingTree != nullptr && planner->samplingTree->search(planner->transformToWorkspace(it.getCoordinate())) == nullptr)
      {
        continue; // sampling tree specified and sampled point not in sampling tree
      }
      if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
      {
        if (planner->hasUnknownAndOccupiedNeighbour6(it.getKey()))
        {
          contourKeys.push_back(it.getKey());
        }
      }
    }

    if (contourKeys.empty())
    {
      ROS_INFO_STREAM("No occ-unknown border found");
      return sampledPoints;
    }

    std::vector<octomap::OcTreeKey> selected_keys;
    std::sample(contourKeys.begin(), contourKeys.end(), std::back_inserter(selected_keys),
           maxSamples, std::mt19937{std::random_device{}()});

    for (const octomap::OcTreeKey &key : selected_keys)
    {
      Viewpoint vp;
      octomath::Vector3 normalDir = planner->computeSurfaceNormalDir(key);
      octomap::point3d contourPoint = planner->planningTree->keyToCoord(key);
      octomap::point3d origin = contourPoint + normalDir * static_cast<float>(planner->sampleRandomSensorDistance());
      tf2::Quaternion viewQuat = planner->dirVecToQuat(-normalDir, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(origin);
      vp.pose.orientation = tf2::toMsg(viewQuat);
      vp.target = contourPoint;

      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(origin)) == nullptr)
      {
        continue; // workspace specified and sampled point not in workspace
      }
      if (planner->compute_ik_when_sampling)
      {
        if (!planner->manipulator_group.setJointValueTarget(planner->transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(planner->manipulator_group.getJointValueTarget()));
      #else
        planner->manipulator_group.getJointValueTarget(vp.joint_target);
      #endif
      }
      octomap_vpp::RoiOcTreeNode *node = planner->planningTree->search(origin);
      if (node != nullptr && node->getLogOdds() > 0) // Node is occupied
      {
        continue;
      }

      if (utility->computeUtility(vp, origin, contourPoint, viewQuat))
      {
        sampledPoints.push_back(vp);
      }
    }
    ROS_INFO_STREAM("Border point count: " << contourKeys.size());

    return sampledPoints;
}

std::vector<Viewpoint> BorderSampler::sampleViewpoints()
{
    boost::mutex::scoped_lock lock(planner->tree_mtx);

    std::vector<Viewpoint> sampledPoints;

    octomap::point3d box_min(camPos.x() - 0.5f, camPos.y() - 0.5f, camPos.z() - 0.5f);
    octomap::point3d box_max(camPos.x() + 0.5f, camPos.y() + 0.5f, camPos.z() + 0.5f);

    std::vector<octomap::OcTreeKey> viewpoint_candidates;

    for (auto it = planner->planningTree->begin_leafs_bbx(box_min, box_max), end = planner->planningTree->end_leafs_bbx(); it != end; it++)
    {
      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(it.getCoordinate())) == nullptr)
      {
        continue; // workspace specified and sampled point not in workspace
      }
      if (it->getLogOdds() < 0) // is node free; TODO: replace with bounds later
      {
        if (planner->hasDirectUnknownNeighbour(it.getKey()))
        {
          viewpoint_candidates.push_back(it.getKey());
        }
      }
    }

    if (viewpoint_candidates.empty())
    {
      ROS_INFO_STREAM("No occ-unknown border found");
      return sampledPoints;
    }

    std::vector<octomap::OcTreeKey> selected_viewpoints;
    std::sample(viewpoint_candidates.begin(), viewpoint_candidates.end(), std::back_inserter(selected_viewpoints),
           maxSamples, std::mt19937{std::random_device{}()});

    for (const octomap::OcTreeKey &key : selected_viewpoints)
    {
      Viewpoint vp;

      octomap::point3d origin = planner->planningTree->keyToCoord(key);
      octomath::Vector3 dirVec = planner->computeUnknownDir(key);
      tf2::Quaternion viewQuat = planner->dirVecToQuat(dirVec, camQuat, viewDir);
      vp.pose.position = octomap::pointOctomapToMsg(origin);
      vp.pose.orientation = tf2::toMsg(viewQuat);
      vp.target = origin + dirVec;

      if (planner->compute_ik_when_sampling)
      {
        if (!planner->manipulator_group.setJointValueTarget(planner->transformToWorkspace(vp.pose), "camera_link"))
          continue;

      #if ROS_VERSION_MAJOR == 1 && ROS_VERSION_MINOR <= 14 // ROS melodic or older
        vp.joint_target.reset(new robot_state::RobotState(planner->manipulator_group.getJointValueTarget()));
      #else
        planner->manipulator_group.getJointValueTarget(vp.joint_target);
      #endif
      }

      if (utility->computeUtility(vp, origin, vp.target, viewQuat))
      {
        sampledPoints.push_back(vp);
      }
    }

    return sampledPoints;
}

} // namespace roi_viewpoint_planner
