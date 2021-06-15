#include "roi_viewpoint_planner/viewpoint_utilities.h"
#include "roi_viewpoint_planner/viewpoint_samplers.h"
#include "roi_viewpoint_planner/viewpoint_planner.h"

namespace roi_viewpoint_planner
{

UtilityBase::UtilityBase(SamplerBase *sampler, ViewpointPlanner *planner) : sampler(sampler), planner(planner)
{}

UtilityBase::~UtilityBase()
{}

double UtilityBase::computeExpectedRayIGinSamplingTree(const octomap::KeyRay &ray)
{
  double expected_gain = 0;
  //double curProb = 1;
  for (const octomap::OcTreeKey &key : ray)
  {
    octomap_vpp::RoiOcTreeNode *node = planner->planningTree->search(key);
    double reachability = planner->samplingTree ? planner->samplingTree->getReachability(planner->transformToWorkspace(planner->planningTree->keyToCoord(key))) : 1.0;
    if (reachability > 0) reachability = 1.0; // Test: binarize reachability
    if (node == nullptr)
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
  }
  if (ray.size() > 0)
    expected_gain /= ray.size();

  return expected_gain;
}

bool SingleRayUtility::computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion&)
{
    octomap::KeyRay ray;
    planner->planningTree->computeRayKeys(target, origin, ray);
    bool view_occluded = false;
    bool last_node_free = false;
    size_t unknown_nodes = 0;
    for (const octomap::OcTreeKey &key : ray)
    {
      octomap_vpp::RoiOcTreeNode *node = planner->planningTree->search(key);
      if (node == nullptr)
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

      vp.infoGain = static_cast<double>(unknown_nodes) / static_cast<double>(ray.size());
      vp.distance = (origin - sampler->camPos).norm();
      vp.utility = vp.infoGain - 0.2 * vp.distance;
      vp.isFree = last_node_free;
    }
    return !view_occluded;
}

bool MultiRayUtility::computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d&, const tf2::Quaternion &viewQuat)
{
    octomap::pose6d viewpose(origin, octomath::Quaternion(
                                 static_cast<float>(viewQuat.w()),
                                 static_cast<float>(viewQuat.x()),
                                 static_cast<float>(viewQuat.y()),
                                 static_cast<float>(viewQuat.z())));
    const double hfov = 80 * M_PI / 180.0;
    const size_t x_steps = 8;
    const size_t y_steps = 6;
    const double maxRange = 5.0;

    double f_rec =  2 * tan(hfov / 2) / static_cast<double>(x_steps);
    double cx = static_cast<double>(x_steps) / 2.0;
    double cy = static_cast<double>(y_steps) / 2.0;
    double value = 0;
    for (size_t i = 0; i < x_steps; i++)
    {
      for(size_t j = 0; j < y_steps; j++)
      {
        double x = (i + 0.5 - cx) * f_rec;
        double y = (j + 0.5 - cy) * f_rec;
        octomap::point3d dir(x, y, 1.0);
        octomap::point3d end = dir * maxRange;
        end = viewpose.transform(end);
        octomap::KeyRay ray;
        planner->planningTree->computeRayKeys(viewpose.trans(), end, ray);
        double gain = planner->computeExpectedRayIGinSamplingTree(ray);
        //ROS_INFO_STREAM("Ray (" << i << ", " << j << ") from " << viewpoint.trans() << " to " << end << ": " << ray.size() << " Keys, Gain: " << gain);
        value += gain;
      }
    }
    value /= x_steps * y_steps;
    vp.infoGain = value;
    vp.distance = (origin - sampler->camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    return true;
}

RoiVicinityUtility::RoiVicinityUtility(SamplerBase *sampler, ViewpointPlanner *planner) : UtilityBase(sampler, planner)
{
    ROS_INFO_STREAM("Inflating ROIs for utility...");
    ros::Time inflationBegin = ros::Time::now();
    inflated_roi_tree = planner->planningTree->computeInflatedRois(0.05, 0.3);
    ros::Time inflationEnd = ros::Time::now();
    ROS_INFO_STREAM("Inflation time: " << (inflationEnd - inflationBegin));
}

double RoiVicinityUtility::computeRoiWeightedIG(const octomap::KeyRay &ray)
{
    double ray_val = 0.0;
    for (const octomap::OcTreeKey &key : ray)
    {
      const octomap::OcTreeKey inf_key = inflated_roi_tree->coordToKey(planner->planningTree->keyToCoord(key));
      octomap_vpp::RoiOcTreeNode *node = planner->planningTree->search(key);
      octomap_vpp::InflatedRoiOcTreeNode *inf_node = inflated_roi_tree->search(inf_key);
      double weight = 0.5;
      if (inf_node != nullptr)
      {
          weight += static_cast<double>(inf_node->getValue()) * 0.5;
      }
      if (node == nullptr)
      {
        ray_val += weight;
        continue;
      }
      double occ = node->getOccupancy();
      if (occ > 0.6) // View is blocked
      {
        break;
      }
      else if (occ > 0.4) // unknown
      {
        ray_val += weight;
      }
      //else //free
    }
    ray_val /= ray.size();
    return ray_val;
}

bool RoiVicinityUtility::computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat)
{
    /*octomap::pose6d viewpose(origin, octomath::Quaternion(
                                 static_cast<float>(viewQuat.w()),
                                 static_cast<float>(viewQuat.x()),
                                 static_cast<float>(viewQuat.y()),
                                 static_cast<float>(viewQuat.z())));
    const double hfov = 80 * M_PI / 180.0;
    const size_t x_steps = 8;
    const size_t y_steps = 6;
    const double maxRange = 5.0;

    double f_rec =  2 * tan(hfov / 2) / static_cast<double>(x_steps);
    double cx = static_cast<double>(x_steps) / 2.0;
    double cy = static_cast<double>(y_steps) / 2.0;
    double value = 0;
    for (size_t i = 0; i < x_steps; i++)
    {
      for(size_t j = 0; j < y_steps; j++)
      {
        double x = (i + 0.5 - cx) * f_rec;
        double y = (j + 0.5 - cy) * f_rec;
        octomap::point3d dir(x, y, 1.0);
        octomap::point3d end = dir * maxRange;
        end = viewpose.transform(end);
        octomap::KeyRay ray;
        planner->planningTree->computeRayKeys(viewpose.trans(), end, ray);
        double gain = computeRoiWeightedIG(ray);
        //ROS_INFO_STREAM("Ray (" << i << ", " << j << ") from " << viewpoint.trans() << " to " << end << ": " << ray.size() << " Keys, Gain: " << gain);
        value += gain;
      }
    }
    value /= x_steps * y_steps;
    vp.infoGain = value;
    vp.distance = (origin - sampler->camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    return true;*/

    octomap::KeyRay ray;
    planner->planningTree->computeRayKeys(origin, target, ray);
    double gain = computeRoiWeightedIG(ray);
    vp.infoGain = gain;
    vp.distance = (origin - sampler->camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    return true;
}

bool RoiOcclusionUtility::computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat)
{
    octomap::pose6d viewpose(origin, octomath::Quaternion(
                                 static_cast<float>(viewQuat.w()),
                                 static_cast<float>(viewQuat.x()),
                                 static_cast<float>(viewQuat.y()),
                                 static_cast<float>(viewQuat.z())));

    const double ANGLE_STEP = 15.0 * 180.0/M_PI;
    tf2::Quaternion dRoll, dPitch, dYaw;
    dRoll.setRPY(ANGLE_STEP, 0, 0);
    dPitch.setRPY(0, ANGLE_STEP, 0);
    dYaw.setRPY(0, 0, ANGLE_STEP);

    double hfov = 80.0 * M_PI / 180.0;
    size_t x_steps = 8;
    size_t y_steps = 6;
    float maxRange = 5.0;

    double f_rec =  2 * tan(hfov / 2) / static_cast<double>(x_steps);
    double cx = static_cast<double>(x_steps) / 2.0;
    double cy = static_cast<double>(y_steps) / 2.0;
    double value = 0;
    for (size_t i = 0; i < x_steps; i++)
    {
      for(size_t j = 0; j < y_steps; j++)
      {
        float x = static_cast<float>((i + 0.5 - cx) * f_rec);
        float y = static_cast<float>((j + 0.5 - cy) * f_rec);
        octomap::point3d dir(x, y, 1.0);
        octomap::point3d end = dir * maxRange;
        end = viewpose.transform(end);
        octomap::KeyRay ray;
        planner->planningTree->computeRayKeys(viewpose.trans(), end, ray);
        double gain = planner->computeExpectedRayIGinSamplingTree(ray);
        //ROS_INFO_STREAM("Ray (" << i << ", " << j << ") from " << viewpoint.trans() << " to " << end << ": " << ray.size() << " Keys, Gain: " << gain);
        value += gain;
      }
    }
    value /= x_steps * y_steps;
    return true;
}

} // namespace roi_viewpoint_planner
