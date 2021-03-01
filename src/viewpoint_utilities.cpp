#include "viewpoint_utilities.h"
#include "viewpoint_samplers.h"
#include "viewpoint_planner.h"

namespace roi_viewpoint_planner
{

UtilityBase::UtilityBase(SamplerBase *sampler, ViewpointPlanner *planner) : sampler(sampler), planner(planner)
{}

UtilityBase::~UtilityBase()
{}

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
    vp.infoGain = planner->computeViewpointSamplingValue(viewpose, 80.0 * M_PI / 180.0, 8, 6, 5.0);
    vp.distance = (origin - sampler->camPos).norm();
    vp.utility = vp.infoGain - 0.2 * vp.distance;
    vp.isFree = true;
    return true;
}

bool RoiVicinityUtility::computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat)
{
    octomap::point3d box_min(target.x() - 0.5f, target.y() - 0.5f, target.z() - 0.5f);
    octomap::point3d box_max(target.x() + 0.5f, target.y() + 0.5f, target.z() + 0.5f);

    size_t roi_nodes;

    for (auto it = planner->planningTree->begin_leafs_bbx(box_min, box_max), end = planner->planningTree->end_leafs_bbx(); it != end; it++)
    {
      if (planner->workspaceTree != nullptr && planner->workspaceTree->search(planner->transformToWorkspace(it.getCoordinate())) == nullptr)
      {
        continue; // workspace specified and sampled point not in workspace
      }
      if (planner->planningTree->isNodeROI(*it))
      {
          roi_nodes++;
      }
    }
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
}

} // namespace roi_viewpoint_planner
