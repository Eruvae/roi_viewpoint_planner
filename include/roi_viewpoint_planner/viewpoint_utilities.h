#ifndef VIEWPOINT_UTILITIES_H
#define VIEWPOINT_UTILITIES_H

#include "roi_viewpoint_planner/rvp_types.h"
#include <tf2/LinearMath/Quaternion.h>
#include <octomap/OcTreeKey.h>
#include <octomap_vpp/InflatedRoiOcTree.h>

namespace roi_viewpoint_planner
{

class ViewpointPlanner;
class SamplerBase;

class UtilityBase
{
protected:
    double computeExpectedRayIGinSamplingTree(const octomap::KeyRay &ray);

public:
    virtual ~UtilityBase();

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) = 0;

protected:
    SamplerBase *sampler;
    ViewpointPlanner *planner;

    UtilityBase(SamplerBase *sampler, ViewpointPlanner *planner);
};

class SingleRayUtility : public UtilityBase
{
public:
    SingleRayUtility(SamplerBase *sampler, ViewpointPlanner *planner) : UtilityBase(sampler, planner) {}

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) override;
};

class MultiRayUtility : public UtilityBase
{
public:
    MultiRayUtility(SamplerBase *sampler, ViewpointPlanner *planner) : UtilityBase(sampler, planner) {}

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) override;
};

class RoiVicinityUtility : public UtilityBase
{
private:
    std::shared_ptr<octomap_vpp::InflatedRoiOcTree> inflated_roi_tree;

    double computeRoiWeightedIG(const octomap::KeyRay &ray);
public:
    RoiVicinityUtility(SamplerBase *sampler, ViewpointPlanner *planner);

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) override;
};

class RoiOcclusionUtility : public UtilityBase
{
public:
    RoiOcclusionUtility(SamplerBase *sampler, ViewpointPlanner *planner) : UtilityBase(sampler, planner) {}

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) override;
};

static UtilityBase* createUtility(UtilityType util_type, SamplerBase *sampler, ViewpointPlanner *planner)
{
    switch (util_type)
    {
    case UtilityType::SINGLE_RAY_UTILITY:
        return new SingleRayUtility(sampler, planner);
    case UtilityType::MULTI_RAY_UTILITY:
        return new MultiRayUtility(sampler, planner);
    case UtilityType::ROI_VICINITY_UTILITY:
        return new RoiVicinityUtility(sampler, planner);
    case UtilityType::ROI_OCCLUSION_UTILITY:
        return new RoiOcclusionUtility(sampler, planner);
    }
    return nullptr;
}

} // namespace roi_viewpoint_planner

#endif // VIEWPOINT_UTILITIES_H
