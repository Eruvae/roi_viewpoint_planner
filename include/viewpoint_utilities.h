#ifndef VIEWPOINT_UTILITIES_H
#define VIEWPOINT_UTILITIES_H

#include "rvp_types.h"
#include <tf2/LinearMath/Quaternion.h>

namespace roi_viewpoint_planner
{

class ViewpointPlanner;
class SamplerBase;

class UtilityBase
{
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
    RoiVicinityUtility(SamplerBase *sampler, ViewpointPlanner *planner) : UtilityBase(sampler, planner) {}

    virtual bool computeUtility(Viewpoint &vp, const octomap::point3d &origin, const octomap::point3d &target, const tf2::Quaternion &viewQuat) override;
};

class RoiOcclusionUtility : public UtilityBase
{
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
    }
    return nullptr;
}

} // namespace roi_viewpoint_planner

#endif // VIEWPOINT_UTILITIES_H
