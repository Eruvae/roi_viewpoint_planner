#ifndef VIEWPOINT_SAMPLERS_H
#define VIEWPOINT_SAMPLERS_H

#include <octomap/octomap_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <random>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap_vpp/WorkspaceOcTree.h>

#include "rvp_types.h"
#include "viewpoint_utilities.h"

namespace roi_viewpoint_planner {

class ViewpointPlanner;

class SamplerBase
{
    friend class UtilityBase;
    friend class SingleRayUtility;
    friend class MultiRayUtility;
    friend class RoiVicinityUtility;
    friend class RoiOcclusionUtility;
public:
    virtual ~SamplerBase();

    virtual std::vector<Viewpoint> sampleViewpoints() = 0;

protected:
    ViewpointPlanner *planner;
    UtilityBase *utility;

    const octomap::point3d camPos;
    const tf2::Quaternion camQuat;
    const tf2::Matrix3x3 camMat;
    const tf2::Vector3 viewDir;
    const size_t maxSamples;

    /*static const std::default_random_engine random_engine;

    static octomap::point3d sampleRandomPointOnSphere(const octomap::point3d &center, float radius)
    {
      static std::normal_distribution<float> distribution(0.0, 1.0);
      octomap::point3d p;
      for (unsigned int i = 0; i < 3; i++)
        p(i) = distribution(random_engine);

      p.normalize();
      p *= radius;
      p += center;
      return p;
    }*/

    SamplerBase(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type);
};

class RoiContourSampler : public SamplerBase
{
public:
    RoiContourSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

class RoiAdjacentSampler : public SamplerBase
{
public:
    RoiAdjacentSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

class RoiCenterSampler : public SamplerBase
{
public:
    RoiCenterSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

class ExplorationSampler : public SamplerBase
{
public:
    ExplorationSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

class ContourSampler : public SamplerBase
{
public:
    ContourSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

class BorderSampler : public SamplerBase
{
public:
    BorderSampler(ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type) :
        SamplerBase(planner, camPos, camQuat, maxSamples, util_type)
    {}

    virtual std::vector<Viewpoint> sampleViewpoints() override;
};

static std::unique_ptr<SamplerBase> createSampler(SamplerType sampler_type, ViewpointPlanner *planner, const octomap::point3d &camPos, const tf2::Quaternion &camQuat, size_t maxSamples, UtilityType util_type)
{
    switch (sampler_type)
    {
    case SamplerType::ROI_CONTOUR_SAMPLER:
        return std::unique_ptr<SamplerBase>(new RoiContourSampler(planner, camPos, camQuat, maxSamples, util_type));
    case SamplerType::ROI_ADJACENT_SAMPLER:
        return std::unique_ptr<SamplerBase>(new RoiAdjacentSampler(planner, camPos, camQuat, maxSamples, util_type));
    case SamplerType::ROI_CENTER_SAMPLER:
        return std::unique_ptr<SamplerBase>(new RoiCenterSampler(planner, camPos, camQuat, maxSamples, util_type));
    case SamplerType::EXPLORATION_SAMPLER:
        return std::unique_ptr<SamplerBase>(new ExplorationSampler(planner, camPos, camQuat, maxSamples, util_type));
    case SamplerType::CONTOUR_SAMPLER:
        return std::unique_ptr<SamplerBase>(new ContourSampler(planner, camPos, camQuat, maxSamples, util_type));
    case SamplerType::BORDER_SAMPLER:
        return std::unique_ptr<SamplerBase>(new BorderSampler(planner, camPos, camQuat, maxSamples, util_type));
    }
    return nullptr;
}


}

#endif // VIEWPOINT_SAMPLERS_H
