#ifndef DIRECT_PLANNER_INTERFACE_H
#define DIRECT_PLANNER_INTERFACE_H

#include "rvp_evaluation/octree_provider_interface.h"

namespace roi_viewpoint_planner
{

class ViewpointPlanner;

class DirectPlannerInterface : public rvp_evaluation::OctreeProviderInterface
{
private:
  ViewpointPlanner *planner;
  rvp_evaluation::MutexRef<boost::mutex> tree_mtx;

public:
  DirectPlannerInterface(roi_viewpoint_planner::ViewpointPlanner *planner);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual rvp_evaluation::MutexBase& getTreeMutex();
};

}


#endif // DIRECT_PLANNER_INTERFACE_H
