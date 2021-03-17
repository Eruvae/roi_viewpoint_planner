#ifndef DIRECT_PLANNER_INTERFACE_H
#define DIRECT_PLANNER_INTERFACE_H

#include "planner_interface.h"

namespace roi_viewpoint_planner
{

class ViewpointPlanner;

class DirectPlannerInterface : public PlannerInterface
{
  ViewpointPlanner *planner;
public:
  DirectPlannerInterface(roi_viewpoint_planner::ViewpointPlanner *planner);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual boost::mutex& getTreeMutex();
};

}


#endif // DIRECT_PLANNER_INTERFACE_H
