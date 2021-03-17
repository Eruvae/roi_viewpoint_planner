#include "planner_interfaces/direct_planner_interface.h"
#include "viewpoint_planner.h"

namespace roi_viewpoint_planner
{

DirectPlannerInterface::DirectPlannerInterface(roi_viewpoint_planner::ViewpointPlanner *planner) : planner(planner)
{
}

boost::mutex& DirectPlannerInterface::getTreeMutex()
{
  return planner->tree_mtx;;
}

std::shared_ptr<octomap_vpp::RoiOcTree> DirectPlannerInterface::getPlanningTree()
{
  return planner->planningTree;
}

}
