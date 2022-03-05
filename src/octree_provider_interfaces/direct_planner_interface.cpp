#include "roi_viewpoint_planner/octree_provider_interfaces/direct_planner_interface.h"
#include "roi_viewpoint_planner/viewpoint_planner.h"

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

double DirectPlannerInterface::getTreeResolution()
{
  return planner->planningTree->getResolution();
}

}
