#ifndef PLANNER_INTERFACE_H
#define PLANNER_INTERFACE_H

#include <octomap_vpp/RoiOcTree.h>
#include <boost/thread/mutex.hpp>

namespace roi_viewpoint_planner
{

class PlannerInterface
{
public:
  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree() = 0;
  virtual boost::mutex& getTreeMutex() = 0;
};

}

#endif // PLANNER_INTERFACE_H
