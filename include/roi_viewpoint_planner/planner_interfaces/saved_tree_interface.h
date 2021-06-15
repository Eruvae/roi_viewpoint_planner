#ifndef SAVED_TREE_INTERFACE_H
#define SAVED_TREE_INTERFACE_H

#include "roi_viewpoint_planner/planner_interface.h"

namespace roi_viewpoint_planner
{

class SavedTreeInterface : public PlannerInterface
{
private:
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  boost::mutex tree_mtx;

public:
  SavedTreeInterface(double tree_resolution);
  bool readOctree(const std::string &filename);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual boost::mutex& getTreeMutex();
};

}

#endif // SAVED_TREE_INTERFACE_H
