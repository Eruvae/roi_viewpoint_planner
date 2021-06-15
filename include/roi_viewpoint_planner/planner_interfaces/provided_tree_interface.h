#ifndef PROVIDED_TREE_INTERFACE_H
#define PROVIDED_TREE_INTERFACE_H

#include "roi_viewpoint_planner/planner_interface.h"

namespace roi_viewpoint_planner
{

class ProvidedTreeInterface : public PlannerInterface
{
private:
  std::shared_ptr<octomap_vpp::RoiOcTree> planningTree;
  boost::mutex &tree_mtx;
  boost::mutex own_mtx; // Used only if an external one isn't specified

public:
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree);
  ProvidedTreeInterface(std::shared_ptr<octomap_vpp::RoiOcTree> planningTree, boost::mutex &tree_mtx);

  virtual std::shared_ptr<octomap_vpp::RoiOcTree> getPlanningTree();
  virtual double getTreeResolution();
  virtual boost::mutex& getTreeMutex();
};

}


#endif // PROVIDED_TREE_INTERFACE_H
