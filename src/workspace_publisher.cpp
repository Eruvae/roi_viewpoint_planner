#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap_vpp/WorkspaceOcTree.h>
#include <octomap_vpp/CountingOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("roi_viewpoint_planner");

  ros::Publisher workspaceTreePub = nh.advertise<octomap_msgs::Octomap>("workspace_tree", 1, true);
  ros::Publisher samplingTreePub = nh.advertise<octomap_msgs::Octomap>("sampling_tree", 1, true);

  std::string wstree_default_package = ros::package::getPath("phenorob_ur5e");
  std::string wstree_file = nhp.param<std::string>("workspace_tree", wstree_default_package + "/workspace_trees/ur_retractable/workspace_map.ot");
  std::string sampling_tree_file = nhp.param<std::string>("sampling_tree", wstree_default_package + "/workspace_trees/ur_retractable/inflated_workspace_map.ot");
  std::string ws_frame = nhp.param<std::string>("ws_frame", "arm_base_link");

  // Load workspace

  octomap_vpp::WorkspaceOcTree *workspaceTree, *samplingTree;
  octomap_msgs::Octomap ws_msg, st_msg;

  octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(wstree_file);
  if (!tree)
  {
    ROS_ERROR_STREAM("Workspace tree file could not be loaded");
    return -1;
  }
  else
  {
    octomap_vpp::CountingOcTree *countingTree = dynamic_cast<octomap_vpp::CountingOcTree*>(tree);

    if (countingTree) // convert to workspace tree if counting tree loaded
    {
      workspaceTree = new octomap_vpp::WorkspaceOcTree(*countingTree);
      delete countingTree;
    }
    else
    {
      workspaceTree = dynamic_cast<octomap_vpp::WorkspaceOcTree*>(tree);
    }

    if (!workspaceTree)
    {
      ROS_ERROR("Workspace tree type not recognized; please load either CountingOcTree or WorkspaceOcTree");
      delete tree;
      return -1;
    }
    else
    {
      ws_msg.header.frame_id = ws_frame;
      ws_msg.header.stamp = ros::Time(0);
      bool msg_generated = octomap_msgs::fullMapToMsg(*workspaceTree, ws_msg);
      if (!msg_generated)
      {
        ROS_ERROR("Workspace message could not be generated");
        return -1;
      }
    }
  }

  tree = octomap::AbstractOcTree::read(sampling_tree_file);
  if (!tree)
  {
    ROS_ERROR_STREAM("Sampling tree file could not be loaded");
  }
  else
  {
    samplingTree = dynamic_cast<octomap_vpp::WorkspaceOcTree*>(tree);
    if (!samplingTree)
    {
      ROS_ERROR("Sampling tree must be of type WorkspaceOcTree");
      delete tree;
    }
  }

  if (!samplingTree) // if sampling tree not specified, use workspace octree
  {
    samplingTree = workspaceTree;
  }

  if (samplingTree)
  {
    st_msg.header.frame_id = ws_frame;
    st_msg.header.stamp = ros::Time(0);
    bool msg_generated = octomap_msgs::fullMapToMsg(*samplingTree, st_msg);
    if (!msg_generated)
    {
      ROS_ERROR("Workspace message could not be generated");
    }
  }

  workspaceTreePub.publish(ws_msg);
  samplingTreePub.publish(st_msg);
  ros::spin();

  /*for (ros::Rate r(1); r.sleep(); ros::ok())
  {
    //ros::Time t = ros::Time::now();
    //ws_msg.header.stamp = t;
    //st_msg.header.stamp = t;
    workspaceTreePub.publish(ws_msg);
    samplingTreePub.publish(st_msg);
  }*/
}
