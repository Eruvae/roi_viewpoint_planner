#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/client.h>
#include <roi_viewpoint_planner_msgs/PlannerConfig.h>
#include <octomap_vpp/RoiOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/dynamic_bitset.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <octomap_vpp/marching_cubes.h>
#include <octomap_vpp/octomap_pcl.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/Trigger.h>
#include <roi_viewpoint_planner_msgs/SaveOctomap.h>
#include "roi_viewpoint_planner_msgs/EvaluatorConfig.h"
#include "compute_cubes.h"
#include "point_cloud_color_handler_clusters.h"

#include "evaluator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  Evaluator evaluator(nh, nhp);

  for (int i = 0; ros::ok() && i < 10; i++)
  {
    const std::string resultsFileName = "planner_results_" + std::to_string(i) + ".csv";
    std::ofstream resultsFile(resultsFileName);
    resultsFile << "Time (s), Detected ROI cluster, Total ROI cluster, ROI percentage, Average distance, Average volume accuracy, Covered ROI volume, False ROI volume, ROI key count, True ROI keys, False ROI keys" << std::endl;

    bool planner_active = false;
    for(ros::Rate rate(1); ros::ok() && !planner_active; rate.sleep())
    {
      planner_active = evaluator.activatePlanner();
      if (planner_active)
        ROS_INFO("Planner activated");
      else
        ROS_INFO("Planner could not be activated, retrying in 1 second...");
    }

    const double PLANNING_TIME = 180;
    ros::Time plannerStartTime = ros::Time::now();

    for (ros::Rate rate(1); ros::ok(); rate.sleep())
    {
      ros::Time currentTime = ros::Time::now();

      const EvaluationParameters &res = evaluator.processDetectedRois();

      double passed_time = (currentTime - plannerStartTime).toSec();

      resultsFile << passed_time << ", " << res.detected_roi_clusters << ", " << res.total_roi_clusters << ", " << res.roi_percentage<< ", "
                  << res.average_distance << ", " << res.average_accuracy << ", " << res.covered_roi_volume << ", " << res.false_roi_volume << ", "
                  << res.roi_key_count << ", " << res.true_roi_key_count << ", " << res.false_roi_key_count << std::endl;

      if (passed_time > PLANNING_TIME) // PLANNING_TIME s timeout
      {
        for(ros::Rate rate(1); ros::ok() && planner_active; rate.sleep())
        {
          planner_active = !evaluator.stopPlanner();
          if (!planner_active)
            ROS_INFO("Planner stopped");
          else
            ROS_INFO("Planner could not be stopped, retrying in 1 second...");
        }
        break;
      }
    }

    resultsFile.close();

    bool octree_saved = false;
    for(ros::Rate rate(1); ros::ok() && !octree_saved; rate.sleep())
    {
      octree_saved = evaluator.saveOctree("result_tree_" + std::to_string(i));
      if (octree_saved)
        ROS_INFO("Saving octree successful");
      else
        ROS_INFO("Octree could not be saved, retrying in 1 second...");
    }

    bool planner_reset = false;
    for(ros::Rate rate(1); ros::ok() && !planner_reset; rate.sleep())
    {
      planner_reset = evaluator.resetPlanner();
      if (planner_reset)
        ROS_INFO("Planner reset successful");
      else
        ROS_INFO("Planner could not be resetted, retrying in 1 second...");
    }

    evaluator.clearOctree();
  }
}
