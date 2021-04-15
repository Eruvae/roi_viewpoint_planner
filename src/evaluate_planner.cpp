#include <ros/ros.h>
#include "evaluator.h"
#include "planner_interfaces/external_planner_interface.h"

using namespace roi_viewpoint_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Read ROS parameters
  double tree_resolution;
  if (!nh.param<double>("/roi_viewpoint_planner/tree_resolution", tree_resolution, 0.01))
  {
    ROS_WARN("Planning tree resolution not found, using 0.01 as default");
  }
  const std::vector<std::string> mode_list = {"idle", "map_only", "automatic", "roi_contours", "roi_centers", "roi_adjacent", "exploration", "contours", "border"};
  const std::string planning_mode_str = nhp.param<std::string>("planning_mode", "automatic");
  auto mode_it = std::find(mode_list.begin(), mode_list.end(), planning_mode_str);
  if (mode_it == mode_list.end())
  {
    ROS_ERROR("Specified planning mode not recognized");
    return false;
  }
  int planning_mode = mode_it - mode_list.begin();

  std::shared_ptr<ExternalPlannerInterface> planner(new ExternalPlannerInterface(nh, planning_mode, tree_resolution));
  Evaluator evaluator(planner, nh, nhp, true);

  for (int i = 0; ros::ok() && i < 20; i++)
  {
    const std::string resultsFileName = "planner_results_" + std::to_string(i) + ".csv";
    const std::string singleFruitResultsName = "results_single_fruits_" + std::to_string(i) + ".csv";
    std::ofstream resultsFile(resultsFileName);
    std::ofstream singleFruitResultsFile(singleFruitResultsName);
    evaluator.writeHeader(resultsFile);

    bool planner_active = false;
    for(ros::Rate rate(1); ros::ok() && !planner_active; rate.sleep())
    {
      planner_active = planner->activatePlanner();
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

      EvaluationParameters res = evaluator.processDetectedRois();

      double passed_time = (currentTime - plannerStartTime).toSec();

      evaluator.writeParams(resultsFile, passed_time, res);

      singleFruitResultsFile << passed_time << ",";
      for (size_t i = 0; i < res.fruit_cell_percentages.size(); i++)
      {
        singleFruitResultsFile << res.fruit_cell_percentages[i];
        if (i < res.fruit_cell_percentages.size() - 1)
          singleFruitResultsFile << ",";
        else
          singleFruitResultsFile << std::endl;
      }

      if (passed_time > PLANNING_TIME) // PLANNING_TIME s timeout
      {
        for(ros::Rate rate(1); ros::ok() && planner_active; rate.sleep())
        {
          planner_active = !planner->stopPlanner();
          if (!planner_active)
            ROS_INFO("Planner stopped");
          else
            ROS_INFO("Planner could not be stopped, retrying in 1 second...");
        }
        break;
      }
    }

    resultsFile.close();
    singleFruitResultsFile.close();

    bool octree_saved = false;
    for(ros::Rate rate(1); ros::ok() && !octree_saved; rate.sleep())
    {
      octree_saved = planner->saveOctree("result_tree_" + std::to_string(i));
      if (octree_saved)
        ROS_INFO("Saving octree successful");
      else
        ROS_INFO("Octree could not be saved, retrying in 1 second...");
    }

    bool planner_reset = false;
    for(ros::Rate rate(1); ros::ok() && !planner_reset; rate.sleep())
    {
      planner_reset = planner->resetPlanner();
      if (planner_reset)
        ROS_INFO("Planner reset successful");
      else
        ROS_INFO("Planner could not be resetted, retrying in 1 second...");
    }

    planner->clearOctree();
  }
}
