#include <ros/ros.h>
#include "roi_viewpoint_planner/evaluator.h"
#include "roi_viewpoint_planner/planner_interfaces/external_planner_interface.h"

using namespace roi_viewpoint_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_planner_no_gt");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  double tree_resolution;
  if (!nh.param<double>("/roi_viewpoint_planner/tree_resolution", tree_resolution, 0.01))
  {
    ROS_WARN("Planning tree resolution not found, using 0.01 as default");
  }

  std::shared_ptr<ExternalPlannerInterface> planner(new ExternalPlannerInterface(nh, roi_viewpoint_planner::Planner_MAP_ONLY, tree_resolution));
  Evaluator evaluator(planner, nh, nhp, false);

  const std::string resultsFileName = "planner_results.csv";
  std::ofstream resultsFile(resultsFileName);
  resultsFile << "Time (s), Total ROI cluster, ROI key count" << std::endl;

  bool planner_active = false;
  for(ros::Rate rate(1); ros::ok() && !planner_active; rate.sleep())
  {
    planner_active = planner->activatePlanner();
    if (planner_active)
      ROS_INFO("Planner activated");
    else
      ROS_INFO("Planner could not be activated, retrying in 1 second...");
  }

  ros::Time plannerStartTime = ros::Time::now();

  for (ros::Rate rate(1); ros::ok(); rate.sleep())
  {
    ros::Time currentTime = ros::Time::now();

    const EvaluationParameters &res = evaluator.processDetectedRois();

    double passed_time = (currentTime - plannerStartTime).toSec();

    resultsFile << passed_time << ", " << res.roi_key_count << std::endl;
  }

  resultsFile.close();

}
