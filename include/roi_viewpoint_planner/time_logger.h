#ifndef TIME_LOGGER_H
#define TIME_LOGGER_H

#include <ros/ros.h>
#include <fstream>
#include <array>

namespace roi_viewpoint_planner {

class TimeLogger
{
public:
  enum Event
  {
    MAP_PUBLISHED = 0,
    CAM_POS_COMPUTED = 1,
    MOVE_TO_SEE_APPLIED = 2,
    VIEWPOINTS_SAMPLED = 3,
    VIEWPOINTS_SELECTED = 4,
    PLAN_EXECUTED = 5,
    WAITED_FOR_SCAN = 6,
    EVALUATED = 7,
    SIZE = 8
  };

private:
  inline static const std::string header_line =
      "Map published," // 0
      "Cam pos computed," // 1
      "Move to see applied," // 2
      "Viewpoints sampled," // 3
      "Viewpoints selected," // 4
      "Plan executed," // 5
      "Waited for scan," // 6
      "Evaluated"; // 7

  std::ofstream out_file;
  std::string file_prefix;
  int file_index;
  bool loop_started;

  std::vector<std::string> line;

  ros::Time time_last;

public:
  TimeLogger(const std::string &file_prefix = "rvp_times_") : file_prefix(file_prefix), file_index(-2), loop_started(false)
  {
    initNewFile();
  }

  ~TimeLogger()
  {
    out_file.close();
  }

  void startLoop()
  {
    line = std::vector<std::string>(Event::SIZE);
    loop_started = true;
    time_last = ros::Time::now();
  }

  void saveTime(Event event)
  {
    if (!loop_started)
      return;

    ros::Time time_now = ros::Time::now();
    double time = (time_now - time_last).toSec();
    time_last = time_now;

    line[event] = std::to_string(time);
  }

  void saveInfo(Event event, const std::string &info)
  {
    if (!loop_started)
      return;

    line[event] = info;
  }

  void endLoop()
  {
    if (!loop_started)
      return;

    for (size_t i=0; i < Event::SIZE - 1; i++)
      out_file << line[i] << ",";

    out_file << line[Event::SIZE - 1] << std::endl;

    loop_started = false;
  }

  void initNewFile(bool set_index=false, int index=0)
  {
    if (out_file.is_open())
      out_file.close();

    loop_started = false;
    if (set_index)
      file_index = index;
    else
      file_index++;

    out_file.open(file_prefix + std::to_string(file_index) + ".csv");
    out_file << header_line << std::endl;
  }
};

} // namespace roi_viewpoint_planner

#endif // TIME_LOGGER_H
