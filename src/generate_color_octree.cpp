#include <ros/ros.h>
#include <octomap/octomap_types.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_vpp/octomap_transforms.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <octomap/Pointcloud.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

std::string map_frame;
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;
ros::Publisher octomap_pub;

std::unique_ptr<octomap::ColorOcTree> color_tree;

struct Color
{
  uint8_t r, g, b;
};

void pointcloudCallback(const sensor_msgs::PointCloud2 &pc)
{
  geometry_msgs::TransformStamped pc_frame_tf;
  try
  {
    pc_frame_tf = tf_buffer->lookupTransform(map_frame, pc.header.frame_id, pc.header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame in registerRoiOCL: " << e.what());
    return;
  }

  octomap::pose6d t = octomap_vpp::transformToOctomath(pc_frame_tf.transform);
  octomap::point3d scan_orig = t.trans();

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(pc, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(pc, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(pc, "b");

  octomap::Pointcloud points;
  std::vector<Color> colors;

  for (size_t i = 0; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b, ++i)
  {
    if (std::isfinite(*iter_x) && std::isfinite(*iter_y) && std::isfinite(*iter_z))
    {
      octomap::point3d p(*iter_x, *iter_y, *iter_z);
      octomap::point3d p_tf = t.transform(p);
      points.push_back(p_tf);
      colors.push_back({*iter_r, *iter_g, *iter_b});
    }
  }

  color_tree->insertPointCloud(points, scan_orig);
  for (size_t i = 0; i < points.size(); i++)
  {
    const octomap::point3d &p = points[i];
    const Color &c = colors[i];
    color_tree->integrateNodeColor(p.x(), p.y(), p.z(), c.r, c.g, c.b);
  }

  octomap_msgs::Octomap map_msg;
  map_msg.header.frame_id = map_frame;
  map_msg.header.stamp = ros::Time::now();
  bool msg_generated = octomap_msgs::fullMapToMsg(*color_tree, map_msg);
  if (msg_generated)
  {
    octomap_pub.publish(map_msg);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_color_octree");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  map_frame = nhp.param<std::string>("map_frame", "world");
  const double tree_resolution = nhp.param<double>("tree_resolution", 0.01);
  const std::string input = nhp.param<std::string>("input", "/camera/aligned_depth_to_color/points");
  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, input, 1));
  ROS_INFO_STREAM("Subsribed to " << pc_sub->getTopic());
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, "world", 100, nhp));

  color_tree.reset(new octomap::ColorOcTree(tree_resolution));
  octomap_pub = nhp.advertise<octomap_msgs::Octomap>("color_tree", 1);
  transform_filter->registerCallback(pointcloudCallback);

  ros::spin();

  color_tree->write("color_octree.ot");
}
