// C++ Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// pcl
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// pcl ROS
#include <pcl_conversions/pcl_conversions.h>

class NodeManager
{
  public:
    // Constructor
    NodeManager()
    {
      map_octree = new octomap::OcTree(0.1);
    }

    // Property Definitions
      // Callback booleans
      bool map_updated = false;
      bool position_updated = false;

      // Map
      octomap::OcTree* map_octree;
      int map_size = 0;

      // Position
      pcl::PointXYZ position;

    // Method Definitions
    void CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackOdometry(const nav_msgs::Odometry msg);
    int CalculateMapSize();
};

void NodeManager::CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  map_updated = true;
  return;
}

void NodeManager::CallbackOdometry(const nav_msgs::Odometry msg)
{
  position.x = msg.pose.pose.position.x;
  position.y = msg.pose.pose.position.y;
  position.z = msg.pose.pose.position.z;
  position_updated = true;
  return;
}

int NodeManager::CalculateMapSize()
{
  if (map_updated == true) {
    map_octree->expand();
    map_size = (int)map_octree->calcNumNodes();
    map_updated == false;
  }
  return map_size;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "monitor");
  ros::NodeHandle n;

  NodeManager monitor_node;

  // Declare subscribers for the camera info and the raw depth image.  Use the image truncator callbacks
  ros::Subscriber sub1 = n.subscribe("octomap_binary", 1, &NodeManager::CallbackOctomap, &monitor_node);
  ros::Subscriber sub2 = n.subscribe("odometry", 1, &NodeManager::CallbackOdometry, &monitor_node);

  // ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud2>("depth_cam_fixed", 5);

  // Get depth camera min and max range values
  int map_size_limit;
  n.param("monitor/map_size_limit", map_size_limit, (int)500);
  // n.param("project_truncated_depths/rMax", rMax, (double)6.5);

  // Declare and read in the node update rate from the launch file parameters
  double rate;
  n.param("monitor/rate", rate, (double)0.2); // Every 5 seconds
  ros::Rate r(rate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    if (monitor_node.CalculateMapSize() >= map_size_limit) putenv("STOP_TEST=1");
  }

  return 0;
}