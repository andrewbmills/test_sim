// C++ Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
// ROS libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

struct MapStats
{
  double voxel_size = 0.0;
  int free_count = 0;
  int occupied_count = 0;
  int unseen_count = 0;
};

MapStats GetMapStats(octomap::OcTree* map) {
  MapStats stats;
  stats.voxel_size = map->getResolution();
  // Expand pruned nodes in octree
  map->expand();
  // Iterate through expanded octree
  for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
       end=map->end_leafs(); it!=end; ++it) {
    if (it->getOccupancy() >= 0.6)
    {
      // Add to occupied count
      stats.occupied_count++;
    }
    else if (it->getOccupancy() <= 0.4)
    {
      // Add to free count
      stats.free_count++;
    }
    else
    {
      // Add to unseen count
      stats.unseen_count++;
    }
  }
  return stats;
}

class NodeManager
{
  public:
    // Constructor
    NodeManager()
    {
      map_octree = new octomap::OcTree(0.1);
    }

    // Property Definitions
      // Subscription booleans and last message times
      bool received_map = false;
      bool received_odometry = false;
      std::time_t last_map_time;
      std::time_t last_odometry_time;

      // Map
      octomap::OcTree* map_octree;
      std::vector<std::pair<double, MapStats>> maps_stats_history;

      // Pose History
      std::vector<std::pair<double, geometry_msgs::Pose>> pose_history;

    // Method Definitions
    void CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackOdometry(const nav_msgs::Odometry msg);
    bool WriteHistoryToCSV(std::string filename_map, std::string filename_poses);
};

void NodeManager::CallbackOctomap(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  maps_stats_history.push_back(std::make_pair(msg->header.stamp.toSec(), GetMapStats(map_octree)));
  last_map_time = std::time(NULL);
  return;
}

void NodeManager::CallbackOdometry(const nav_msgs::Odometry msg)
{
  if (received_odometry == false) received_odometry = true;
  pose_history.push_back(std::make_pair(msg.header.stamp.toSec(), msg.pose.pose));
  last_odometry_time = std::time(NULL);
  return;
}

bool NodeManager::WriteHistoryToCSV(std::string filename_poses, std::string filename_map)
{
  // Write pose data to file
  std::ofstream f;
  f.open(filename_poses, std::ios::out);
  if (f.is_open()) {
    for (int i=0; i<pose_history.size(); i++) {
      f << pose_history[i].first;
      f << ",";
      f << pose_history[i].second.position.x;
      f << ",";
      f << pose_history[i].second.position.y;
      f << ",";
      f << pose_history[i].second.position.z;
      f << ",";
      f << pose_history[i].second.orientation.w;
      f << ",";
      f << pose_history[i].second.orientation.x;
      f << ",";
      f << pose_history[i].second.orientation.y;
      f << ",";
      f << pose_history[i].second.orientation.z;
      if (i<(pose_history.size()-1)) f << "\n";
    }
  }
  else return false;
  f.close();

  // Write map data to file
  f.open(filename_map, std::ios::out);
  f << maps_stats_history[0].second.voxel_size;
  f << "\n";
  if (f.is_open()) {
    for (int i=0; i<maps_stats_history.size(); i++) {
      f << maps_stats_history[i].first;
      f << ",";
      f << maps_stats_history[i].second.free_count;
      f << ",";
      f << maps_stats_history[i].second.occupied_count;
      f << ",";
      f << maps_stats_history[i].second.unseen_count;
      if (i<(maps_stats_history.size()-1))f << "\n";
    }
  }
  else return false;
  f.close();
  return true;
}

int main(int argc, char **argv)
{
  // Initialize ROS node with name and object instantiation
  ros::init(argc, argv, "save_test_data");
  ros::NodeHandle n;

  NodeManager save_data_node;

  // Declare subscribers for the camera info and the raw depth image.  Use the image truncator callbacks
  ros::Subscriber sub1 = n.subscribe("octomap_full", 1, &NodeManager::CallbackOctomap, &save_data_node);
  ros::Subscriber sub2 = n.subscribe("odometry", 1, &NodeManager::CallbackOdometry, &save_data_node);
  // ros::Subscriber sub3 = n.subscribe("planned_path", 1, &NodeManager::CallbackPath, &save_data_node);
  // ros::Subscriber sub4 = n.subscribe("planning_time", 1, &NodeManager::CallbackPlanningTime, &save_data_node);

  // Get filename params
  std::string filename_poses;
  std::string filename_map;
  n.param<std::string>("save_test_data/filename_poses", filename_poses, "/home/andrew/tests/data/default_pose_data.csv");
  n.param<std::string>("save_test_data/filename_map", filename_map, "/home/andrew/tests/data/default_map_data.csv");
  
  // Write to file stopping criteria
  float timeout;
  n.param("save_test_data/timeout", timeout, (float)3.0);

  // Declare and read in the node update rate from the launch file parameters
  double rate;
  n.param("save_test_data/rate", rate, (double)10.0); // 10Hz default
  ros::Rate r(rate);

  // Run the node until ROS quits
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    if (save_data_node.received_map && save_data_node.received_odometry && (std::difftime(std::time(NULL), save_data_node.last_map_time) > timeout) &&
    (std::difftime(std::time(NULL), save_data_node.last_odometry_time) > timeout)) {
      ROS_INFO("No new messages for %0.1f seconds.  Bag is complete, writing data to file.", timeout);
      break;
    }
  }

  // Write histories to file when core dies (not sure if this works as intended)
  if (save_data_node.WriteHistoryToCSV(filename_poses, filename_map)) {
    ROS_INFO("Data written to file.");
  }
  else
  {
    ROS_INFO("Data could not be written to file.");
  }
  return 0;
}