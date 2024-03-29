// C++ Standard Libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
// ROS libraries
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>
// Octomap libaries
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
// PCL libraries
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

struct MapLimits
{
  double x_min, x_max, y_min, y_max, z_min, z_max;
};

struct MapStats
{
  int free_count = 0;
  int occupied_count = 0;
  int unseen_count = 0;
  MapLimits limits;
};

struct BoundingBox
{
  bool is_on = false;
  float x_min, x_max, y_min, y_max, z_min, z_max;
};

bool InsideBoundingBox(float x, float y, float z, BoundingBox box)
{
  return (box.is_on)*(x >= box.x_min)*(x <= box.x_max)*(y >= box.y_min)*(y <= box.y_max)*(z >= box.z_min)*(z <= box.z_max);
}

MapLimits InitializeMapLimits(double x, double y, double z) {
  MapLimits limits;
  limits.x_min = x;
  limits.x_max = x;
  limits.y_min = y;
  limits.y_max = y;
  limits.z_min = z;
  limits.z_max = z;
  return limits;
}

MapLimits UpdateMapLimits(double x, double y, double z, MapLimits limits) {
  limits.x_min = std::min(x, limits.x_min);
  limits.x_max = std::max(x, limits.x_max);
  limits.y_min = std::min(y, limits.y_min);
  limits.y_max = std::max(y, limits.y_max);
  limits.z_min = std::min(z, limits.z_min);
  limits.z_max = std::max(z, limits.z_max);
  return limits;
}

MapLimits GetLimitsOverStats(std::vector<std::pair<double, MapStats>> stats) {
  MapLimits max_limits;
  if (stats.size()) MapLimits max_limits = stats[0].second.limits;
  for (int i=0; i<stats.size(); ++i) {
    max_limits.x_min = std::min(max_limits.x_min, stats[i].second.limits.x_min);
    max_limits.x_max = std::min(max_limits.x_max, stats[i].second.limits.x_max);
    max_limits.y_min = std::min(max_limits.y_min, stats[i].second.limits.y_min);
    max_limits.y_max = std::min(max_limits.y_max, stats[i].second.limits.y_max);
    max_limits.z_min = std::min(max_limits.z_min, stats[i].second.limits.z_min);
    max_limits.z_max = std::min(max_limits.z_max, stats[i].second.limits.z_max);
  }
  return max_limits;
}

MapStats GetMapStats(octomap::OcTree* map, BoundingBox box) {
  MapStats stats;
  // Expand pruned nodes in octree
  map->expand();

  octomap::OcTree::leaf_iterator it = map->begin_leafs();
  stats.limits = InitializeMapLimits((double)it.getX(), (double)it.getY(), (double)it.getZ());

  // Iterate through expanded octree
  for(octomap::OcTree::leaf_iterator it = map->begin_leafs(),
       end=map->end_leafs(); it!=end; ++it) {
    stats.limits = UpdateMapLimits(it.getX(), it.getY(), it.getZ(), stats.limits);
    if (InsideBoundingBox(it.getX(), it.getY(), it.getZ(), box)) {
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
  }
  return stats;
}

MapStats GetMapStats(pcl::PointCloud<pcl::PointXYZI>::Ptr map, BoundingBox box) {
  MapStats stats;
  // Iterate through expanded octree
  for(int i=0; i<map->points.size(); i++) {
    pcl::PointXYZI p = map->points[i];
    if (i == 0) stats.limits = InitializeMapLimits((double)p.x, (double)p.y, (double)p.z);
    stats.limits = UpdateMapLimits((double)p.x, (double)p.y, (double)p.z, stats.limits);
    if (InsideBoundingBox(p.x, p.y, p.z, box)) {
      if ((p.intensity <= 0.19) && (p.intensity >=-0.19))
      {
        // Add to occupied count
        stats.occupied_count++;
      }
      else if (p.intensity > 0.19)
      {
        // Add to free count
        stats.free_count++;
      }
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
      double begin_time = -1.0;
      ros::Time simulation_time;

      // Map
      octomap::OcTree* map_octree;
      std::vector<std::pair<double, MapStats>> maps_stats_history;

      // Pose History
      std::vector<std::pair<double, geometry_msgs::Pose>> pose_history;

      // Map bounding box
      BoundingBox box;

    // Method Definitions
    void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg);
    void CallbackOdometry(const nav_msgs::Odometry msg);
    void CallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg);
    void CallbackClock(const rosgraph_msgs::Clock msg);
    bool WriteHistoryToCSV(std::string filename_map, std::string filename_poses, float voxel_size);
};

void NodeManager::CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  maps_stats_history.push_back(std::make_pair(simulation_time.toSec(), GetMapStats(map_octree, box)));
  last_map_time = std::time(NULL);
  return;
}

void NodeManager::CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  maps_stats_history.push_back(std::make_pair(simulation_time.toSec(), GetMapStats(map_octree, box)));
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

void NodeManager::CallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud);
  maps_stats_history.push_back(std::make_pair(simulation_time.toSec(), GetMapStats(cloud, box)));
  last_map_time = std::time(NULL);
  return;
}

void NodeManager::CallbackClock(const rosgraph_msgs::Clock msg)
{
  simulation_time = msg.clock;
  return;
}

bool NodeManager::WriteHistoryToCSV(std::string filename_poses, std::string filename_map, float voxel_size)
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

  MapLimits map_limits = GetLimitsOverStats(maps_stats_history);
  
  // Write map data to file
  f.open(filename_map, std::ios::out);
  f << voxel_size;
  f << "\n";
  f << map_limits.x_min << "," << map_limits.x_max << "," << map_limits.y_min << ",";
  f << map_limits.y_max << "," << map_limits.z_min << "," << map_limits.z_max << "\n";
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
  ros::init(argc, argv, "save_test_data");
  ros::NodeHandle n;

  NodeManager save_data_node;

  ros::Subscriber sub_octomap_full = n.subscribe("octomap_full", 1, &NodeManager::CallbackOctomapFull, &save_data_node);
  ros::Subscriber sub_odometry = n.subscribe("odometry", 1, &NodeManager::CallbackOdometry, &save_data_node);
  ros::Subscriber sub_map_pointcloud = n.subscribe("map_cloud", 1, &NodeManager::CallbackPointCloud, &save_data_node);
  ros::Subscriber sub_octomap_binary = n.subscribe("octomap_binary", 1, &NodeManager::CallbackOctomapBinary, &save_data_node);
  ros::Subscriber sub_clock = n.subscribe("/clock", 1, &NodeManager::CallbackClock, &save_data_node);

  std::string filename_poses;
  std::string filename_map;
  float voxel_size;
  n.param<std::string>("save_test_data/filename_poses", filename_poses, "/home/andrew/tests/data/default_pose_data.csv");
  n.param<std::string>("save_test_data/filename_map", filename_map, "/home/andrew/tests/data/default_map_data.csv");
  n.param("save_test_data/voxel_size", voxel_size, (float)0.2);
  n.param("save_test_data/filter_by_bbox", save_data_node.box.is_on, false);
  n.param("save_test_data/bbox_min_x", save_data_node.box.x_min, (float)0.0);
  n.param("save_test_data/bbox_max_x", save_data_node.box.x_max, (float)0.0);
  n.param("save_test_data/bbox_min_y", save_data_node.box.y_min, (float)0.0);
  n.param("save_test_data/bbox_max_y", save_data_node.box.y_max, (float)0.0);
  n.param("save_test_data/bbox_min_z", save_data_node.box.z_min, (float)0.0);
  n.param("save_test_data/bbox_max_z", save_data_node.box.z_max, (float)0.0);
  float timeout;
  n.param("save_test_data/timeout", timeout, (float)5.0);
  double rate;
  n.param("save_test_data/rate", rate, (double)10.0); // 10Hz default
  ros::Rate r(rate);

  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    if (save_data_node.received_map && save_data_node.received_odometry && (std::difftime(std::time(NULL), save_data_node.last_map_time) > timeout) &&
    (std::difftime(std::time(NULL), save_data_node.last_odometry_time) > timeout)) {
      ROS_INFO("No new messages for %0.1f seconds.  Bag is complete, writing data to file.", timeout);
      break;
    }
  }

  if (save_data_node.WriteHistoryToCSV(filename_poses, filename_map, voxel_size)) {
    ROS_INFO("Data written to file.");
  }
  else
  {
    ROS_INFO("Data could not be written to file.");
  }
  return 0;
}