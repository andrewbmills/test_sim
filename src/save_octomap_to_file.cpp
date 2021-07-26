#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

octomap::OcTree* map_octree;
bool received_map = false;

void CallbackOctomapFull(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*msg);
  return;
}

void CallbackOctomapBinary(const octomap_msgs::Octomap::ConstPtr msg)
{
  if (msg->data.size() == 0) return;
  if (received_map == false) received_map = true;
  delete map_octree;
  map_octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*msg);
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_octomap_to_file");
  ros::NodeHandle n;

  ros::Subscriber sub_octomap_full = n.subscribe("octomap_full", 1, CallbackOctomapFull);
  ros::Subscriber sub_octomap_binary = n.subscribe("octomap_binary", 1, CallbackOctomapBinary);

  std::string filename = "/home/andrew/Desktop/octomap.ot";
  n.param<std::string>("save_octomap_to_file/filename", filename, "/home/andrew/Desktop/octomap.ot");

  // Run the node until ROS quits
  ros::Rate r(1.0);
  while (ros::ok())
  {
    r.sleep(); // Node sleeps to update at a rate as close as possible to the updateRate parameter
    ros::spinOnce(); // All subscriber callbacks are called here.
    ROS_INFO("Waiting for a map message...");
    if (received_map) {
      // Export the received octree to a .ot file
      ROS_INFO("Octomap message received, writing to file...", filename);
      map_octree->write(filename);
      ROS_INFO("Octomap saved.  Exiting node...");
      break;
    }
  }

  return 0;
}