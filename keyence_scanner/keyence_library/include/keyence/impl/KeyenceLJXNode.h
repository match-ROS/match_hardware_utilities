#ifndef KEYENCE_LJX_NODE_H
#define KEYENCE_LJX_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <vector>

// Keyence LJ-X8000A Interface (Pfad/Name ggf. anpassen)
#include "LJX8_IF.h"

class KeyenceLJXNode
{
public:
  explicit KeyenceLJXNode(ros::NodeHandle& nh);
  ~KeyenceLJXNode();

  void spin();

private:
  void openCommunication();
  void setupProfileRequest();
  void acquireAndPublishProfile();

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  std::string ip_address_;
  int         port_;
  std::string frame_id_;
  std::string topic_name_;
  double      publish_rate_;

  long device_id_;  // m_nCurrentDeviceID-Äquivalent

  // Strukturen aus LJX8_IF.h
  LJX8IF_PROFILE_REQUEST  request_;
  LJX8IF_PROFILE_INFO     profile_info_;
  LJX8IF_PROFILE_HEADER   response_;      // Name je nach Header, ggf. anpassen

  std::vector<long> vec_profile_data_;
};

#endif // KEYENCE_LJX_NODE_H
