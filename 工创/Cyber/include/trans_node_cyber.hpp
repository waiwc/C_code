#ifndef TRANS_NODE_CYBER_HPP
#define TRANS_NODE_CYBER_HPP

#pragma once

#include <signal.h>
#include <memory>
#include <random>
#include <tuple>
#include <thread>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/wait.h>

// #include <Python.h>

#include "Service/SimOneIOStruct.h"
#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneStreamingAPI.h"

#include "glm/vec3.hpp"
#include "glm/quat.hpp"
#include "3rdparty/Config.h"
#include "3rdparty/timer.hpp"
#include "3rdparty/verify_ip.hpp"

#include "TaskCyberChassisImu.hpp"
#include "TaskCyberContiRadar.hpp"
#include "TaskCyberControl.hpp"
#include "TaskCyberGps.hpp"
#include "TaskCyberObstacles.hpp"
#include "TaskCyberPoseControl.hpp"
#include "TaskCyberRoutingReq.hpp"
#include "TaskCyberTrafficLight.hpp"
#include "TaskCyberTrajectory.hpp"
#include "TaskCyberChassisImu_Gps.hpp"
// #include "TaskCyberImage.hpp"
// #include "TaskCyberPointCloud.hpp"

// #include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>
// #include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Imu.h>
// #include <geometry_msgs/TwistStamped.h>

// #include <tf2/LinearMath/Transform.h>
// #include <tf2/convert.h>
// #include <tf2/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/transform_broadcaster.h>

#define INDEX_SIZE 110

template <class Type>
Type stringToNum(const std::string& str)
{
  std::istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}

class cyber_trans_node
{
public:
  cyber_trans_node();
  ~cyber_trans_node();

  static int64_t getCurrentTime();

  static void exit_handler(int signo);
  void exit_ini();
  static bool pram_opt(int argc, char* argv[]);
  static void config_ini();
  void simone_ini();
  void init();

  void monitor_case_status();

  void run_autopilot();
  void run_rcv();
  void run_pub();
  void run();

private: 
  static bool mSigTerm;

  static std::ofstream log_img;

  static std::string bio_ip;
  static std::string vehicle_id;
  static bool enable_physical_sensor;

  // static Img_Param_T mImgParam;
  // static PCD_Param_T mPCDParam;

  static std::string mChannelControl_R;
  static std::string mChannelEstimate_R;
  static std::string mChannelRoutingResponse_R;
  static std::string mChannelTrajectory_R;
  static std::string mChannelChassis_W;
  static std::string mChannelContiRadar_W;
  static std::string mChannelGps_W;
  static std::string mChannelImage_W;
  static std::string mChannelIMU_W;
  static std::string mChannelStat_W;
  static std::string mChannelObstacles_W;
  static std::string mChannelPointCloud_W;
  static std::string mChannelRoutingReq_W;
  static std::string mChannelTrafficLight_W;

  SimOne_Data_CaseInfo caseInfo;

  CW_Chassis_Imu mChassisImu;
  CW_Gps mGps;
  CW_ChassisImuGps mChassisImuGps;
  CW_ContiRadar mContiRadar;
  CW_Obstacles mObstacles;
  CW_RoutingReq mRoutingReq;
  CW_TrafficLight mTrafficLight;
  CW_Trajectory mTrajectory;

  // CW_Image mImage;
  // CW_PointCloud mPointCloud;

  CW_Ctl mCtl;
};

#endif
