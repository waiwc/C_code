#ifndef TRANS_NODE_ROS_HPP
#define TRANS_NODE_ROS_HPP

#pragma once

#include "Service/SimOneIOStruct.h"
// #include "SimOneIOStruct_STD.h"
// #include "SimOneNetAPI.h"

#include "SimOneServiceAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneStreamingAPI.h"

#include "glm/vec3.hpp"
#include "glm/quat.hpp"
#include "Config.h"

#include "caseinfo.h"
#include "control.h"
#include "control_trajectory.h"
#include "driver_status.h"
#include "EBrakeMode.h"
#include "EGearMode.h"
#include "environment.h"
#include "ESimOneData_BoundaryColor.h"
#include "ESimOneData_BoundaryType.h"
#include "ESimOneData_LineCurveParameter.h"
#include "ESimOneLaneType.h"
#include "ESimone_Vehicle_EventInfo_Type.h"
#include "esp_control.h"
#include "ESteeringMode.h"
#include "EThrottleMode.h"
#include "gps.h"
#include "laneinfo.h"
#include "mainvehicle.h"
#include "mainvehicle_info.h"
#include "mainvehicle_status.h"
#include "obstacle.h"
#include "pose_control.h"
#include "prediction.h"
#include "radardetection.h"
#include "sensorconfigurations.h"
#include "sensordetections.h"
#include "signal_lights.h"
#include "SimOne_Data_LaneLineInfo.h"
#include "SimOne_Data_Position_Info_Array.h"
#include "SimOne_Data_Position_Info.h"
#include "SimOne_Data_RadarDetection_Entry.h"
#include "SimOne_Data_SensorDetections_Entry.h"
#include "SimOne_Data_Traffic_Info.h"
#include "SimOne_Data_TrafficLight.h"
#include "SimOne_Data_Trajectory_Entry.h"
#include "SimOne_Data_UltrasonicRadarDetection_Entry.h"
#include "SimOne_Data_UltrasonicRadar.h"
#include "SimOneData_Vec3f.h"
#include "SimOne_Driver_Status.h"
#include "SimOne_Obstacle_Type.h"
#include "SimOneSensorConfiguration.h"
#include "SimOne_TrafficLight_Status.h"
#include "trafficlights.h"
#include "trajectory.h"
#include "trajectory_point.h"
#include "ultrasonicradars.h"
#include "vehicle_eventinfo.h"
#include "vehicle_extra.h"
#include "waypoints_entry.h"
#include "waypoints.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <random>
#include <tuple>

#include <thread>

// #define MOVING_SLOWLY 5 // dds_obstacle::ObstacleMotionState::OBS_STATUS_MOVING_SLOWLY  m/s
// #define INDEICS_SIZE 14 // 主车 四个轮子坐标 + 扭矩 + 方向盘转角
// #define INDEX_SO_M_SW 0
// #define INDEX_SO_X_L1 1
// #define INDEX_SO_Y_L1 2
// #define INDEX_SO_Z_L1 3
// #define INDEX_SO_X_L2 4
// #define INDEX_SO_Y_L2 5
// #define INDEX_SO_Z_L2 6
// #define INDEX_SO_X_R1 7
// #define INDEX_SO_Y_R1 8
// #define INDEX_SO_Z_R1 9
// #define INDEX_SO_X_R2 10
// #define INDEX_SO_Y_R2 11
// #define INDEX_SO_Z_R2 12
// #define INDEX_SO_Steer_SW 13

template <class Type>
Type stringToNum(const std::string& str)
{
	std::istringstream iss(str);
	Type num;
	iss >> num;
	return num;    
}

class ros_trans_node
{
public:
  ros_trans_node();
  ~ros_trans_node();

  static int64_t getCurrentTime();

  void config_ini();
  void simone_ini();
  void init();

  void monitor_case_status();

  void run_rcv();
  void run_pub();
  void run();
  
private:
  static void pub_gps_cb(const char* mainVehicleId, SimOne_Data_Gps *pGps);
  static void pub_ground_truth_cb(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle);
  static void pub_image_cb(SimOne_Streaming_Image* pImage);
  static void pub_point_cloud_cb(SimOne_Streaming_Point_Cloud* pPointcloud);
  static void pub_radar_detections_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections);
  static void pub_sensor_detections_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth);
  static void pub_sensor_laneInfo_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo);
  // static void Pub_OSISetSensorData(int mainVehicleId, int sensorId, SimOne_Data_OSI *pSensorData);
  // static void Pub_OSISetGroundTruth(int mainVehicleId, SimOne_Data_OSI *pGroundtruth);
  // static void Pub_ScenarioEventCB(int mainVehicleId, const char* event, const char* data);

  void rcv_ctl_cb(const msg_gen::control& msg);
  void rcv_esp_ctl_cb(const msg_gen::esp_control& msg);
  void rcv_pose_ctl_cb(const msg_gen::pose_control& msg);

  ros::NodeHandle handle_gps, handle_ground_truth, handle_image, handle_point_cloud, handle_radar, handle_sensor, handle_laneinfo;
  ros::NodeHandle handle_ctl, handle_esp_ctl, handle_pose_ctl;

  ros::Publisher pub_gps, pub_ground_truth, pub_image, pub_point_cloud, pub_radar, pub_sensor, pub_laneinfo;
  ros::Subscriber sub_ctl, sub_pose_ctl, sub_esp_ctl;
  static ros::Publisher* pub_gps_p;
  static ros::Publisher* pub_ground_truth_p;
  static ros::Publisher* pub_image_p;
  static ros::Publisher* pub_point_cloud_p;
  static ros::Publisher* pub_radar_p;
  static ros::Publisher* pub_sensor_p;
  static ros::Publisher* pub_laneinfo_p;
  // ros::Timer timer_image;

  static std::ofstream log_img;

  std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();

  std::string bio_ip;
  std::string vehicle_id;
  SimOne_Data_CaseInfo caseInfo;

  bool enable_physical_sensor;
  std::string img_ip;
  unsigned short img_port;

	std::string pcd_ip;
  unsigned short pcd_port;
  unsigned short pcd_port_info;

	std::string gps_topic;
	std::string ground_truth_topic;
  std::string image_topic;
  std::string point_cloud_topic;
  std::string radar_topic;
  std::string sensor_topic;
  std::string lane_info_topic;

  std::string ctl_topic;
  std::string pose_ctl_topic;
  std::string esp_ctl_topic;
};

#endif
