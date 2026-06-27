#include "trans_node_ros.hpp"

// #include "mgrs_converter.hpp"
// #include "coord_conver/tsf.h"
// #include "SimOneSMAPI.h"
#include <ros/ros.h>
// #include "cybertron/glm/vec3.hpp"
// #include "cybertron/glm/quat.hpp"
// #include "cybertron/glm/mat4.hpp"

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

ros::Publisher* ros_trans_node::pub_gps_p = nullptr;
ros::Publisher* ros_trans_node::pub_ground_truth_p = nullptr;
ros::Publisher* ros_trans_node::pub_image_p = nullptr;
ros::Publisher* ros_trans_node::pub_point_cloud_p = nullptr;
ros::Publisher* ros_trans_node::pub_radar_p = nullptr;
ros::Publisher* ros_trans_node::pub_sensor_p = nullptr;
ros::Publisher* ros_trans_node::pub_laneinfo_p = nullptr;

std::ofstream ros_trans_node::log_img;

ros_trans_node::ros_trans_node(){}
ros_trans_node::~ros_trans_node(){}

int64_t ros_trans_node::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

void ros_trans_node::pub_gps_cb(const char* mainVehicleId, SimOne_Data_Gps *pGps)
{
  msg_gen::gps gps_d;
  gps_d.posX = pGps->posX;
  gps_d.posY = pGps->posY;
  gps_d.posZ = pGps->posZ;
  gps_d.oriX = pGps->oriX;
  gps_d.oriY = pGps->oriY;
  gps_d.oriZ = pGps->oriZ;
  gps_d.velX = pGps->velX;
  gps_d.velY = pGps->velY;
  gps_d.velZ = pGps->velZ;
  gps_d.throttle = pGps->throttle;
  gps_d.brake = pGps->brake;
  gps_d.steering = pGps->steering;
  gps_d.gear = pGps->gear;
  gps_d.accelX = pGps->accelX;
  gps_d.accelY = pGps->accelY;
  gps_d.accelZ = pGps->accelZ;
  gps_d.angVelX = pGps->angVelX;
  gps_d.angVelY = pGps->angVelY;
  gps_d.angVelZ = pGps->angVelZ;
  gps_d.wheelSpeedFL = pGps->wheelSpeedFL;
  gps_d.wheelSpeedFR = pGps->wheelSpeedFR;
  gps_d.wheelSpeedRL = pGps->wheelSpeedRL;
  gps_d.wheelSpeedRR = pGps->wheelSpeedRR;
  gps_d.engineRpm = pGps->engineRpm;
  gps_d.odometer = pGps->odometer;
  gps_d.extraStateSize = pGps->extraStateSize;
  for (int i=0; i<sizeof(pGps->extraStates)/sizeof(pGps->extraStates[0]); i++)
  {
	  gps_d.extraStates.emplace_back(pGps->extraStates[i]);
  }

  pub_gps_p->publish(gps_d);
}

void ros_trans_node::pub_ground_truth_cb(const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle)
{
  msg_gen::obstacle obstacle_d;
  obstacle_d.obstacleSize = pObstacle->obstacleSize;
  for (int i=0; i<obstacle_d.obstacleSize; i++)
  {
    msg_gen::SimOne_Data_Obstacle_Entry obstacle_entry;
    obstacle_entry.id = pObstacle->obstacle[i].id;
    obstacle_entry.viewId = pObstacle->obstacle[i].viewId;
    obstacle_entry.type.SimOne_Obstacle_Type = pObstacle->obstacle[i].type;
    obstacle_entry.theta = pObstacle->obstacle[i].theta;
    obstacle_entry.posX = pObstacle->obstacle[i].posX;
    obstacle_entry.posY = pObstacle->obstacle[i].posY;
    obstacle_entry.posZ = pObstacle->obstacle[i].posZ;
    obstacle_entry.oriX = pObstacle->obstacle[i].oriX;
    obstacle_entry.oriY = pObstacle->obstacle[i].oriY;
    obstacle_entry.oriZ = pObstacle->obstacle[i].oriZ;
    obstacle_entry.velX = pObstacle->obstacle[i].velX;
    obstacle_entry.velY = pObstacle->obstacle[i].velY;
    obstacle_entry.velZ = pObstacle->obstacle[i].velZ;
    obstacle_entry.length = pObstacle->obstacle[i].length;
    obstacle_entry.width = pObstacle->obstacle[i].width;
    obstacle_entry.height = pObstacle->obstacle[i].height;
    obstacle_d.obstacle.emplace_back(obstacle_entry);
  }
  pub_ground_truth_p->publish(obstacle_d);
}

void ros_trans_node::pub_image_cb(SimOne_Streaming_Image* pImage)
{
  printf("--------------------------------------------- t-img before pub: %ld\n", getCurrentTime());

  if (pImage->width != 0 && pImage->height != 0)
  {
    sensor_msgs::Image img_d;
    img_d.header.seq = pImage->frame;
    img_d.header.stamp = ros::Time::now();
    // img.header.stamp = image->timestamp; // long long timestamp millisecond
    img_d.header.frame_id = "short_camera";
    img_d.width = pImage->width;   // int Image resolution width 1920 max
    img_d.height = pImage->height; // int Image resolution height 1080 max
    img_d.encoding = "bgr8";
    img_d.is_bigendian = false;
    img_d.step = img_d.width * 3;
    img_d.data.resize(img_d.step * img_d.height);
    // memcpy(img.data.data(), image->imageData, img.step * img.height);	// char[] SOSM_IMAGE_DATA_SIZE_MAX 1920 x 1080 x 3 max
    memcpy(img_d.data.data(), pImage->imageData, pImage->imageDataSize); // char[] SOSM_IMAGE_DATA_SIZE_MAX 1920 x 1080 x 3 max	// int image data size
    // image->format;			// enum SimOne_Image_Format Image format, RGB only for now
    // image->version;		// int API version
    // image->dated;			// bool Mark if this data has been handled

    // log_img << "width: " << pImage->width << "\n";
    // log_img << "height: " << pImage->height << "\n";
    // log_img << "imageDataSize: " << pImage->imageDataSize << "\n";
    // log_img << "---------- frame: " << pImage->frame << " ------------\n";

    pub_image_p->publish(img_d);

    printf("======================== t-img pub done: %ld\n", getCurrentTime());
  }

}

void ros_trans_node::pub_point_cloud_cb(SimOne_Streaming_Point_Cloud* pPointcloud)
{
  printf("--------------------------------------------- t-pcd before pub: %ld\n", getCurrentTime());

  // static std::string sNames[] = {"x", "y", "z", "intensity", "ring"};
  // static int sOffsets[] = {0, 4, 8, 12, 14};
  // static int sDatatypes[] = {7, 7, 7, 2, 2};
  static std::string sNames[] = {"x", "y", "z", "intensity"};
  static int sOffsets[] = {0, 4, 8, 12};
  static int sDatatypes[] = {7, 7, 7, 7};

  sensor_msgs::PointCloud2 point_cloud_d;
  pcl::PCLPointCloud2 pointCloud;

  pointCloud.header.seq = pPointcloud->frame; // sequence number long long
  pointCloud.header.stamp = ros::Time::now().toNSec() / 1e3;
  pointCloud.header.frame_id = "base_link";
  pointCloud.point_step = pPointcloud->pointStep; // int
  pointCloud.row_step = pPointcloud->pointStep * pPointcloud->width;
  pointCloud.height = pPointcloud->height; // int
  pointCloud.width = pPointcloud->width;   // inti
  // printf("point cloud height: %d\n",pointCloud.height);

  for (int i = 0; i < 4; i++)
  {
    pcl::PCLPointField field;
    field.name = sNames[i].c_str();
    field.offset = sOffsets[i];
    field.datatype = sDatatypes[i];
    field.count = 1;
    pointCloud.fields.push_back(field);
  }

  pointCloud.is_bigendian = false;
  int size = pPointcloud->width * pPointcloud->height * pPointcloud->pointStep;
  pointCloud.data.reserve(size);
  pointCloud.data.resize(size);
  memcpy(&pointCloud.data[0], pPointcloud->pointCloudData, size * sizeof(char));
  pointCloud.is_dense = true;
  pcl_conversions::fromPCL(pointCloud, point_cloud_d); // pcl::toROSMsg(pointCloud, output_feature_msg.cluster);

  pub_point_cloud_p->publish(point_cloud_d);

  printf("======================== t-pcd pub done: %ld\n", getCurrentTime());
}

void ros_trans_node::pub_radar_detections_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)
{
  msg_gen::radardetection radar_detection_d;

  radar_detection_d.detectNum = pDetections->detectNum;
  for (int i=0; i<radar_detection_d.detectNum; i++)
  {
    msg_gen::SimOne_Data_RadarDetection_Entry radar_detection_entry;
    radar_detection_entry.id = pDetections->detections[i].id;
    radar_detection_entry.subId = pDetections->detections[i].subId;
    radar_detection_entry.type.SimOne_Obstacle_Type  = pDetections->detections[i].type;
    radar_detection_entry.posX = pDetections->detections[i].posX;
    radar_detection_entry.posY = pDetections->detections[i].posY;
    radar_detection_entry.posZ = pDetections->detections[i].posZ;
    radar_detection_entry.velX = pDetections->detections[i].velX;
    radar_detection_entry.velY = pDetections->detections[i].velY;
    radar_detection_entry.velZ = pDetections->detections[i].velZ;
    radar_detection_entry.range = pDetections->detections[i].range;
    radar_detection_entry.rangeRate = pDetections->detections[i].rangeRate;
    radar_detection_entry.azimuth = pDetections->detections[i].azimuth;
    radar_detection_entry.vertical = pDetections->detections[i].vertical;
    radar_detection_entry.snrdb = pDetections->detections[i].snrdb;
    radar_detection_entry.rcsdb = pDetections->detections[i].rcsdb;
    radar_detection_entry.probability = pDetections->detections[i].probability;
    radar_detection_d.detections.emplace_back(radar_detection_entry);
  }
  pub_radar_p->publish(radar_detection_d);
}

void ros_trans_node::pub_sensor_detections_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections *pGroundtruth)
{
  msg_gen::sensordetections sensor_detections_d;

  sensor_detections_d.objectSize = pGroundtruth->objectSize;
  for (int i=0; i<sensor_detections_d.objectSize; i++)
  {
    msg_gen::SimOne_Data_SensorDetections_Entry sensor_detection_entry;

    sensor_detection_entry.id = pGroundtruth->objects[i].id;
    sensor_detection_entry.type.SimOne_Obstacle_Type = pGroundtruth->objects[i].type;
    sensor_detection_entry.posX = pGroundtruth->objects[i].posX;
    sensor_detection_entry.posY = pGroundtruth->objects[i].posY;
    sensor_detection_entry.posZ = pGroundtruth->objects[i].posZ;
    sensor_detection_entry.oriX = pGroundtruth->objects[i].oriX;
    sensor_detection_entry.oriY = pGroundtruth->objects[i].oriY;
    sensor_detection_entry.oriZ = pGroundtruth->objects[i].oriZ;
    sensor_detection_entry.length = pGroundtruth->objects[i].length;
    sensor_detection_entry.width = pGroundtruth->objects[i].width;
    sensor_detection_entry.height = pGroundtruth->objects[i].height;
    sensor_detection_entry.range = pGroundtruth->objects[i].range;
    sensor_detection_entry.velX = pGroundtruth->objects[i].velX;
    sensor_detection_entry.velY = pGroundtruth->objects[i].velY;
    sensor_detection_entry.velZ = pGroundtruth->objects[i].velZ;
    sensor_detection_entry.probability = pGroundtruth->objects[i].probability;
    sensor_detection_entry.relativePosX = pGroundtruth->objects[i].relativePosX;
    sensor_detection_entry.relativePosY = pGroundtruth->objects[i].relativePosY;
    sensor_detection_entry.relativePosZ = pGroundtruth->objects[i].relativePosZ;
    sensor_detection_entry.relativeRotX = pGroundtruth->objects[i].relativeRotX;
    sensor_detection_entry.relativeRotY = pGroundtruth->objects[i].relativeRotY;
    sensor_detection_entry.relativeRotZ = pGroundtruth->objects[i].relativeRotZ;
    sensor_detection_entry.relativeVelX = pGroundtruth->objects[i].relativeVelX;
    sensor_detection_entry.relativeVelY = pGroundtruth->objects[i].relativeVelY;
    sensor_detection_entry.relativeVelZ = pGroundtruth->objects[i].relativeVelZ;
    sensor_detection_entry.bbox2dMinX = pGroundtruth->objects[i].bbox2dMinX;
    sensor_detection_entry.bbox2dMinY = pGroundtruth->objects[i].bbox2dMinY;
    sensor_detection_entry.bbox2dMaxX = pGroundtruth->objects[i].bbox2dMaxX;
    sensor_detections_d.objects.emplace_back(sensor_detection_entry);
  }
  pub_sensor_p->publish(sensor_detections_d);
}

void ros_trans_node::pub_sensor_laneInfo_cb(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pLaneInfo)
{
  msg_gen::laneinfo lane_info_d;

  lane_info_d.id = pLaneInfo->id;
  lane_info_d.laneType.LaneType = pLaneInfo->laneType;
  lane_info_d.laneLeftID = pLaneInfo->laneLeftID;
  lane_info_d.laneRightID = pLaneInfo->laneRightID;
  for (int i=0; i<sizeof(pLaneInfo->lanePredecessorID) / sizeof(pLaneInfo->lanePredecessorID[0]); i++)
  {
    lane_info_d.lanePredecessorID.emplace_back(pLaneInfo->lanePredecessorID[i]);
  }
  for (int i=0; i<sizeof(pLaneInfo->laneSuccessorID) / sizeof(pLaneInfo->laneSuccessorID[0]); i++)
  {
    lane_info_d.laneSuccessorID.emplace_back(pLaneInfo->laneSuccessorID[i]);
  }
  lane_info_d.l_Line.lineID = pLaneInfo->l_Line.lineID;
  lane_info_d.l_Line.lineType.BoundaryType = pLaneInfo->l_Line.lineType;
  lane_info_d.l_Line.lineColor.BoundaryColor = pLaneInfo->l_Line.lineColor;
  lane_info_d.l_Line.linewidth = pLaneInfo->l_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->l_Line.linePoints) / sizeof(pLaneInfo->l_Line.linePoints[0]); i++)
  {
    msg_gen::SimOneData_Vec3f v3f;
    v3f.x = pLaneInfo->l_Line.linePoints[i].x;
    v3f.y = pLaneInfo->l_Line.linePoints[i].y;
    v3f.z = pLaneInfo->l_Line.linePoints[i].z;
    lane_info_d.l_Line.linePoints.emplace_back(v3f);
  }
  lane_info_d.l_Line.linecurveParameter.C0 = pLaneInfo->l_Line.linecurveParameter.C0;
  lane_info_d.l_Line.linecurveParameter.C1 = pLaneInfo->l_Line.linecurveParameter.C1;
  lane_info_d.l_Line.linecurveParameter.C2 = pLaneInfo->l_Line.linecurveParameter.C2;
  lane_info_d.l_Line.linecurveParameter.C3 = pLaneInfo->l_Line.linecurveParameter.C3;
  lane_info_d.l_Line.linecurveParameter.firstPoints.x = pLaneInfo->l_Line.linecurveParameter.firstPoints.x;
  lane_info_d.l_Line.linecurveParameter.firstPoints.y = pLaneInfo->l_Line.linecurveParameter.firstPoints.y;
  lane_info_d.l_Line.linecurveParameter.firstPoints.z = pLaneInfo->l_Line.linecurveParameter.firstPoints.z;
  lane_info_d.l_Line.linecurveParameter.endPoints.x = pLaneInfo->l_Line.linecurveParameter.endPoints.x;
  lane_info_d.l_Line.linecurveParameter.endPoints.y = pLaneInfo->l_Line.linecurveParameter.endPoints.y;
  lane_info_d.l_Line.linecurveParameter.endPoints.z = pLaneInfo->l_Line.linecurveParameter.endPoints.z;
  lane_info_d.l_Line.linecurveParameter.length = pLaneInfo->l_Line.linecurveParameter.length;

  lane_info_d.c_Line.lineID = pLaneInfo->c_Line.lineID;
  lane_info_d.c_Line.lineType.BoundaryType = pLaneInfo->c_Line.lineType;
  lane_info_d.c_Line.lineColor.BoundaryColor = pLaneInfo->c_Line.lineColor;
  lane_info_d.c_Line.linewidth = pLaneInfo->c_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->c_Line.linePoints) / sizeof(pLaneInfo->c_Line.linePoints[0]); i++)
  {
    msg_gen::SimOneData_Vec3f v3f;
    v3f.x = pLaneInfo->c_Line.linePoints[i].x;
    v3f.y = pLaneInfo->c_Line.linePoints[i].y;
    v3f.z = pLaneInfo->c_Line.linePoints[i].z;
    lane_info_d.c_Line.linePoints.emplace_back(v3f);
  }
  lane_info_d.c_Line.linecurveParameter.C0 = pLaneInfo->c_Line.linecurveParameter.C0;
  lane_info_d.c_Line.linecurveParameter.C1 = pLaneInfo->c_Line.linecurveParameter.C1;
  lane_info_d.c_Line.linecurveParameter.C2 = pLaneInfo->c_Line.linecurveParameter.C2;
  lane_info_d.c_Line.linecurveParameter.C3 = pLaneInfo->c_Line.linecurveParameter.C3;
  lane_info_d.c_Line.linecurveParameter.firstPoints.x = pLaneInfo->c_Line.linecurveParameter.firstPoints.x;
  lane_info_d.c_Line.linecurveParameter.firstPoints.y = pLaneInfo->c_Line.linecurveParameter.firstPoints.y;
  lane_info_d.c_Line.linecurveParameter.firstPoints.z = pLaneInfo->c_Line.linecurveParameter.firstPoints.z;
  lane_info_d.c_Line.linecurveParameter.endPoints.x = pLaneInfo->c_Line.linecurveParameter.endPoints.x;
  lane_info_d.c_Line.linecurveParameter.endPoints.y = pLaneInfo->c_Line.linecurveParameter.endPoints.y;
  lane_info_d.c_Line.linecurveParameter.endPoints.z = pLaneInfo->c_Line.linecurveParameter.endPoints.z;
  lane_info_d.c_Line.linecurveParameter.length = pLaneInfo->c_Line.linecurveParameter.length;

  lane_info_d.r_Line.lineID = pLaneInfo->r_Line.lineID;
  lane_info_d.r_Line.lineType.BoundaryType = pLaneInfo->r_Line.lineType;
  lane_info_d.r_Line.lineColor.BoundaryColor = pLaneInfo->r_Line.lineColor;
  lane_info_d.r_Line.linewidth = pLaneInfo->r_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->r_Line.linePoints) / sizeof(pLaneInfo->r_Line.linePoints[0]); i++)
  {
    msg_gen::SimOneData_Vec3f v3f;
    v3f.x = pLaneInfo->r_Line.linePoints[i].x;
    v3f.y = pLaneInfo->r_Line.linePoints[i].y;
    v3f.z = pLaneInfo->r_Line.linePoints[i].z;
    lane_info_d.r_Line.linePoints.emplace_back(v3f);
  }
  lane_info_d.r_Line.linecurveParameter.C0 = pLaneInfo->r_Line.linecurveParameter.C0;
  lane_info_d.r_Line.linecurveParameter.C1 = pLaneInfo->r_Line.linecurveParameter.C1;
  lane_info_d.r_Line.linecurveParameter.C2 = pLaneInfo->r_Line.linecurveParameter.C2;
  lane_info_d.r_Line.linecurveParameter.C3 = pLaneInfo->r_Line.linecurveParameter.C3;
  lane_info_d.r_Line.linecurveParameter.firstPoints.x = pLaneInfo->r_Line.linecurveParameter.firstPoints.x;
  lane_info_d.r_Line.linecurveParameter.firstPoints.y = pLaneInfo->r_Line.linecurveParameter.firstPoints.y;
  lane_info_d.r_Line.linecurveParameter.firstPoints.z = pLaneInfo->r_Line.linecurveParameter.firstPoints.z;
  lane_info_d.r_Line.linecurveParameter.endPoints.x = pLaneInfo->r_Line.linecurveParameter.endPoints.x;
  lane_info_d.r_Line.linecurveParameter.endPoints.y = pLaneInfo->r_Line.linecurveParameter.endPoints.y;
  lane_info_d.r_Line.linecurveParameter.endPoints.z = pLaneInfo->r_Line.linecurveParameter.endPoints.z;
  lane_info_d.r_Line.linecurveParameter.length = pLaneInfo->r_Line.linecurveParameter.length;

  lane_info_d.ll_Line.lineID = pLaneInfo->ll_Line.lineID;
  lane_info_d.ll_Line.lineType.BoundaryType = pLaneInfo->ll_Line.lineType;
  lane_info_d.ll_Line.lineColor.BoundaryColor = pLaneInfo->ll_Line.lineColor;
  lane_info_d.ll_Line.linewidth = pLaneInfo->ll_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->ll_Line.linePoints) / sizeof(pLaneInfo->ll_Line.linePoints[0]); i++)
  {
    msg_gen::SimOneData_Vec3f v3f;
    v3f.x = pLaneInfo->ll_Line.linePoints[i].x;
    v3f.y = pLaneInfo->ll_Line.linePoints[i].y;
    v3f.z = pLaneInfo->ll_Line.linePoints[i].z;
    lane_info_d.ll_Line.linePoints.emplace_back(v3f);
  }
  lane_info_d.ll_Line.linecurveParameter.C0 = pLaneInfo->ll_Line.linecurveParameter.C0;
  lane_info_d.ll_Line.linecurveParameter.C1 = pLaneInfo->ll_Line.linecurveParameter.C1;
  lane_info_d.ll_Line.linecurveParameter.C2 = pLaneInfo->ll_Line.linecurveParameter.C2;
  lane_info_d.ll_Line.linecurveParameter.C3 = pLaneInfo->ll_Line.linecurveParameter.C3;
  lane_info_d.ll_Line.linecurveParameter.firstPoints.x = pLaneInfo->ll_Line.linecurveParameter.firstPoints.x;
  lane_info_d.ll_Line.linecurveParameter.firstPoints.y = pLaneInfo->ll_Line.linecurveParameter.firstPoints.y;
  lane_info_d.ll_Line.linecurveParameter.firstPoints.z = pLaneInfo->ll_Line.linecurveParameter.firstPoints.z;
  lane_info_d.ll_Line.linecurveParameter.endPoints.x = pLaneInfo->ll_Line.linecurveParameter.endPoints.x;
  lane_info_d.ll_Line.linecurveParameter.endPoints.y = pLaneInfo->ll_Line.linecurveParameter.endPoints.y;
  lane_info_d.ll_Line.linecurveParameter.endPoints.z = pLaneInfo->ll_Line.linecurveParameter.endPoints.z;
  lane_info_d.ll_Line.linecurveParameter.length = pLaneInfo->ll_Line.linecurveParameter.length;

  lane_info_d.rr_Line.lineID = pLaneInfo->rr_Line.lineID;
  lane_info_d.rr_Line.lineType.BoundaryType = pLaneInfo->rr_Line.lineType;
  lane_info_d.rr_Line.lineColor.BoundaryColor = pLaneInfo->rr_Line.lineColor;
  lane_info_d.rr_Line.linewidth = pLaneInfo->rr_Line.linewidth;
  for (int i=0; i<sizeof(pLaneInfo->rr_Line.linePoints) / sizeof(pLaneInfo->rr_Line.linePoints[0]); i++)
  {
    msg_gen::SimOneData_Vec3f v3f;
    v3f.x = pLaneInfo->rr_Line.linePoints[i].x;
    v3f.y = pLaneInfo->rr_Line.linePoints[i].y;
    v3f.z = pLaneInfo->rr_Line.linePoints[i].z;
    lane_info_d.rr_Line.linePoints.emplace_back(v3f);
  }
  lane_info_d.rr_Line.linecurveParameter.C0 = pLaneInfo->rr_Line.linecurveParameter.C0;
  lane_info_d.rr_Line.linecurveParameter.C1 = pLaneInfo->rr_Line.linecurveParameter.C1;
  lane_info_d.rr_Line.linecurveParameter.C2 = pLaneInfo->rr_Line.linecurveParameter.C2;
  lane_info_d.rr_Line.linecurveParameter.C3 = pLaneInfo->rr_Line.linecurveParameter.C3;
  lane_info_d.rr_Line.linecurveParameter.firstPoints.x = pLaneInfo->rr_Line.linecurveParameter.firstPoints.x;
  lane_info_d.rr_Line.linecurveParameter.firstPoints.y = pLaneInfo->rr_Line.linecurveParameter.firstPoints.y;
  lane_info_d.rr_Line.linecurveParameter.firstPoints.z = pLaneInfo->rr_Line.linecurveParameter.firstPoints.z;
  lane_info_d.rr_Line.linecurveParameter.endPoints.x = pLaneInfo->rr_Line.linecurveParameter.endPoints.x;
  lane_info_d.rr_Line.linecurveParameter.endPoints.y = pLaneInfo->rr_Line.linecurveParameter.endPoints.y;
  lane_info_d.rr_Line.linecurveParameter.endPoints.z = pLaneInfo->rr_Line.linecurveParameter.endPoints.z;
  lane_info_d.rr_Line.linecurveParameter.length = pLaneInfo->rr_Line.linecurveParameter.length;

  pub_laneinfo_p->publish(lane_info_d);
}

void ros_trans_node::run_pub()
{
  pub_gps = handle_gps.advertise<msg_gen::gps>(gps_topic.c_str(), 1);
  pub_ground_truth = handle_ground_truth.advertise<msg_gen::obstacle>(ground_truth_topic.c_str(), 1);
  pub_image = handle_image.advertise<sensor_msgs::Image>(image_topic.c_str(), 1);
  pub_point_cloud = handle_point_cloud.advertise<sensor_msgs::PointCloud2>(point_cloud_topic.c_str(), 1);
  pub_radar = handle_radar.advertise<msg_gen::radardetection>(radar_topic.c_str(), 1);
  pub_sensor = handle_sensor.advertise<msg_gen::sensordetections>(sensor_topic.c_str(), 1);
  pub_laneinfo = handle_laneinfo.advertise<msg_gen::laneinfo>(lane_info_topic.c_str(), 1);
  pub_gps_p = &pub_gps;
  pub_ground_truth_p = &pub_ground_truth;
  pub_image_p = &pub_image;
  pub_point_cloud_p = &pub_point_cloud;
  pub_radar_p = &pub_radar;
  pub_sensor_p = &pub_sensor;
  pub_laneinfo_p = &pub_laneinfo;

  if (!SimOneAPI::SetGpsUpdateCB(pub_gps_cb)){printf("\033[1m\033[31m[run_pub]: SetSimOneGpsCB Failed!\033[0m\n");}
  if (!SimOneAPI::SetGroundTruthUpdateCB(pub_ground_truth_cb)){printf("\033[1m\033[31m[run_pub]: SetSimOneGroundTruthCB Failed!\033[0m\n");}
  if(enable_physical_sensor) {
    if (!SimOneAPI::SetStreamingImageUpdateCB(img_ip.c_str(), img_port, pub_image_cb)){printf("\033[1m\033[31m[run_pub]: SetImageUpdateCB Failed!\033[0m\n");}
    if (!SimOneAPI::SetStreamingPointCloudUpdateCB(pcd_ip.c_str(), pcd_port, pcd_port_info, pub_point_cloud_cb)){printf("\033[1m\033[31m[run_pub]: SetPointCloudUpdateCB Failed!\033[0m\n");}
  }
  if (!SimOneAPI::SetRadarDetectionsUpdateCB(pub_radar_detections_cb)){printf("\033[1m\033[31m[run_pub]: SetRadarDetectionsUpdateCB Failed!\033[0m\n");}
  if (!SimOneAPI::SetSensorDetectionsUpdateCB(pub_sensor_detections_cb)){printf("\033[1m\033[31m[run_pub]: SetSensorDetectionsUpdateCB Failed!\033[0m\n");}
  if (!SimOneAPI::SetSensorLaneInfoCB(pub_sensor_laneInfo_cb)){printf("\033[1m\033[31m[run_pub]: SetSensorLaneInfoCB Failed!\033[0m\n");}
}

void ros_trans_node::rcv_ctl_cb(const msg_gen::control& msg)
{
  // std_msgs::Header header;
  // msg.header.time;
  ros::Time current_time = ros::Time::now();
  // pCtrl->steering = mPre_steering_angle;

  pCtrl->steering = -msg.steering * 1.5;

  // msg.steering_angle_velocity;
  pCtrl->throttle = msg.throttle;
  pCtrl->brake = msg.brake;

  ESimOne_Gear_Mode gear;
  switch (msg.gear.GearMode) // NONE/PARKING/REVERSE/NEUTRAL/DRIVE/LOW
  {
  case msg.gear.EGearMode_Neutral:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Neutral; // 0
    break;
  case msg.gear.EGearMode_Drive:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive; // 1
    break;
  case msg.gear.EGearMode_Reverse:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Reverse; // 2
    break;
  case msg.gear.EGearMode_Parking:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Parking; // 3
    break;
  case msg.gear.EGearManualMode_1:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_1; // 4
    break;
  case msg.gear.EGearManualMode_2:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_2; // 5
    break;
  case msg.gear.EGearManualMode_3:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_3; // 6
    break;
  case msg.gear.EGearManualMode_4:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_4; // 7
    break;
  case msg.gear.EGearManualMode_5:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_5; // 8
    break;
  case msg.gear.EGearManualMode_6:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_6; // 9
    break;
  case msg.gear.EGearManualMode_7:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_7; // 10
    break;
  case msg.gear.EGearManualMode_8:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_8; // 11
    break;
  default:
    gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Neutral; // 0
  }
  pCtrl->gear = gear;
  pCtrl->isManualGear = false;
  pCtrl->timestamp = current_time.toNSec();

  if (!SimOneAPI::SetDrive(vehicle_id.c_str(), pCtrl.get()))
  {
    printf("\033[1m\033[31m[rcv_ctl_cb] ------ SimOneSM::SetDrive Failed!\033[0m\n");
  }
}

void ros_trans_node::rcv_pose_ctl_cb(const msg_gen::pose_control& msg)
{
    SimOne_Data_Pose_Control pose_ctl;

    pose_ctl.posX = msg.posX;
    pose_ctl.posY = msg.posY;
    pose_ctl.posZ = msg.posZ;
    pose_ctl.oriX = msg.oriX;
    pose_ctl.oriY = msg.oriY;
    pose_ctl.oriZ = msg.oriZ;
    pose_ctl.autoZ = msg.autoZ;

    if (!SimOneAPI::SetPose(0, &pose_ctl))
    {
      printf("\033[1m\033[31m[rcv_pose_ctl_cb] Set Pose failed!\033[0m\n");
    }
}

void ros_trans_node::rcv_esp_ctl_cb(const msg_gen::esp_control& msg)
{
  SimOne_Data_ESP_Control esp;

  esp.stopDistance = msg.stopDistance;
  esp.velocityLimit = msg.velocityLimit;
  esp.steering = msg.steering;
  esp.steerTorque = msg.steerTorque;
  esp.accel = msg.accel;
  esp.accelUpperLimit = msg.accelUpperLimit;
  esp.accelLowerLimit = msg.accelLowerLimit;
  esp.accelUpperComfLimit = msg.accelUpperComfLimit;
  esp.accelLowerComfLimit = msg.accelLowerComfLimit;
  esp.standStill = msg.standStill;
  esp.driveOff = msg.driveOff;
  esp.brakeMode = msg.brakeMode;
  esp.vlcShutdown = msg.vlcShutdown;
  esp.gearMode = msg.gearMode;

  // if (!SimOneAPI::SetESPDrive(0, &esp))
  // {
  //   printf("\033[1m\033[31m[rcv_esp_ctl_cb] Set ESPDrive failed!\033[0m\n");
  // }
}

void ros_trans_node::run_rcv()
{
  sub_ctl = handle_ctl.subscribe(ctl_topic.c_str(), 1, &ros_trans_node::rcv_ctl_cb, this);
  sub_pose_ctl = handle_pose_ctl.subscribe(pose_ctl_topic.c_str(), 1, &ros_trans_node::rcv_pose_ctl_cb, this);
  sub_esp_ctl = handle_esp_ctl.subscribe(esp_ctl_topic.c_str(), 1, &ros_trans_node::rcv_esp_ctl_cb, this);
}

void ros_trans_node::run()
{
	init();
	std::thread pub_thread = std::thread(std::bind(&ros_trans_node::run_pub, this));
	std::thread rcv_thread = std::thread(std::bind(&ros_trans_node::run_rcv, this));
	pub_thread.detach();
	rcv_thread.detach();
}

void ros_trans_node::config_ini()
{
	rr::RrConfig config;
	config.ReadConfig("config.ini");
	bio_ip = config.ReadString("BridgeIO", "BridgeIO_IP", "");

  vehicle_id = config.ReadString("HostVehicle", "Vehicle_ID", "0");

  enable_physical_sensor = config.ReadInt("Sensor", "ENABLED", 0) > 0;
	img_ip = config.ReadString("Sensor", "IMG_IP", "127.0.0.1");
  img_port= config.ReadInt("Sensor", "IMG_PORT", 0);

	pcd_ip = config.ReadString("Sensor", "PCD_IP", "127.0.0.1");
  pcd_port= config.ReadInt("Sensor", "PCD_PORT", 0);
  pcd_port_info= config.ReadInt("Sensor", "PCD_PORT_INFO", 0);


	gps_topic = config.ReadString("ROS", "GPS_Topic", "");
	ground_truth_topic = config.ReadString("ROS", "GroundTruth_Topic", "");
	image_topic = config.ReadString("ROS", "Image_Topic", "");
	point_cloud_topic = config.ReadString("ROS", "PointCloud_Topic", "");
	radar_topic = config.ReadString("ROS", "Radar_Topic", "");
	sensor_topic = config.ReadString("ROS", "Sensor_Topic", "");
	lane_info_topic = config.ReadString("ROS", "LaneInfo_Topic", "");
	// int port = config.ReadInt("ROS", "Port", 0);
	// float lat = config.ReadFloat("MapBase", "BASE_LATITUDE", 0);
	// float log = config.ReadFloat("MapBase", "BASE_LONGITUDE", 0);
	// float alt = config.ReadFloat("MapBase", "BASE_ALTITUDE", 0);
	ctl_topic = config.ReadString("ROS", "CTL_Topic", "");
	pose_ctl_topic = config.ReadString("ROS", "POSE_CTL_Topic", "");
	esp_ctl_topic = config.ReadString("ROS", "ESP_CTL_Topic", "");
}
void ros_trans_node::simone_ini()
{
  printf("Connecting BridgeIO With Ip : %s ...\n", bio_ip.c_str());
  if (!SimOneAPI::InitSimOneAPI(vehicle_id.c_str(), false, bio_ip.c_str()))
  {
    printf("SimOneAPI::InitSimOneAPI Failed!\n");
  }

  // Get Case-Status
  // while (true)
  // {
  //     if (!(SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Running))
  //     {
  //         if (SimOneAPI::GetCaseRunStatus() == SimOne_Case_Status::SimOne_Case_Status_Stop) // || shutdown_requested)
  //         {
  //             SimOneAPI::TerminateSimOneAPI();
  //             std::cout << "preparing to shut down..." << std::endl;
  //             return;
  //         }
  //         printf("\033[1m\033[31mSimOneAPI::GetCaseRunStatus Failed!\033[0m\n");
  //         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  //         continue;
  //     }
  //     break;
  // }

  // int timeout = 20;
  // if (!SimOneSM::LoadHDMap(timeout))
  // {
  //     printf("\033[1m\033[31mLoad HDMap Failed!\033[0m\n");
  // }
  // SetGPSBase(lat, log, alt); Point wgs = GetWGS(pos);
  // ------------  simone api config end ------------

  // ------------ RegisterSimOneVehicleState Start ------------
  // SimOne_Data_Vehicle_State indics[INDEICS_SIZE];
  // indics[INDEX_SO_M_SW] = SO_M_SW; // steering wheel torque, unit: N.m
  // indics[INDEX_SO_X_L1] = SO_X_L1;
  // indics[INDEX_SO_Y_L1] = SO_Y_L1;
  // indics[INDEX_SO_Z_L1] = SO_Z_L1;
  // indics[INDEX_SO_X_L2] = SO_X_L2;
  // indics[INDEX_SO_Y_L2] = SO_Y_L2;
  // indics[INDEX_SO_Z_L2] = SO_Z_L2;
  // indics[INDEX_SO_X_R1] = SO_X_R1;
  // indics[INDEX_SO_Y_R1] = SO_Y_R1;
  // indics[INDEX_SO_Z_R1] = SO_Z_R1;
  // indics[INDEX_SO_X_R2] = SO_X_R2;
  // indics[INDEX_SO_Y_R2] = SO_Y_R2;
  // indics[INDEX_SO_Z_R2] = SO_Z_R2;
  // indics[INDEX_SO_Steer_SW] = SO_Steer_SW;

  // while (true)
  // {
  //   if (!SimOneAPI::RegisterSimOneVehicleState(indics, INDEICS_SIZE))
  //   {
  //     printf("\033[1m\033[31mSimOneAPI::RegisterSimOneVehicleState Faile!\033[0m\n");
  //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //     continue;
  //   }
  //   break;
  // }

	if (!SimOneAPI::GetCaseInfo(&caseInfo))
  {
    printf("SimOneAPI::GetCaseInfo Failed!\n");
  }
  // ------------ RegisterSimOneVehicleState End ------------
}

void ros_trans_node::init()
{
	config_ini();
	simone_ini();
}

void ros_trans_node::monitor_case_status()
{
  while (true)
  {
    if(SimOneAPI::GetCaseRunStatus() ==ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      SimOneAPI::TerminateSimOneAPI();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  }
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ros_trans_node");
	ros_trans_node tn;
	tn.run();
	ros::spinOnce();
	tn.monitor_case_status();
	return 0;
}
