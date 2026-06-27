#include "PNC_Sample.h"

dumper pncapi_sample::data_dmper;

SimOne_Data_Gps pncapi_sample::m_gps;
std::atomic<bool> pncapi_sample::m_flip;

pncapi_sample::pncapi_sample()
{
  m_gps.posX = 0.0;
  m_gps.posY = 0.0;
  m_gps.posZ = 0.0;
  m_gps.oriX = 0.0;
  m_gps.oriY = 0.0;
  m_gps.oriZ = 0.0;
  m_flip = false;
}

pncapi_sample::~pncapi_sample(){}

int64_t pncapi_sample::getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

// GPS更新回调
void pncapi_sample::get_gps(const char *mainVehicleId, SimOne_Data_Gps *pGps)
{
  data_dmper.dump_gps(mainVehicleId, pGps);

  if (!m_flip.load())
  {
    m_gps.frame = pGps->frame;
    m_gps.posX = pGps->posX;                 // Position X on Opendrive (by meter)
    m_gps.posY = pGps->posY;                 // Position Y on Opendrive (by meter)
    m_gps.posZ = pGps->posZ;                 // Position Z on Opendrive (by meter)
    m_gps.oriX = pGps->oriX;                 // Rotation X on Opendrive (by radian)
    m_gps.oriY = pGps->oriY;                 // Rotation Y on Opendrive (by radian)
    m_gps.oriZ = pGps->oriZ;                 // Rotation Z on Opendrive (by radian)
    m_gps.velX = pGps->velX;                 // MainVehicle Velocity X on Opendrive (by meter)
    m_gps.velY = pGps->velY;                 // MainVehicle Velocity Y on Opendrive (by meter)
    m_gps.velZ = pGps->velZ;                 // MainVehicle Velocity Z on Opendrive (by meter)
    m_gps.throttle = pGps->throttle;         //MainVehicle throttle
    m_gps.brake = pGps->brake;               //MainVehicle brake;
    m_gps.steering = pGps->steering;         //MainVehicle Wheel Steering angle (deg)
    m_gps.gear = pGps->gear;                 // MainVehicle gear position
    m_gps.accelX = pGps->accelX;             // MainVehicle Acceleration X on Opendrive (by meter)
    m_gps.accelY = pGps->accelY;             // MainVehicle Acceleration Y on Opendrive (by meter)
    m_gps.accelZ = pGps->accelZ;             // MainVehicle Acceleration Z on Opendrive (by meter)
    m_gps.angVelX = pGps->angVelX;           // MainVehicle Angular Velocity X on Opendrive (by meter)
    m_gps.angVelY = pGps->angVelY;           // MainVehicle Angular Velocity Y on Opendrive (by meter)
    m_gps.angVelZ = pGps->angVelZ;           // MainVehicle Angular Velocity Z on Opendrive (by meter)
    m_gps.wheelSpeedFL = pGps->wheelSpeedFL; // Speed of front left wheel (by meter/sec)
    m_gps.wheelSpeedFR = pGps->wheelSpeedFR; // Speed of front right wheel (by meter/sec)
    m_gps.wheelSpeedRL = pGps->wheelSpeedRL; // Speed of rear left wheel (by meter/sec)
    m_gps.wheelSpeedRR = pGps->wheelSpeedRR; // Speed of rear right wheel (by meter/sec)
    m_gps.engineRpm = pGps->engineRpm;       // Speed of engine (by r/min)
    m_gps.odometer = pGps->odometer;         // odometer in meter.
    m_flip.store(true);
  }
}

// 通过设置位置点移动主车(无动力学)，点位消息由算法端提供
void pncapi_sample::set_pose_ctl()
{
  if (m_flip.load())
  {
    SimOne_Data_Pose_Control pose_ctl;
    // 基于主车当前位置 X 方向移动一米
    pose_ctl.posX = m_gps.posX + cos(m_gps.oriZ); // Position X on Opendrive (by meter)
    pose_ctl.posY = m_gps.posY + sin(m_gps.oriZ); // Position Y on Opendrive (by meter)
    pose_ctl.posZ = m_gps.posZ;                      // Position Z on Opendrive (by meter)
    pose_ctl.oriX = m_gps.oriX;                      // Rotation X on Opendrive (by radian)
    pose_ctl.oriY = m_gps.oriY;                      // Rotation Y on Opendrive (by radian)
    pose_ctl.oriZ = m_gps.oriZ;                      // Rotation Z on Opendrive (by radian)
    pose_ctl.autoZ = false;                          // Automatically set Z according to scene

    std::cout << "posX/Y/Z: [" << pose_ctl.posX << ", " << pose_ctl.posY << ", " << pose_ctl.posZ << "]" << std::endl;
    std::cout << "oriX/Y/Z: [" << pose_ctl.oriX << ", " << pose_ctl.oriY << ", " << pose_ctl.oriZ << "]" << std::endl;
    std::cout << "------------ set_pose_ctl ------------" << m_gps.frame << std::endl;

    /*
     * 设置主车位置 API
     * input param:
     *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
     * output param:
     *     pPose: Pose to set(input)
     * return: Success or not
    */
    if (!SimOneAPI::SetPose(0, &pose_ctl))
    {
      std::cout << "Set Pose failed!" << std::endl;
    }

    m_flip.store(false);
  }

}

// 通过油门、刹车、方向等消息驱动主车(有动力学)，控制参数由算法端提供
void pncapi_sample::set_drive_ctl()
{
    std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();

    pCtrl->timestamp = getCurrentTime(); // uint64  时间戳，单位us
    pCtrl->throttleMode = ESimOne_Throttle_Mode::ESimOne_Throttle_Mode_Speed; // vehicle speed, m/s,   in this mode, brake input is ignored        
    pCtrl->throttle = 10; // m/s  double 油门开度 0-100。100表示最大油门驱动
    // pCtrl->brakeMode = ESimOne_Brake_Mode_Percent;
    // pCtrl->brake = data.brake_pedal_bar(); // double 制度踏板开度 单位bar
    pCtrl->steeringMode = ESimOne_Steering_Mode::ESimOne_Steering_Mode_SteeringWheelAngle; // steering wheel angle, degree
    pCtrl->steering = 0;
    // pCtrl->handbrake = false;
    pCtrl->isManualGear = false;
    pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive; // forward gear for automatic gear
	  // pCtrl->clutch;

    std::cout << "timestamp: " << pCtrl->timestamp << std::endl;
    std::cout << "throttleMode: " << pCtrl->throttleMode << std::endl;
    std::cout << "throttle: " << pCtrl->throttle << std::endl;
    std::cout << "steeringMode: " << pCtrl->steeringMode << std::endl;
    std::cout << "steering: " << pCtrl->steering << std::endl;
    std::cout << "isManualGear: " << pCtrl->isManualGear << std::endl;
    std::cout << "gear: " << pCtrl->gear << std::endl;
    std::cout << "------ set_drive_control_cb ------" << std::endl;

    /*
		 * 主车控制
		 * input param:
		 *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		 *     pControl: vehicle control data
		 * return: Success or not
		*/
    if (!SimOneAPI::SetDrive(0, pCtrl.get()))
    {
      std::cout << "SimOneAPI::SetDrive Failed!" << std::endl;
    }
}

// 通过规划轨迹点驱动主车(有动力学)，不可同时使用SetDrive
void pncapi_sample::set_drive_trajectory()
{
  std::unique_ptr<SimOne_Data_Control_Trajectory> pTraj = std::make_unique<SimOne_Data_Control_Trajectory>();
  int n = 10;

  if (m_flip.load())
  {
    float posx = m_gps.posX;  // position x
    float posy = m_gps.posY;  // position y
    float speed = 10;            // m/s
    float accel = 1;             // accelelation m/s^2
    float theta = m_gps.oriZ; // yaw   rad
    float kappa = 0;             // curvature
    float relative_time = 0;     // time relative to the first trajectory point
    float s = 0;                 // distance from the first trajectory point

    pTraj->point_num = n;
    for (int i = 0; i < n; i++)
    {
      pTraj->points[i].posx = posx;
      pTraj->points[i].posy = posy;
      pTraj->points[i].speed = speed;
      pTraj->points[i].accel = accel;
      pTraj->points[i].theta = theta;
      pTraj->points[i].kappa = kappa;
      pTraj->points[i].relative_time = relative_time;
      pTraj->points[i].s = s;

      posx = posx + cos(theta);
      posy = posy + sin(theta);
      relative_time += 1;
      s += 1;

      std::cout << "points[" << i << "].posx: " << pTraj->points[i].posx << std::endl;
      std::cout << "points[" << i << "].posy: " << pTraj->points[i].posy << std::endl;
      std::cout << "points[" << i << "].speed: " << pTraj->points[i].speed << std::endl;
      std::cout << "points[" << i << "].accel: " << pTraj->points[i].accel << std::endl;
      std::cout << "points[" << i << "].theta: " << pTraj->points[i].theta << std::endl;
      std::cout << "points[" << i << "].kappa: " << pTraj->points[i].kappa << std::endl;
      std::cout << "points[" << i << "].relative_time: " << pTraj->points[i].relative_time << std::endl;
      std::cout << "points[" << i << "].s: " << pTraj->points[i].s << std::endl;
    }
    pTraj->isReverse = false;
    std::cout << "isReverse: " << pTraj->isReverse << std::endl;

    /*
		 * 主车控制(通过规划轨迹，不可同时使用SetDrive)
		 * input param:
		 *     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		 *     pControlTrajectory: vehicle planning trajectory
		 * return: Success or not
		*/
    if (!SimOneAPI::SetDriveTrajectory("0", pTraj.get()))
    {
      std::cout << "SimOneAPI::SetDrive Failed!" << std::endl;
    }

    m_flip.store(false);
  }
}

// 接的场景事件消息的回调
void pncapi_sample::set_scenario_event(const char* source, const char* target, const char* type, const char* content)
{
  std::string event_info(content);
  std::string car_flw_prefix("CarFollowing");
  if (!event_info.compare(0, car_flw_prefix.length(), car_flw_prefix))
  {
    // car_following
  }
  std::cout << "Get Scenario Event: " << event_info.c_str() << std::endl;
}

// 获取传感器检测到物体的对应真值
void pncapi_sample::get_sensor_detection(const char *mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pGroundtruth)
{
  if (pGroundtruth->objectSize == 0)
  {
    std::cout << "No object detected!" << std::endl;
    return;
  }

  data_dmper.dump_sensor_detections(mainVehicleId, sensorId, pGroundtruth);
}

// 获取传感器检测到车道与车道线数据回调
void pncapi_sample::get_sensor_laneInfo(const char *mainVehicleId, const char *sensorId, SimOne_Data_LaneInfo *pLaneInfo)
{
  data_dmper.dump_sensor_laneInfo(mainVehicleId, sensorId, pLaneInfo);
}

// 主车控制 
void pncapi_sample::mv_ctl()
{
  /*
    * 注册主车GPS更新回调
		* input param:
		*     cb: GPS data update callback function
    * return: Success or not
    */
  if (!SimOneAPI::SetGpsUpdateCB(get_gps))
  {
    printf("\033[1m\033[31m[run_pub]: SetSimOneGpsCB Failed!\033[0m\n");
  }

  /*
		* 注册场景事件回调
		* input param:
		*     cb: scenario event callback function
		* output param:
		*     mainVehicleId: Vehilcle index, configure order of web UI, starts from 0
		*     event: Command sent to the mainVehicle
		*     data: Not used yet
		* return: Success or not
		*/
  if (!SimOneAPI::SetScenarioEventCB(set_scenario_event))
  {
    std::cout << "SimOneSM::SetScenarioEventCB Failed!" << std::endl;
  }

  /*
		* 注册传感器真值信息更新回调
		* input param:
		*   cb: Groundtruth data fetch callback function
		* return: Success or not
		*/
  if (!SimOneAPI::SetSensorDetectionsUpdateCB(get_sensor_detection))
  {
    std::cout << "SimOneAPI::SetSensorDetectionsUpdateCB Failed!" << std::endl;
  }

  /*
    * 注册获取传感器检测到车道与车道线数据回调
		* input param[in]
		*   cb: Groundtruth data fetch callback function
		* return: Success or not
		*/
  if (!SimOneAPI::SetSensorLaneInfoCB(get_sensor_laneInfo))
  {
    std::cout << "SimOneAPI::SetSensorLaneInfoCB Failed!" << std::endl;
  }

  Timer timer_pose_ctl, timer_drive_ctl, timer_drive_trajectory;

  // timer_pose_ctl.start(10, std::bind(&pncapi_sample::set_pose_ctl, this));
  timer_drive_ctl.start(100, std::bind(&pncapi_sample::set_drive_ctl, this));
  // timer_drive_trajectory.start(300, std::bind(&pncapi_sample::set_drive_trajectory, this));

  while(true)
  {
    if (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop)
    {
      // timer_pose_ctl.stop();
      // timer_drive_ctl.stop();
      timer_drive_trajectory.stop();

      SimOneAPI::TerminateSimOneAPI();
      return;
    }
  
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  
}

// SimOne 案例初始化
void pncapi_sample::simone_ini()
{
  /*
		* SimOne API主入口
		* input param:
		*	    hostVehicleId: host vehicle ID(from 0 to 9)
		*     isFrameSync: synchronize frame or not
    * return: Success or not
		*/
  if (!SimOneAPI::InitSimOneAPI("0", true, "127.0.0.1"))
  {
    std::cout << "SimOneAPI::InitSimOneAPI Failed!\n"<< std::endl;
  }

  SimOne_Data_CaseInfo caseInfo;
  /*
		* 获取案例详情
		* output param:
		*	    pCaseInfo: caseName,caseId,taskId,sessionId
		* return: Success or not
		*/
  if (!SimOneAPI::GetCaseInfo(&caseInfo))
  {
    std::cout << "SimOneAPI::GetCaseInfo Failed!" << std::endl;
  }
}

int main(int argc, char* argv[])
{
  pncapi_sample sp;
  sp.simone_ini();
  sp.mv_ctl();

  return 0;
}
