#ifndef Message_H_
#define  Message_H_

#include "DataElement.hpp"
#include "DataFrame.hpp"

namespace V2X {
	/** 1 MessageFrame
	 * 车辆基本安全消息，通过广播将自身的实时状态告知周围车辆。
	 */
	typedef struct BasicSafetyMessage
	{
		MsgCount_t msgCnt; //1
		int id;//1
		int plateNo;//1
		DSecond_t secMark;//1
		f_Position3D pos;//1
		f_PositionConfidenceSet accuracy;//1
		e_TransmissionState transmisson; //档位
		DE_Speed_t speed;//1
		Heading_t heading;//1
		DE_SteeringWheelAngle_t angle; //方向盘转角
		f_MontionConfidenceSet motionCfd; //车辆运动状态精度
		f_AccelerationSet4Way accelSet;
		f_BrakeSystemStatus brakes;
		f_VehicleSize size;
		f_VehicleClassification vehicleClass;
		f_VehicleSafetyExtension safetyExt;
	}m_BasicSafetyMessage;


	/** 2 MapData
	 * 地图信息，由路测单元广播，向车辆传递局部区域的地图信息。
	 * 包括局部区域的路口信息，车道信息，道路之间的连接关系
	 */
	typedef struct MapData {
		MsgCount_t msgCnt;
		MinuteOfTheYear_t timeStamp;
		NodeList_t nodes;
	}m_MapData;

	/** 3 RoadSideInformation
	 *  路测信息
	 */
	typedef struct RoadSideInformation
	{
		MsgCount_t msgCnt;
		MinuteOfTheYear_t timeStamp;
		int id;
		int rsiId;
		AlertType_t alertType;
		DescriptiveName_t description;
		Priority_t priority;
		f_Position3D refPos;
		PathPointList_t alertPath;
		Radius_t alertRadius;
	}m_RoadSideInformation;

	/** 4 RoadsideSafetyMessage
	 * 路测安全信息
	 */
	typedef struct RoadsideSafetyMessage
	{
		MsgCount_t msgCnt;
		int id; //RSU ID
		f_Position3D refPos;
		ParticipantList_t participaints;
	}m_RoadsideSafetyMessage;

	/** 5 SPAT
	 * 信号灯信息，包含一个或多个路口信号灯的当前状态信息。结合 MAP 消息，为车辆提供实时
	 * 的前方信号灯相位信息。
	 */
	typedef struct SPAT
	{
		MsgCount_t msgCnt;
		MinuteOfTheYear_t timeStamp;
		DescriptiveName_t name; //OPTIONAL
		IntersectionStateList_t intersections;
	}m_SPAT;

	typedef struct MessageFrame
	{
		m_BasicSafetyMessage bsmFrame;
		m_MapData mapFrame;
		m_RoadsideSafetyMessage rsmFrame;
		m_SPAT spatFrame;
		m_RoadSideInformation rsiFrame;
	}mf_MessageFrame;
}

#endif // !Message_H_