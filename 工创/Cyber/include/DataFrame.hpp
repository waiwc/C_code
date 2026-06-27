
#include "DataElement.hpp"
#include "SSD/SimVector.h"

/**
 * Sample
 */
typedef struct Rectangle {
	long	 height;
	long	 width;
} f_Rectangle;


/** 1 DF_AccelerationSet4Way
 * 定义车辆四轴加速度
 */
typedef struct AccelerationSet4Way
{
	Acceleration_t lon; //前向为正
	Acceleration_t lat; //横向加速度，向右为正
	VerticalAcceleration_t vert; //垂直加速度，向下为正
	YawRate_t yaw; //横摆角速度 顺时针为正
}f_AccelerationSet4Way;


/** 2 DF_BrakeSysytemStatus
 *  定义车辆刹车系统状态
 */
typedef struct BrakeSystemStatus
{
	e_BrakePedalStatus brakePadel; //刹车踏板状态
	e_BrakeAppliedStatus wheelBrakes;  //四轮刹车状态
	e_TractionControlStatus traction; //油门状态
	e_AntiLockBrakeStatus abs;  //防抱死系统
	e_StabilityControlStatus scs; //稳定控制系统状态
	e_BrakeBoostApplied brakeBoost; //刹车助力系统
	e_AuxiliaryBrakeStatus auxBrakes; //手刹状态

}f_BrakeSystemStatus;



/** 21 DF_NodeReferenceID
 * 参考ID全局唯一ID加上内部ID组成
 */
typedef struct NodeReferenceID
{
	RoadRegulatorID_t region;
	NodeID_t id;
}f_NodeReferenceID;



/** 3 DF_ConnectingLane
 *  用于定位上游车道转向链接的下游车道
 */
typedef struct ConnectingLane {
	LaneID_t lane;
	e_AllowedManeuvers mannuver; //允许运动方向
}f_ConnectingLane;



/** 4 DF_Connection
 *  定义车道与下游路段车道的连接关系
 */
typedef struct Connection
{
	f_NodeReferenceID remoteIntersection; //全局ID
	f_ConnectingLane connectingLane; 
	PhaseID_t phaseId;
}f_Connection;


/** 5 DF_ConnectsToList
 * 定义路段中每条车道在下游路口处与下游路段车道的转向连接关系列表
 */
typedef SSD::SimVector<f_Connection> ConnectsToList;


/** 6 DF_DDateTime
 * 定义完整的日期和时间数据单元
 */
typedef struct DDateTime
{
	DYear_t year;
	DMonth_t month;
	DDay_t day;
	DHour_t hour;
	DMinute_t minute;
	DSecond_t second;
	DTimeOffset_t offset;
}f_DDateTime;


/** 41 DF_Position3D
 * 三维坐标位置，经度纬度高度
 */
typedef struct Position3D
{
	Longitude_t lon;
	Latitude_t lat;
	Elevation_t elevation; //海拔高度
}f_Position3D;




/** 42 DF_PositionConfidenceSet
 * 定义位置的综合精度
 */
typedef struct PositionConfidenceSet
{
	e_PositionConfidence pos; //位置精度
	e_ElevationConfidence elevation; //高度精度
}f_PositionConfidenceSet;


/** 16 DF_MotionConfidenceSet
 * 描述车辆运动状态的精度
 */
typedef struct MontionConfidenceSet
{
	e_SpeedConfidence speedCfd;
	e_HeadingConfidence headingCfd;
	e_SteeringWheelAngleConfidence steerCfd;
}f_MontionConfidenceSet;


/** 7 DF_FullPositionVector
 * 完整参考轨迹信息，用于车辆历史轨迹数据单元
 */
typedef struct FullPositionVector
{
	f_DDateTime utcTime;
	f_Position3D pos;
	Heading_t heading;
	e_TransmissionState transmission; //档位状态
	DE_Speed_t speed; 
	f_PositionConfidenceSet posAccuracy; //三维位置精度
	e_TimeConfidence timeConfidence;
	f_PositionConfidenceSet posConfidence; //三维位置精度
	f_MontionConfidenceSet motionCfd; //车辆运动状态精度

}f_FullPositionVector;


/** 49 DF_TimeChangeDetails
 * 描述了信号灯一个相位状态的完整计时状态。
 */
typedef struct TimeChangeDetails
{
	TimeMark_t startTime;
	TimeMark_t minEndTime;
	TimeMark_t maxEndTime;
	TimeMark_t likelyEndTime;
	Confidence_t timeConfidence;
	TimeMark_t nextStartTime;
	TimeMark_t nextDuration;
}f_TimeChangeDetails;


/** 31 DF_PhaseState
	定义信号灯的一个相位状态。包含该状态对应的灯色，以及实时计时信息。
*/
typedef struct PhaseState
{
	e_LightState light;
	f_TimeChangeDetails timing;
}f_PhaseState;


/** 32 DF_PhaseStateList
 * 定义一个信号灯的一个相位中的相位状态列表。列表中每一个相位状态物理上对应一种相位灯
色，其属性包括了该状态的实时计时信息。
 */
typedef SSD::SimVector<f_PhaseState> PhaseStateList_t;

/** 29 DF_Phase
 *  定义信号灯相位，包括相位ID以及相位状态列表
 */
typedef struct Phase
{
	PhaseID_t id;
	PhaseStateList_t PhaseStates;
}f_Phase;


/** 30 DF_PhaseList
 * 定义一组信号灯包含的所有相位的列表。
  //PhaseList :: = SEQUENCE(SIZE(1..16)) OF Phase
 */
typedef  SSD::SimVector<f_Phase> PhaseList_t;


/** 8 DF_IntersectionState
 * 定义一个路口信号灯的属性和当前状态。包括路口 ID、信号灯工作状态、时间戳以及信号灯
的相位列表。
 */
typedef struct IntersectionState
{
	f_NodeReferenceID intersectionId;
	e_IntersectionStatusObject status;
	MinuteOfTheYear_t moy;
	DSecond_t timeStamp;
	PhaseList_t phases;
}f_IntersectionState;

/** 9 DF_IntersectionStateList
 * 定义路口信号灯集合
 */
typedef SSD::SimVector<f_IntersectionState> IntersectionStateList_t;


/** 13 DF_LaneTypeAttributes
 * 定义不同类别的车道属性和集合
 */
typedef struct LaneTypeAttributes
{
	e_LaneAttributes_Vehicle vehicle;
	e_LaneAttributes_CrossWalk crosswalk;
	e_LaneAttributes_Bike bikeLane;
	e_LaneAttributes_Sidewalk sideWalk;
	e_LaneAttributes_Barrier median;
	e_LaneAttributes_Striping striping;
	e_LaneAttributes_TrackedVehicle trackedVehicle;
	e_LaneAttributes_Parking parking;
}f_LaneTypeAttributes;


/** 11 DF_LaneAttributes
 * 定义车道属性
 */
typedef struct LaneAttributes
{
	e_LaneSharing shareWith;
	f_LaneTypeAttributes laneType;
}f_LaneAttributes;



/** 34 DF_Position-LL-24B
 * 24比特相对经纬度位置，表示当前位置点关于参考位置点的经纬度偏差。由两个12比特的经度、
 * 纬度偏差值组成。
 */
typedef struct Position_LL_XB
{
	OffsetLL_B12_t lon;
	OffsetLL_B12_t lat;
}f_Position_LL_XB;

/** 43 DF_PositionOffsetLL
 * 经纬度偏差，用来描述一个坐标点的相对位置。
 */
typedef struct PositionOffsetLL
{
	f_Position_LL_XB position_LL1;
}f_PositionOffsetLL;



/** 53 DF_VerticalOffset
 * 垂直方向位置偏差
 */
typedef union VerticalOffset
{
	VertOffset_BX_t offset;
}f_VerticalOffset;



/** 44 DF_PositionOffsetLLV
 * 定义三维的相对位置
 */
typedef struct PositionOffsetLLV
{
	f_PositionOffsetLL offsetLL;
	f_VerticalOffset offsetV;
}f_PositionOffsetLLV;

/** 46 DF_RoadPoint
 * 表示完整道路上标记的一个位置点属性。包括相对位置、位置点处路段截面的特殊属性集合，
 * 以及该位置点到下游相邻位置点之间局部路段的特殊属性集合。
 */
typedef struct RoadPoint {
	f_PositionOffsetLLV posOffset;
}f_RoadPoint;

//LocalENU
typedef struct RoadPoint_base
{
	float x;
	float y;
	float z;
}f_RoadPoint_base;

/** 33 DF_PointList
 *  定义一个有向路段上的中间位置点列表，用来表达路段上截面或片段的特殊属性。列表中所有
位置点需按上游至下游顺序排列。
 */
typedef SSD::SimVector<f_RoadPoint>  PointList_t;

/** 45 DF_RegulatorySpeedLimit
 * 定义限速属性。包括限速类型以及所规定的限速值
 */
typedef struct RegulatorySpeedLimit
{
	e_SpeedLimitType type;
	DE_Speed_t  speed;
}f_RegulatorySpeedLimit;


/** 48 DF_SpeedLimitList
 * 定义路段或车道的限速列表。
 */
typedef SSD::SimVector<f_RegulatorySpeedLimit> SpeedLimitList_t;

/** 10 DF_Lane
 *  定义车道
 */
typedef struct Lane
{
	LaneID_t laneId;
	f_LaneAttributes laneAttributes;
	e_AllowedManeuvers mannuvers;
	ConnectsToList connectsTo;
	SpeedLimitList_t speedLimits;
	PointList_t points;
}f_Lane;



/** 12 DF_LaneList
 *  定义一个路段中包含的车道列表
 */
typedef SSD::SimVector<f_Lane> LaneList_t;


/** 17 DF_Movement
 *  道路与下游路段的连接关系，以及该链接对应的本地路口处信号灯相位ID。
 */
typedef struct Movement
{
	f_NodeReferenceID remoteIntersection;
	PhaseID_t phaseId;
}f_Movement;

/** 18 DF_MovementList
 * 描述一条路段与下游路段的连接关系列表。
 */
typedef SSD::SimVector<f_Movement> MovementList_t;


/** 14 DF_Link
 * 定义路段，从一个节点到相邻另一个节点的道路为一个有向路段
 */
typedef struct Link
{
	DescriptiveName_t name;
	f_NodeReferenceID upstreamNodeId;
	SpeedLimitList_t speedLimits;
	LaneWidth_t lanewidth;
	PointList_t points;
	MovementList_t movements;
	LaneList_t lanes;
}f_Link;

/** 15 DF_LinkList
 *  定义路段列表
 */
typedef SSD::SimVector <f_Link> LinkList_t;


/** 19  DF_Node
 * 地图节点。节点是地图的最基本组成部分，可以是交叉路口，也可以是一个路段的端点。在地
 * 图上，相邻且有序的两个节点确定一条有向路段。
 * 节点属性包括名称、ID、位置以及与节点相连的上游路段集合。
 */
typedef struct Node
{
	DescriptiveName_t name;
	f_NodeReferenceID id;
	f_Position3D refPos;
	LinkList_t inLinks;
}f_Node;

/** 20 DF_NodeList
 *  定义地图节点列表。
 */
typedef SSD::SimVector<f_Node> NodeList_t;

/** 52 DF_VehicleSize
 * 车辆尺寸
 */
typedef struct VehicleSize
{
	VehicleWidth_t width;
	VehicleLength_t length;
	VehicleHeight_t height;
}f_VehicleSize;

/** 50 DF_VehicleClassification
 *  定义车辆的分类，可从多个维度对车辆类型进行定义
 */
typedef struct VehicleClassification
{
	BasicVehicleClass_t classification;
}f_VehicleClassification;


/** 22 DF_ParticipantData
 * 定义交通参与者的基本安全信息。对应 BSM 消息，该参与者信息由 RSU 探测得到。
 */
typedef struct ParticipantData
{
	e_ParticipantType ptcType;
	int ptcId;
	//-- 0 is RSU itself
	//-- 1..65535 represent participants detected by RSU
	e_SourceType source;//参与者类型
	int id;
	int plateNo;
	DSecond_t secMark;
	f_PositionOffsetLLV pos;
	f_PositionConfidenceSet accuracy;
	e_TransmissionState transmission; //档位状态
	DE_Speed_t speed; //速度
	Heading_t heading; //航向
	DE_SteeringWheelAngle_t angle; //方向盘角度
	f_MontionConfidenceSet motionCfd; //车辆运动状态的精度
	f_AccelerationSet4Way accelSet; //四轴加速度
	f_VehicleSize size;
	f_VehicleClassification vehicleClass;
}f_ParticipantData;


/** 23 DF_ParticipantList
 *  定义交通参与者列表
 */
typedef SSD::SimVector<f_ParticipantData> ParticipantList_t;

/** 25 DF_PathHistoryPoint
 *  定义车辆的历史轨迹点，包括位置时间，以及轨迹点处的车速位置精度和航向
 */
typedef struct PathHistoryPoint
{
	f_PositionOffsetLLV llvOffset;
	DTimeOffset_t timeOffset;
	DE_Speed_t speed;
	f_PositionConfidenceSet psoAccuracy;
	CoarseHeading_t heading;
}f_PathHistoryPoint;

/** 26 DF_PathHistoryPointList
 * 定义车辆的历史轨迹点列表
 */
typedef SSD::SimVector<f_PathHistoryPoint> PathHistoryPointList_t;


/** 24 DF_PathHistory
 * 定义车辆历史轨迹
 */
typedef struct PathHistory
{
	f_FullPositionVector initialPosition;
	e_GNSSstatus currGNSSstatus;
	PathHistoryPointList_t crumbData;
}f_PathHistory;

/** 27 DF_PathPointList
 *  定义RSA消息中，消息作用范围的车辆行进轨迹区段，该区段由有序位置点列组成，排列顺序与车辆行进方向一致
 */
typedef SSD::SimVector<f_PositionOffsetLLV> PathPointList_t;

/** 28 DF_PathPrediction
 *  定义车辆的预测路线
 */
typedef struct PathPrediction
{
	RadiusOfCurvature_t radiusOfCurve;
	Confidence_t confidence;
}f_PathPrediction;


/** 47 DF_SignalState
 *  定义一个路口信号灯的属性和当前状态。
 * 包括工作模式、当前时间戳、以及实时的相位信息列表
 */
typedef struct SignalState
{
	e_IntersectionStatusObject status; //工作机状态
	MinuteOfTheYear_t moy;
	DSecond_t timeStamp;
	PhaseList_t phases;
}f_SignalState;

/** 51 DF_VehicleSafetyExtension
 *  定义车辆安全辅助信息集合，用于BSM消息中，做基础安全信息的填充
 * 包括车辆特殊事件状态，车辆历史轨迹，路线预测，车身灯光状态。
 */
typedef struct TrafficLight
{
	int state;
	int remainTime;
}f_TrafficLight;

typedef struct VehicleSafetyExtension
{
	int events;
	f_PathHistory pathHistory;
	f_PathPrediction pathPrediction;
	e_ExteriorLights lights;
	SSD::SimVector<f_TrafficLight> trafficLights;
}f_VehicleSafetyExtension;
