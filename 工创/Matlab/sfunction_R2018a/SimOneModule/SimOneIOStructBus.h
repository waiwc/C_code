#include <stdbool.h>

typedef struct {
	int id; // Obstacle ID
	int type; // Obstacle Type
	float theta; // Obstacle vertical rotation (by radian)
	float posX; // Obstacle Position X no Opendrive (by meter)
	float posY; // Obstacle Position Y no Opendrive (by meter)
	float posZ; // Obstacle Position Z no Opendrive (by meter)
	float velX; // Obstacle Velocity X no Opendrive (by meter)
	float velY; // Obstacle Velocity Y no Opendrive (by meter)
	float velZ; // Obstacle Velocity Z no Opendrive (by meter)
	float length; // Obstacle length
	float width; // Obstacle width
	float height; // Obstacle height
} SimOne_Data_Obstacle_Bus;

typedef struct{
	int id;					// Detection Object ID
	int subId;				// Detection Object Sub ID
	int type;				// Detection Object Type
	float posX;				// Detection Object Position X in meter
	float posY;				// Detection Object Position Y in meter
	float posZ;				// Detection Object Position Z in meter
	float velX;				// Detection Object Velocity X
	float velY;				// Detection Object Velocity Y
	float velZ;				// Detection Object Velocity Z
	float range;			// Detection Object relative range in meter
	float rangeRate;		// Detection Object relative range rate in m/s
	float azimuth;			// Detection Object azimuth angle
	float vertical;			// Detection Object vertical angle
	float snrdb;			// Signal noise ratio
	float rcsdb;			// Detection Object RCS
	float probability;		// Detection probability
} SimOne_Data_RadarDetection_Bus;

typedef struct{
	float obstacleRanges;
} SimOne_Data_UltrasonicRadar_Bus;

typedef struct{
    int id;					// Detection Object ID
	int type;	            // Detection Object Type
	float posX;				// Detection Object Position X in meter
	float posY;				// Detection Object Position Y in meter
	float posZ;				// Detection Object Position Z in meter
	float oriX;				// Rotation X in radian
	float oriY;				// Rotation Y in radian
	float oriZ;				// Rotation Z in radian
	float length;			// Detection Object Length in meter
	float width;			// Detection Object Width in meter
	float height;			// Detection Object Height in meter
	float range;			// Detection Object relative range in meter
	float velX;				// Detection Object Velocity X
	float velY;				// Detection Object Velocity Y
	float velZ;				// Detection Object Velocity Z
	float probability;		// Detection probability
	float relativePosX;		// Relative position X in sensor space
	float relativePosY;		// Relative position Y in sensor space
	float relativePosZ;		// Relative position Z in sensor space
	float relativeVelX;		// Relative velocity X in sensor space
	float relativeVelY;		// Relative velocity Y in sensor space
	float relativeVelZ;		// Relative velocity Z in sensor space
} SimOne_Data_SensorDetections_Bus;

typedef struct{
    float posX; // Position X no Opendrive (by meter)
	float posY; // Position Y no Opendrive (by meter)
	float posZ; // Position Z no Opendrive (by meter)
	float oriX; // Rotation X no Opendrive (by radian)
	float oriY; // Rotation Y no Opendrive (by radian)
	float oriZ; // Rotation Z no Opendrive (by radian)
	bool autoZ; // Automatically set Z according to scene
} SimOne_Data_Pose_Control_Bus;

typedef struct{
    float throttle;
	float brake;
    // steering mode
    int steeringMode;
	float steering;
	bool handbrake;
	bool isManualGear;
	// gear location
	int  gear;
} SimOne_Data_Control_Bus;

typedef struct{
    float posX; // Position X no Opendrive (by meter)
	float posY; // Position Y no Opendrive (by meter)
	float posZ; // Position Z no Opendrive (by meter)
	float oriX; // Rotation X no Opendrive (by radian)
	float oriY; // Rotation Y no Opendrive (by radian)
	float oriZ; // Rotation Z no Opendrive (by radian)
	float velX; // MainVehicle Velocity X on Opendrive (by meter)
	float velY; // MainVehicle Velocity Y on Opendrive (by meter)
	float velZ; // MainVehicle Velocity Z on Opendrive (by meter)
	float throttle; //MainVehicle throttle
	float brake; //MainVehilce brake;
	float steering; //MainVehicle Steering angle
	int gear; // MainVehicle gear position
	float accelX; // MainVehilce Acceleration X on Opendrive (by meter)
	float accelY; // MainVehilce Acceleration Y on Opendrive (by meter)
	float accelZ; // MainVehilce Acceleration Z on Opendrive (by meter)
	float angVelX; // MainVehilce Angular Velocity X on Opendrive (by meter)
	float angVelY; // MainVehilce Angular Velocity Y on Opendrive (by meter)
	float angVelZ; // MainVehilce Angular Velocity Z on Opendrive (by meter)
	float wheelSpeedFL;  // Speed of front left wheel (by meter/sec)
	float wheelSpeedFR;  // Speed of front right wheel (by meter/sec)
	float wheelSpeedRL;  // Speed of rear left wheel (by meter/sec)
	float wheelSpeedRR;  // Speed of rear right wheel (by meter/sec)
    float engineRpm; // Speed of engine (by r/min)
    float odometer; // odometer in meter
} SimOne_Data_Gps_Bus;

#define SENSOR_TYPE_MAX_LENGHT 64
typedef struct{
    int index;
    int mainVehicle;
    int sensorId;
    char sensorType[SENSOR_TYPE_MAX_LENGHT];
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	int hz;
} SimOne_Data_Sensorconfigurations_Bus;






