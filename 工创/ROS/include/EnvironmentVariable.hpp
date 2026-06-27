// 
// Created By: Huiyong.Men 2019/01/23
// 
// This file defines all environment variables used by the product

/*
Unreal node use single process name
e.g. CZ_SINGLE_UNREAL_PROCESS=Demo
Unreal node will start Demo.exe for all kinds of Unreal task instead of "CybertronVehicle"/"CybertronTraffic"/"CybertronSensor"
*/
#define CZ_SINGLE_UNREAL_PROCESS "CZ_SINGLE_UNREAL_PROCESS"

/*
Accurate Node Timer flag
e.g. CZ_SINGLE_UNREAL_PROCESS=1
NodeTimer will try to make sure time accuracy, taking more CPU resource as well.
*/
#define CZ_ACCURATE_NODE_TIMER "CZ_ACCURATE_NODE_TIMER"

/*
Constraint Node Timer flag
e.g. CZ_CONSTRAINT_NODE_TIMER=1
NodeTimer wait each node time step confirm message
*/
#define CZ_CONSTRAINT_NODE_TIMER "CZ_CONSTRAINT_NODE_TIMER"

/*
Use art scene for simulation node
e.g. CZ_SIM_WITH_ART_SCENE=1
*/
#define CZ_SIM_WITH_ART_SCENE "CZ_SIM_WITH_ART_SCENE"

/*
Print debug log to UE screen
e.g. CZ_PRINT_DEBUG_LOG_TO_SCREEN=1
*/
#define CZ_PRINT_DEBUG_LOG_TO_SCREEN "CZ_PRINT_DEBUG_LOG_TO_SCREEN"

/*
Default record file save path
e.g. CZ_RECORD_PATH=C:/SimOne/Record
*/
#define CZ_RECORD_PATH "CZ_RECORD_PATH"

/*
Start with wine on Linux
e.g. CZ_START_UNREAL_WITH_WINE=1
CZ_START_IMAGE_WITH_WINE has better describablity , use this instead.
*/
#define CZ_START_UNREAL_WITH_WINE "CZ_START_UNREAL_WITH_WINE"
#define CZ_START_IMAGE_WITH_WINE "CZ_START_IMAGE_WITH_WINE"
/*
Start sessions with parallel support
e.g. CZ_PARALLEL_SESSION=1
*/
#define CZ_PARALLEL_SESSION "CZ_PARALLEL_SESSION"

/*
Start task without skipping any minor nodes, such as Sensor and Observer
e.g. CZ_INTACT_RUNS=1
*/
#define CZ_INTACT_RUNS "CZ_INTACT_RUNS"

/*
Log path (Windows and Linux)
e.g. CZ_LOG_PATH=%cd%/log  CZ_LOG_PATH=`pwd`/log
*/
#define CZ_LOG_PATH "CZ_LOG_PATH"

/*
Encryption dog ID
e.g. CODE_METER_ID=3132701
*/
#define CODE_METER_ID "CODE_METER_ID"

#define CZ_STATIC_UNREAL_PROCESS "CZ_STATIC_UNREAL_PROCESS"

#define CZ_NEW_READY "CZ_NEW_READY"