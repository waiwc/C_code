#ifndef PUB_H_
#define PUB_H_

#define DEBUG_CONTROL

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>

#include "Service/SimOneIOStruct.h"
#include "SimOne_Data_GpsPubSubTypes.h"
#include "SimOne_Data_ObstaclePubSubTypes.h"
#include "SimOne_Data.h"
#include "SimOne_Data_Gps.h"
#include "SimOne_Data_Obstacle.h"

#ifdef DEBUG_CONTROL
#include "SimOne_Data_ControlPubSubTypes.h"
#include "SimOne_Data_Control.h"
#endif

#include <chrono>
#include <atomic>

class BenchMarkPublisher;

enum EPub_DataType
{
	EPub_Gps,
	EPub_Obstacle,

#ifdef DEBUG_CONTROL
	EPub_Control
#endif
};

struct Param_TransportDescriptor
{
	int IPv4_Port;
};

class PubListener : public eprosima::fastrtps::PublisherListener
{
public:
	PubListener() {}
	PubListener(BenchMarkPublisher *parent);

	~PubListener(){};

	void onPublicationMatched(eprosima::fastrtps::Publisher *pub, eprosima::fastrtps::rtps::MatchingInfo &info);

	BenchMarkPublisher *mParent;
	int n_matched;
};

class BenchMarkPublisher {
public:
	BenchMarkPublisher();
	virtual ~BenchMarkPublisher();
	bool init(int transport, eprosima::fastrtps::ReliabilityQosPolicyKind kind, const std::string& topicName, int domain,  EPub_DataType dataType, int TCP_PortBase);

	void writeGps(SimOne_Data_Gps *pGps);
	void writeObstacle(SimOne_Data_Obstacle *pObstacle);
#ifdef DEBUG_CONTROL
	void writeControl();
#endif
	void publish(void *p);
private:
	EPub_DataType m_PubDataType;

	eprosima::fastrtps::Participant* mp_participant;
	eprosima::fastrtps::Publisher* mp_publisher;

	PubListener m_pubListener;

	SimOneDDS_Data_GpsPubSubType m_typeGps;
	SimOneDDS_Data_ObstaclePubSubType m_typeObstacle;

#ifdef DEBUG_CONTROL
	SimOneDDS_Data_ControlPubSubType m_typeControl;
#endif
	Param_TransportDescriptor m_transDescriptor;
};

#endif
