
#ifndef SUB_H_
#define SUB_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>

#include "Service/SimOneIOStruct.h"
#include "SimOne_Data_ControlPubSubTypes.h"
#include "SimOne_Data_Control.h"

class BenchMarkSubscriber;

enum ESub_DataType
{
	ESub_Control
};

class SubListener : public eprosima::fastrtps::SubscriberListener
{
public:
    SubListener() {}
    SubListener(BenchMarkSubscriber *parent);
    ~SubListener(){};

    void onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info);
    void setControl(SimOneDDS_Data_Control &pData);
    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub);

    BenchMarkSubscriber *mParent;

    eprosima::fastrtps::SampleInfo_t m_info;
    int n_matched;
    uint32_t n_samples;
};

class BenchMarkSubscriber
{
    friend class SubListener;

public:
	BenchMarkSubscriber();
	virtual ~BenchMarkSubscriber();

    typedef bool (*funcDrive)(const char*, SimOne_Data_Control *);
    bool init(int transport, eprosima::fastrtps::ReliabilityQosPolicyKind reliabilityKind, const std::string &topicName, int domain, ESub_DataType dataType, const std::string &TCPv4_Ip, int TCP_PortBase);
    void setDriveTask(funcDrive p);
    void run();

private:
    eprosima::fastrtps::Participant* mp_participant;
    eprosima::fastrtps::Subscriber* mp_subscriber;
    SubListener m_subListener;
    funcDrive pDrive;
    ESub_DataType m_SubDataType;

    SimOneDDS_Data_ControlPubSubType m_typeControl;
};

#endif
