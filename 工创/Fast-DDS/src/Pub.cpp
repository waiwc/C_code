#include "Pub.h"
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv6TransportDescriptor.h>
#include <fastrtps/transport/UDPv6TransportDescriptor.h>
#include <fastrtps/Domain.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>

#include <thread>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastdds::rtps;

BenchMarkPublisher::BenchMarkPublisher(): mp_participant(nullptr), mp_publisher(nullptr), m_pubListener(this){}

BenchMarkPublisher::~BenchMarkPublisher()
{
    // if (m_vSamples != nullptr)
    // {
    //     delete(m_vSamples);
    //     m_vSamples = nullptr;
    // }
    // TODO Auto-generated destructor stub
    Domain::removeParticipant(mp_participant);
}

bool BenchMarkPublisher::init(int transport, ReliabilityQosPolicyKind reliabilityKind, const std::string& topicName, int domain, EPub_DataType dataType, int TCP_PortBase)
{
    m_PubDataType = dataType;

    ParticipantAttributes PParam;
    PParam.domainId = domain;
    PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
    PParam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(1, 0);
    PParam.rtps.setName("Participant_pub");

    // PParam.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
    // PParam.rtps.builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
    // PParam.rtps.builtin.discovery_config.static_edp_xml_config("file://Subscriber.xml");


    if (transport == 1) // UDPv4
    {
        PParam.rtps.useBuiltinTransports = true;
    }
    else if (transport == 2) // TCPv4
    {
        PParam.rtps.useBuiltinTransports = false;

        std::shared_ptr<TCPv4TransportDescriptor> descriptor = std::make_shared<TCPv4TransportDescriptor>();
        descriptor->sendBufferSize = 8912896; // 8.5Mb
        descriptor->receiveBufferSize = 8912896; // 8.5Mb
        descriptor->add_listener_port(TCP_PortBase + static_cast<int>(dataType));
        PParam.rtps.userTransports.push_back(descriptor);
    }
    else if (transport == 3) // UDPv6
    {
        //uint32_t kind = LOCATOR_KIND_UDPv6;
    }
    else if (transport == 4) // TCPv6
    {
        PParam.rtps.useBuiltinTransports = false;

        std::shared_ptr<TCPv6TransportDescriptor> descriptor = std::make_shared<TCPv6TransportDescriptor>();
        descriptor->sendBufferSize = 8912896; // 8.5Mb
        descriptor->receiveBufferSize = 8912896; // 8.5Mb
        descriptor->add_listener_port(TCP_PortBase + static_cast<int>(dataType));
        printf("------------descriptor->listener_port: %d\n", descriptor->listening_ports[0]);
        PParam.rtps.userTransports.push_back(descriptor);
    }
    else // SharedMem
    {
        PParam.rtps.useBuiltinTransports = false;
        std::shared_ptr<SharedMemTransportDescriptor> shm_transport = std::make_shared<SharedMemTransportDescriptor>();
        shm_transport->segment_size(2 * 1024 * 1024);
        PParam.rtps.userTransports.push_back(shm_transport);
    }


    




    mp_participant = Domain::createParticipant(PParam);

    if (mp_participant == nullptr)
    {
        return false;
    }

    //CREATE THE PUBLISHER
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;

    //REGISTER THE TYPE
    switch (dataType)
    {
        default:
        case EPub_Gps:
            Wparam.topic.topicDataType = "SimOneDDS_Data_Gps";
            Domain::registerType(mp_participant, &m_typeGps);
            break;
        case EPub_Obstacle:
            Wparam.topic.topicDataType = "SimOneDDS_Data_Obstacle";
            Domain::registerType(mp_participant, &m_typeObstacle);
            break;
    #ifdef DEBUG_CONTROL
        case EPub_Control:
            Wparam.topic.topicDataType = "SimOneDDS_Data_Control";
            Domain::registerType(mp_participant, &m_typeControl);
            break;
    #endif
    }

    Wparam.topic.topicName = topicName;
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    //Wparam.topic.historyQos.depth = 30;
    //Wparam.topic.resourceLimitsQos.max_samples = 50;
    //Wparam.topic.resourceLimitsQos.allocated_samples = 20;
    Wparam.topic.historyQos.depth = 30;
    Wparam.topic.resourceLimitsQos.max_samples = 50;
    Wparam.topic.resourceLimitsQos.allocated_samples = 20;
    Wparam.times.heartbeatPeriod.seconds = 2;
    Wparam.times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
    Wparam.qos.m_reliability.kind = reliabilityKind;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;
    //Wparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    //Wparam.setUserDefinedID(1);
    //Wparam.setEntityID(2);

    mp_publisher = Domain::createPublisher(mp_participant, Wparam, (PublisherListener*)&m_pubListener);
    if (mp_publisher == nullptr)
    {
        return false;
    }

    return true;
}

void BenchMarkPublisher::writeGps(SimOne_Data_Gps *pGps)
{
	SimOneDDS_Data_Gps data_Gps;
    data_Gps.posX(pGps->posX);
    data_Gps.posY(pGps->posY);
    data_Gps.posZ(pGps->posZ);
    data_Gps.oriX(pGps->oriX);
    data_Gps.oriY(pGps->oriY);
    data_Gps.oriZ(pGps->oriZ);
    data_Gps.velX(pGps->velX);
    data_Gps.velY(pGps->velY);
    data_Gps.velZ(pGps->velZ);
    data_Gps.throttle(pGps->throttle);
    data_Gps.brake(pGps->brake);
    data_Gps.steering(pGps->steering);
    data_Gps.gear(pGps->gear);
    data_Gps.accelX(pGps->accelX);
    data_Gps.accelY(pGps->accelY);
    data_Gps.accelZ(pGps->accelZ);
    data_Gps.angVelX(pGps->angVelX);
    data_Gps.angVelY(pGps->angVelY);
    data_Gps.angVelZ(pGps->angVelZ);
    data_Gps.wheelSpeedFL(pGps->wheelSpeedFL);
    data_Gps.wheelSpeedFR(pGps->wheelSpeedFR);
    data_Gps.wheelSpeedRL(pGps->wheelSpeedRL);
    data_Gps.wheelSpeedRR(pGps->wheelSpeedRR);
    data_Gps.engineRpm(pGps->engineRpm);
    data_Gps.odometer(pGps->odometer);

    mp_publisher->write((void*)&data_Gps);
}

void BenchMarkPublisher::writeObstacle(SimOne_Data_Obstacle *pObstacle)
{
    SimOneDDS_Data_Obstacle data_Obstacle;
    data_Obstacle.obstacleSize(pObstacle->obstacleSize);
    for (int i = 0; i < pObstacle->obstacleSize; i++)
    {
        SimOneDDS_Data_Obstacle_Entry obstacle_entry;
        obstacle_entry.id(pObstacle->obstacle[i].id);
        obstacle_entry.viewId(pObstacle->obstacle[i].viewId);
        switch (pObstacle->obstacle[i].type)
        {
        default:
        case ESimOne_Obstacle_Type_Unknown:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Unknown);
            break;
        case ESimOne_Obstacle_Type_Pedestrian:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Pedestrian);
            break;
        case ESimOne_Obstacle_Type_Pole:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Pole);
            break;
        case ESimOne_Obstacle_Type_Car:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Car);
            break;
        case ESimOne_Obstacle_Type_Static:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Static);
            break;
        case ESimOne_Obstacle_Type_Bicycle:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Bicycle);
            break;
        case ESimOne_Obstacle_Type_Fence:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Fence);
            break;
        case ESimOne_Obstacle_Type_RoadMark:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_RoadMark);
            break;
        case ESimOne_Obstacle_Type_TrafficSign:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_TrafficSign);
            break;
        case ESimOne_Obstacle_Type_TrafficLight:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_TrafficLight);
            break;
        case ESimOne_Obstacle_Type_Rider:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Rider);
            break;
        case ESimOne_Obstacle_Type_Truck:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Truck);
            break;
        case ESimOne_Obstacle_Type_Bus:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Bus);
            break;
        case ESimOne_Obstacle_Type_SpecialVehicle:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_SpecialVehicle);
            break;
        case ESimOne_Obstacle_Type_Motorcycle:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Motorcycle);
            break;
        case ESimOne_Obstacle_Type_Dynamic:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_Dynamic);
            break;
        case ESimOne_Obstacle_Type_GuardRail:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_GuardRail);
            break;
        case ESimOne_Obstacle_Type_SpeedLimitSign:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_SpeedLimitSign);
            break;
        case ESimOne_Obstacle_Type_BicycleStatic:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_BicycleStatic);
            break;
        case ESimOne_Obstacle_Type_RoadObstacle:
            obstacle_entry.type(ESimOneDDS_Obstacle_Type_RoadObstacle);
            break;
        }
        obstacle_entry.theta(pObstacle->obstacle[i].theta);
        obstacle_entry.posX(pObstacle->obstacle[i].posX);
        obstacle_entry.posY(pObstacle->obstacle[i].posY);
        obstacle_entry.posZ(pObstacle->obstacle[i].posZ);
        obstacle_entry.oriX(pObstacle->obstacle[i].oriX);
        obstacle_entry.oriY(pObstacle->obstacle[i].oriY);
        obstacle_entry.oriZ(pObstacle->obstacle[i].oriZ);
        obstacle_entry.velX(pObstacle->obstacle[i].velX);
        obstacle_entry.velY(pObstacle->obstacle[i].velY);
        obstacle_entry.velZ(pObstacle->obstacle[i].velZ);
        obstacle_entry.length(pObstacle->obstacle[i].length);
        obstacle_entry.width(pObstacle->obstacle[i].width);
        obstacle_entry.height(pObstacle->obstacle[i].height);
        data_Obstacle.obstacle()[i] = obstacle_entry;
    }

  mp_publisher->write((void *)&data_Obstacle);
}

#ifdef DEBUG_CONTROL
void BenchMarkPublisher::writeControl()
{
	SimOneDDS_Data_Control data_Control;

    data_Control.throttleMode(ESimOneDDS_Throttle_Mode_Percent);
    data_Control.throttle(0.1);
    data_Control.brakeMode(ESimOneDDS_Brake_Mode_Percent);
    data_Control.brake(0.f);
    data_Control.steeringMode(ESimOneDDS_Steering_Mode_Percent);
    data_Control.steering(0.f);
    data_Control.gear(ESimOneDDS_Gear_Mode_Drive);

    mp_publisher->write((void *)&data_Control);
}
#endif

PubListener::PubListener(BenchMarkPublisher* parent): mParent(parent), n_matched(0){}

void PubListener::onPublicationMatched(Publisher* pub, MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        std::cout << "Publisher matched." << std::endl;
        n_matched++;
    }
    else
    {
        std::cout << "Publisher unmatched." << std::endl;
    }
}

void BenchMarkPublisher::publish(void *p)
{
    if (m_pubListener.n_matched > 0)
    {
        switch (m_PubDataType)
        {
            default:
            case EPub_Gps:	
            {
                writeGps(static_cast<SimOne_Data_Gps *>(p));
                break;
            }
            case EPub_Obstacle:
            {
                writeObstacle(static_cast<SimOne_Data_Obstacle *>(p));
                break;
            }
#ifdef DEBUG_CONTROL
            case EPub_Control:
            {
                writeControl();
                break;
            }
#endif
            case 3:
            {
                // m_HelloBig.index(0);
                // mp_publisher->write((void*)&m_HelloBig);
                // std::cout << m_HelloBig.index() << std::endl;
            }
        }
    }
}
