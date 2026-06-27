#include "Sub.h"
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/transport/TCPv6TransportDescriptor.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastrtps/Domain.h>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastdds::rtps;

SubListener::SubListener(BenchMarkSubscriber* parent): mParent(parent), n_matched(0), n_samples(0){}

void SubListener::onSubscriptionMatched(Subscriber* sub, MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void SubListener::setControl(SimOneDDS_Data_Control &data)
{
    std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();

    pCtrl->throttleMode = static_cast<ESimOne_Throttle_Mode>(data.throttleMode());
    pCtrl->throttle = data.throttle();
    pCtrl->brakeMode = static_cast<ESimOne_Brake_Mode>(data.brakeMode());
    pCtrl->brake = data.brake();
    pCtrl->steeringMode = static_cast<ESimOne_Steering_Mode>(data.steeringMode());
    pCtrl->steering = data.steering(); 
    pCtrl->gear = static_cast<ESimOne_Gear_Mode>(data.gear());

    std::cout << "------------ Ctl ------------" << std::endl;
    std::cout << "gear = " << pCtrl->gear << std::endl;
    std::cout << "throttleMode = " << pCtrl->throttleMode << std::endl;
    std::cout << "throttle = " << pCtrl->throttle << std::endl;
    std::cout << "steeringMode = " << pCtrl->steeringMode << std::endl;
    std::cout << "steering = " << pCtrl->steering << std::endl;
    std::cout << "brakeMode = " << pCtrl->brakeMode << std::endl;
    std::cout << "brake = " << pCtrl->brake << std::endl;

    if (!mParent->pDrive("0", pCtrl.get()))
    {
        std::cout << "[setControl] SetDrive Failed!" << std::endl;
    }
}

void SubListener::onNewDataMessage(Subscriber* sub)
{
    switch (mParent->m_SubDataType)
    {
        default:
        // case 0:
        // {

	       //  SimOneDDS_Data_Gps data_Gps;

        //     if (sub->takeNextData((void*)&data_Gps, &m_info))
        //     {
        //         if (m_info.sampleKind == ALIVE)
        //         {
        //             // mParent->m_Hello.index(mParent->m_Hello.index() + 1);
        //             // mParent->mp_publisher->write((void*)&mParent->m_Hello);
        //         }
        //     }
        // }
        // break;
        case ESub_Control:
        {
	        SimOneDDS_Data_Control data_Control;
            if (sub->takeNextData((void*)&data_Control, &m_info))
            {
                if (m_info.sampleKind == ALIVE)
                {
                    setControl(data_Control);
                    // mParent->m_HelloSmall.index(mParent->m_HelloSmall.index() + 1);
                    // mParent->mp_publisher->write((void*)&mParent->m_HelloSmall);
                }
            }
        }
        break;
        case 2:
        {
            // if (sub->takeNextData((void*)&mParent->m_HelloMedium, &m_info))
            // {
            //     if (m_info.sampleKind == ALIVE)
            //     {
            //         mParent->m_HelloMedium.index(mParent->m_HelloMedium.index() + 1);
            //         mParent->mp_publisher->write((void*)&mParent->m_HelloMedium);
            //     }
            // }
        }
        break;
        case 3:
        {
            // if (sub->takeNextData((void*)&mParent->m_HelloBig, &m_info))
            // {
            //     if (m_info.sampleKind == ALIVE)
            //     {
            //         mParent->m_HelloBig.index(mParent->m_HelloBig.index() + 1);
            //         // mParent->mp_publisher->write((void*)&mParent->m_HelloBig);
            //     }
            // }
        }
        break;
    }
}

BenchMarkSubscriber::BenchMarkSubscriber(): mp_participant(nullptr), mp_subscriber(nullptr), m_subListener(this){}

BenchMarkSubscriber::~BenchMarkSubscriber()
{
    Domain::removeParticipant(mp_participant);
}

bool BenchMarkSubscriber::init(int transport, ReliabilityQosPolicyKind reliabilityKind, const std::string& topicName, int domain, ESub_DataType dataType,
    const std::string& TCPv4_Ip, int TCP_PortBase)
{
    m_SubDataType = dataType;

    ParticipantAttributes PParam;
    PParam.domainId = domain;
    PParam.rtps.builtin.discovery_config.leaseDuration = c_TimeInfinite;
    PParam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(1, 0);
    PParam.rtps.setName("Participant_sub");

    if (transport == 1)
    {
        PParam.rtps.useBuiltinTransports = true;

        // Create a descriptor for the new transport.
        // auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        // udp_transport->sendBufferSize = 8912896; // 8.5Mb   9216;
        // udp_transport->receiveBufferSize = 8912896; // 8.5Mb    9216;
        // udp_transport->non_blocking_send = true;
        // PParam.rtps.userTransports.push_back(udp_transport);
    }
    else if (transport == 2)
    {
        PParam.rtps.useBuiltinTransports = false;

        Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
        IPLocator::setIPv4(initial_peer_locator, TCPv4_Ip.c_str()); // IPLocator::setIPv4(initial_peer_locator, "::1");
        initial_peer_locator.port = TCP_PortBase + dataType;
        printf("2--------initial_peer_locator.port: %d\n", initial_peer_locator.port);
        PParam.rtps.builtin.initialPeersList.push_back(initial_peer_locator); // Publisher's channel

        PParam.rtps.useBuiltinTransports = false;
        std::shared_ptr<TCPv4TransportDescriptor> descriptor = std::make_shared<TCPv4TransportDescriptor>();
        descriptor->sendBufferSize = 8912896; // 8.5Mb
        descriptor->receiveBufferSize = 8912896; // 8.5Mb
        PParam.rtps.userTransports.push_back(descriptor);
    }
    else if (transport == 3)
    {
        //uint32_t kind = LOCATOR_KIND_UDPv6;
    }
    else if (transport == 4)
    {
        PParam.rtps.useBuiltinTransports = false;

        Locator_t initial_peer_locator;
        initial_peer_locator.kind = LOCATOR_KIND_TCPv6;
        IPLocator::setIPv6(initial_peer_locator, TCPv4_Ip.c_str()); // IPLocator::setIPv6(initial_peer_locator, "::1");
        initial_peer_locator.port = TCP_PortBase + dataType;
        PParam.rtps.builtin.initialPeersList.push_back(initial_peer_locator); // Publisher's channel

        PParam.rtps.useBuiltinTransports = false;
        std::shared_ptr<TCPv6TransportDescriptor> descriptor = std::make_shared<TCPv6TransportDescriptor>();
        descriptor->sendBufferSize = 8912896; // 8.5Mb
        descriptor->receiveBufferSize = 8912896; // 8.5Mb
        PParam.rtps.userTransports.push_back(descriptor);
    }
    else
    {
        PParam.rtps.useBuiltinTransports = false;

        std::shared_ptr<SharedMemTransportDescriptor> sm_transport = std::make_shared<SharedMemTransportDescriptor>();
        sm_transport->segment_size(2 * 1024 * 1024);
        PParam.rtps.userTransports.push_back(sm_transport);
    }

    mp_participant = Domain::createParticipant(PParam);
    if (mp_participant == nullptr)
    {
        return false;
    }

    //CREATE THE SUBSCRIBER
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;

    //REGISTER THE TYPE
    switch (dataType)
    {
        default:
        case ESub_Control:
            Rparam.topic.topicDataType = "SimOneDDS_Data_Control";
            Domain::registerType(mp_participant, &m_typeControl);
            break;
    }

    Rparam.topic.topicName = topicName;
    Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = 30;
    Rparam.topic.resourceLimitsQos.max_samples = 50;
    Rparam.topic.resourceLimitsQos.allocated_samples = 20;
    Rparam.qos.m_reliability.kind = reliabilityKind;
    Rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, (SubscriberListener*)&m_subListener);

    if (mp_subscriber == nullptr)
    {
        return false;
    }

    return true;
}

void BenchMarkSubscriber::setDriveTask(funcDrive p)
{
    pDrive = p;
}

void BenchMarkSubscriber::run()
{
    std::cout << "Subscriber running..." << std::endl;
    std::cin.ignore();
}
