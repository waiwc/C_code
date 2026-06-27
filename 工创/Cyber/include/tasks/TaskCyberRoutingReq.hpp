#ifndef TASKCYBERROUTINGREQ_HPP_
#define TASKCYBERROUTINGREQ_HPP_

#include <iostream>
#include <string.h>
#include <math.h>
#include "CyberWriterRoutingReq.hpp"
#include "CyberReaderRoutingResponse.hpp"
#include "SimOnePNCAPI.h"

#include "SimOneSensorAPI.h"

class CW_RoutingReq
{
public:
    CW_RoutingReq(std::string channel_name);
    ~CW_RoutingReq();
    bool set_routing_req(unsigned &point_size, CyberWriterRoutingReq::WayPoint *point_list);
    void task_routing_req(bool & routing_rsp);

private:
    CyberWriterRoutingReq mCyberRoutingReq;
    CyberReaderRoutingResponse mCyberRoutingRes;
};

#endif
