#include "TaskCyberRoutingReq.hpp"

CW_RoutingReq::CW_RoutingReq(std::string channel_name):mCyberRoutingReq(channel_name.c_str()){}
CW_RoutingReq::~CW_RoutingReq(){}

bool CW_RoutingReq::set_routing_req(unsigned& point_size, CyberWriterRoutingReq::WayPoint*point_list)
{
    std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
    if (!SimOneAPI::GetWayPoints("0", pWayPoints.get()))
    {
        std::cout << "[CW_RoutingReq] GetWayPoints Failed!" << std::endl;
        return false;
    }

    point_size = pWayPoints->wayPointsSize;
    for (int i=0; i<point_size; i++)
    {
        SimOne_Data_WayPoints_Entry& point = pWayPoints->wayPoints[i];
        point_list[i].x = point.posX;
        point_list[i].y = point.posY;
        point_list[i].headingx = point.heading_x;
        point_list[i].headingy = point.heading_y;
        point_list[i].headingz = point.heading_z;
        point_list[i].headingw = point.heading_w;
        point_list[i].speed = 0;
        point_list[i].accel = 0;
        point_list[i].time_interval = 0;
    }

    return true;
}

void CW_RoutingReq::task_routing_req(bool &routing_rsp)
{
    static uint64_t  frame = 0;

    mCyberRoutingReq.mWayPointSize = 0;
    memset(mCyberRoutingReq.mWayPointList, 0, sizeof(CyberWriterRoutingReq::WayPoint) * WAYPOINT_MAX_SIZE);

    if (!routing_rsp && set_routing_req(mCyberRoutingReq.mWayPointSize, mCyberRoutingReq.mWayPointList))
    {
        // std::cout << "-------------- WayPointSize: " << mCyberRoutingReq.mWayPointSize << std::endl;
        // for (int i=0; i<mCyberRoutingReq.mWayPointSize; i++)
        // {
        //     std::cout << "  mWayPointList[" << i << "].x: "<< mCyberRoutingReq.mWayPointList[i].x << std::endl;
        //     std::cout << "  mWayPointList[" << i << "].y: "<< mCyberRoutingReq.mWayPointList[i].y << std::endl;
        //     std::cout << "  mWayPointList[" << i << "].headingx: "<< mCyberRoutingReq.mWayPointList[i].headingx << std::endl;
        //     std::cout << "  mWayPointList[" << i << "].headingy: "<< mCyberRoutingReq.mWayPointList[i].headingy << std::endl;
        //     std::cout << "  mWayPointList[" << i << "].headingz: "<< mCyberRoutingReq.mWayPointList[i].headingz << std::endl;
        //     std::cout << "  mWayPointList[" << i << "].headingw: "<< mCyberRoutingReq.mWayPointList[i].headingw << std::endl;
        // }
        mCyberRoutingReq.switchControlIndex = mCyberRoutingReq.mWayPointSize - 1;

        mCyberRoutingReq.publish(frame++);
        mCyberRoutingRes.spinOnce();
        if (mCyberRoutingRes.haveMessage) {
            std::cout << "[CW_RoutingReq] Got route response!" << std::endl;;
            routing_rsp = true;
        }
        std::cout << "[CW_RoutingReq] Wait route response ..." << std::endl;
    }
}
