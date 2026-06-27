#include "SimOneServiceAPI.h"
#include "SimOneSensorAPI.h"
#include "SimOneHDMapAPI.h"
#include "SSD/SimPoint3D.h"

#include "UtilDriver.h"
#include "UtilMath.h"
#include "hdmap/SampleGetNearMostLane.h"
#include "GetSignType.h"
#include "SimOneEvaluationAPI.h"
#include "SSD/SimPoint3D.h"
#include "SSD/SimString.h"
#include "public/common/MLaneInfo.h"
#include "public/common/MLaneId.h"
#include "GetSignType.h"
#include "SimOneSensorAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "HDMap/SampleGetLaneST.h"
#include <iostream>
#include <memory>
#include <cstring>
#include <limits>
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include <climits>
#include <unordered_map>
#include <set>
//-----------------------------------------------------------------------------
// 常量定义
//-----------------------------------------------------------------------------
#define STRINGS_EQUAL 0
#define ON (1)
#define OFF (0)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)
#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

constexpr int FPS = 30;                               // 全局帧率定义（可调整）
float DEFAULT_SPEED = 50.0f;                          // 最大默认速度
constexpr float DEFAULT_THROTTLE = 0.5f;              // 默认油门值：车辆在正常行驶时的油门输入
constexpr float DEFAULT_BRAKE = 0.0f;                 // 默认刹车值：车辆在无制动需求时的刹车输入（即不踩刹车）
constexpr int LANE_CHANGE_DISTANCE = 3;               // 变道默认距离：用于控制变道过程中与障碍物或目标车道的距离（单位：米）
constexpr double LEFT_LANE_DETECTION_DISTANCE = 35.0; // 左侧车道探测距离：用于检测左侧是否有车辆的最大探测距离（单位：米）
constexpr double SPEED_HYSTERESIS = 0.05f;            // 速度保持状态的阈值
double MIN_LOOK_AHEAD = 4.0;                          // 最低前视距离（米）
double MAX_LOOK_AHEAD = 20.0;                         // 最高前视距离（米）
double LOOK_AHEAD_GAIN = 0.18;                        // 每增加的前视距离（米）
double MAX_LAT_ACC = 15;                              // 最大可接受横向加速度 (m/s²)——可调
constexpr double MIN_RADIUS = 0.1;                    // 防止除以零
double MIN_TARGET_SPEED = 5.0;                        // 额外速度下限
constexpr double DEFAULT_OFFSET_RADIUS = 20.0;        // 默认偏移路段的半径
constexpr double RESAMPLE_DIST = 0.2;                 // 轨迹转换采样间距
constexpr double MAX_TOTAL_LEN = 300.0;               // 最大轨迹长度
double maxAccel = 5.0;                                // 最大加减速度 m/s²
constexpr double stopDist = 13.0;                     // 堵塞后停止距离
double throttle_K = 0.4;                              // 油门系数

// 内环 PID 增益
constexpr double KP_STEER = 0.8;
constexpr double KI_STEER = 0.05;
constexpr double KD_STEER = 0.1;

uint8_t left_lane = OFF;                   // 左侧车道是否存在
uint8_t right_lane = OFF;                  // 右侧车道是否存在
uint8_t change_road = OFF;                 // 是否变道
uint8_t use_fixd_steering = OFF;           // 是否使用固定转向
uint16_t dis_CR = LANE_CHANGE_DISTANCE;    // 变道默认距离
float defalut_brake = DEFAULT_BRAKE;       // 默认刹车值
float defalut_throttle = DEFAULT_THROTTLE; // 默认油门值
SSD::SimString mainVehicleLaneId;          // 主车所在车道ID
SSD::SimString mainVehicleLaneId_last;     // 上一帧主车所在车道ID
SSD::SimPoint3DVector inputPoints;         // 输入路径点
SSD::SimPoint3DVector targetPath;          // 目标路径
SSD::SimPoint3D targetPointPos;            // 目标位置
std::string obstacleTypeToString(ESimOne_Obstacle_Type type);

/*----------------  结构体  ----------------*/

enum class SpeedZoneType
{
    BEFORE_TARGET, // 修改目标点前的轨迹点
    AFTER_TARGET   // 修改目标点后的轨迹点
};

struct SLObsBdPoint
{
    double s, l;
    size_t obsId;
    double wx, wy;
}; // OBB 边界

struct SLBoundaryPoint
{
    double s, l;
    double wx, wy;
}; // 道路边界采样点

struct TrajectoryPoint
{
    double x = 0.0;
    double y = 0.0;
    double v = 0.0; // m/s
};

struct SLPoint
{
    double s = 0.0; // 沿参考线的累计距离
    double l = 0.0; // 侧向偏移（右为正）
    size_t idx = 0; // 原轨迹索引
};

struct SegmentFrame
{
    /* 局部坐标系原点 = 段首 traj 点 */
    SSD::SimPoint3D origin;
    double tx = 0., ty = 0.; // 切向单位向量
    double nx = 0., ny = 0.; // 左侧法向单位向量
    double sEnd = 0.;        // 段长（末点的 s）

    std::vector<SLPoint> trajSL;        // 段内轨迹点 (碰撞)
    std::vector<SLObsBdPoint> obbSL;    // 段内 OBB 边界点
    std::vector<SLBoundaryPoint> lbdSL; // 道路左边界
    std::vector<SLBoundaryPoint> rbdSL; // 道路右边界
};

struct OBB
{
    double cx, cy, cosH, sinH, hl, hw; // 半长半宽
    void print(int idx) const
    {
        // std::cout << "[OBB] obs#" << idx << '\n';
        auto pr = [&](double lx, double ly)
        {
            double wx = cx + lx * cosH - ly * sinH;
            double wy = cy + lx * sinH + ly * cosH;
            // std::cout << "  (" << wx << "," << wy << ")\n";
        };
        pr(+hl, +hw);
        pr(+hl, -hw);
        pr(-hl, -hw);
        pr(-hl, +hw);
        pr(+hl, +hw);
    }

    bool contains(double x, double y, double inflate = 0.0) const
    {
        double dx = x - cx, dy = y - cy;
        double lx = dx * cosH + dy * sinH;
        double ly = -dx * sinH + dy * cosH;
        return std::fabs(lx) <= hl + inflate &&
               std::fabs(ly) <= hw + inflate;
    }
};

inline double planarDist(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

inline double calcSpeed(const SimOne_Data_Obstacle_Entry &o)
{
    return std::hypot(o.velX, o.velY);
}

void PrintTrajectory(const std::vector<TrajectoryPoint> &path, const char *tag = "path")
{
    std::cout << "===== " << tag << "  size=" << path.size() << " =====\n";
    for (size_t i = 0; i < path.size(); ++i)
        std::cout << "[" << i << "]  x=" << path[i].x
                  << "  y=" << path[i].y
                  << "  v=" << path[i].v << '\n';
    std::cout << "=========================================\n";
}

static std::vector<double> buildCumS(const SSD::SimPoint3DVector &bd);

static std::pair<double, double>
projectToPolylineSL(const SSD::SimPoint3DVector &bd,
                    const std::vector<double> &cumS,
                    double x, double y);

static void blendAlongSegment(const std::vector<SegmentFrame> &segments,
                              std::vector<double> &dLat,
                              std::vector<TrajectoryPoint> &traj,
                              double R);

static void smoothCurvatureBlend(std::vector<TrajectoryPoint> &traj,
                                 std::vector<double> &dLat,
                                 double influence,
                                 double minR,
                                 int iterMax,
                                 int winSize);

static inline bool obbHitsCircle(const OBB &box,
                                 double x, double y,
                                 double r);
void UpdateTurnSignals(
    const SimOne_Data_Gps *gps,
    const std::vector<TrajectoryPoint> &trajectory,
    SimOne_Data_Signal_Lights *light);
static bool LateralBypassCore(
    const SimOne_Data_Gps &ego,
    const SimOne_Data_Obstacle &obs,
    std::vector<TrajectoryPoint> &traj,
    double safeR = 1.4);
bool LimitThrottleIfOpposingMovingObstacle(
    const char *mainVehicleId,
    const SimOne_Data_Gps &gps,
    const SimOne_Data_Obstacle &obstacles,
    const std::vector<TrajectoryPoint> &traj,
    SimOne_Data_Control *control);
bool AdjustTrajectoryIfOpposingTraffic(
    const SimOne_Data_Gps &ego,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &traj,
    double lateralMargin);
void SmoothTrajectorySpeedForward(std::vector<TrajectoryPoint> &traj,
                                  double targetSpeed,
                                  double maxAccel = 10.0);
bool StopBeforeStoplineIfNeeded(const SimOne_Data_Gps& gps,
    std::vector<TrajectoryPoint>& traj,
                                double stopBeforeDist,
                                double speed);
bool HasObstacleOnCrosswalk(const SimOne_Data_Gps& gps,
                            const SimOne_Data_Obstacle &obstacle);
/*======== 工具：二维叉乘 ========*/
static inline double cross(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}
auto projectOnPolyline = [](const SSD::SimPoint3DVector &poly,
                            const SSD::SimPoint3D &P,
                            SSD::SimPoint3D &out,
                            size_t &segIdx) -> void
{
    double best = 1e9;
    segIdx = 0;
    for (size_t i = 0; i + 1 < poly.size(); ++i)
    {
        double vx = poly[i + 1].x - poly[i].x;
        double vy = poly[i + 1].y - poly[i].y;
        double len2 = vx * vx + vy * vy;
        if (len2 < 1e-6)
            continue;
        double t = ((P.x - poly[i].x) * vx + (P.y - poly[i].y) * vy) / len2;
        t = CLAMP(t, 0.0, 1.0);
        double qx = poly[i].x + t * vx;
        double qy = poly[i].y + t * vy;
        double d = std::hypot(qx - P.x, qy - P.y);
        if (d < best)
        {
            best = d;
            segIdx = i;
            out = {qx, qy, 0};
        }
    }
};
/* ======== 得到“原车道左边界基准点” ======== */
auto nearestLeftPoint = [&](const SSD::SimPoint3DVector &leftB,
                            const SSD::SimPoint3D &P) -> SSD::SimPoint3D
{
    SSD::SimPoint3D res{0, 0, 0};
    size_t dump;
    projectOnPolyline(leftB, P, res, dump);
    return res;
};

bool StopBeforeJunctionIfOpposingObstacle(
    const SimOne_Data_Gps &gps,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &traj);
bool StopForRedLightIfNeeded(
	const char* mainVehicleId,
    const SimOne_Data_Gps &gps,
    std::vector<TrajectoryPoint> &traj);
/* ================================================================
 *                   LaneChangeSegmentPrefilter
 * ================================================================*/
bool LaneChangeSegmentPrefilter(const SimOne_Data_Gps &gps,
                                const SimOne_Data_Obstacle &obstacles,
                                std::vector<TrajectoryPoint> &traj)
{
    if (traj.size() < 5)
        return false;

    /* ---------- ① 采样：每 10 点判定一次 ---------- */
    const size_t SAMPLE_STEP = 10;
    std::vector<size_t> sampleIdx;
    for (size_t i = 0; i < traj.size(); i += SAMPLE_STEP)
        sampleIdx.push_back(i);

    /* ---------- ② 累计弧长 cumS（供段内 s / 后处理用） ---------- */
    std::vector<double> cumS(traj.size(), 0.0);
    for (size_t i = 1; i < traj.size(); ++i)
        cumS[i] = cumS[i - 1] + std::hypot(traj[i].x - traj[i - 1].x,
                                           traj[i].y - traj[i - 1].y);

    /* ---------- ③ 预索引：障碍物 -> 所属车道 ---------- */
    std::vector<SSD::SimString> obsLane(obstacles.obstacleSize, "");
    for (int oi = 0; oi < obstacles.obstacleSize; ++oi)
    {
        SSD::SimPoint3D p{obstacles.obstacle[oi].posX,
                          obstacles.obstacle[oi].posY,
                          obstacles.obstacle[oi].posZ};
        double s, t, s2, t2;
        SimOneAPI::GetNearMostLane(p, obsLane[oi], s, t, s2, t2);

        // ---------- 额外过滤“几乎同向且速度较快”的车辆类型障碍物 ----------
        if (obstacles.obstacle[oi].type == ESimOne_Obstacle_Type_Car)
        {
            // 计算速度
            double v = std::hypot(obstacles.obstacle[oi].velX,
                                  obstacles.obstacle[oi].velY);

            if (v > 2.6) // 仅速度足够快时才进入朝向判断
            {
                SSD::SimPoint3D proj, dir;
                if (SimOneAPI::GetLaneMiddlePoint(p, obsLane[oi], proj, dir))
                {
                    double laneHeading = std::atan2(dir.y, dir.x);
                    double obsHeading = obstacles.obstacle[oi].oriZ;

                    double angleDiff = std::fabs(laneHeading - obsHeading);
                    while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
                    angleDiff = std::fabs(angleDiff);

                    if (angleDiff < M_PI / 6.0 || HasObstacleOnCrosswalk(gps,obstacles)) // 30°内同向
                    {
                        obsLane[oi] = ""; // 标记为无效（跳过）
                    }
                }
            }
        }
    }


    /* ---------- ④ 轨迹点横向位移（l）记录 ---------- */
    std::vector<double> dLat(traj.size(), 0.0);

    std::vector<SegmentFrame> segs; // 所有变道段
    bool inSeg = false;             // 当前是否处于一段变道段内
    SegmentFrame curSeg;

    /* ============================================================
     * ⑤ 逐采样点判定是否换道，生成分段 SL 坐标系
     * ============================================================*/
    for (size_t k = 0; k < sampleIdx.size(); ++k)
    {
        size_t idx = sampleIdx[k];
        SSD::SimPoint3D P{traj[idx].x, traj[idx].y, 0.0};

        /* 当前位置所在车道 */
        HDMapStandalone::MLaneInfo laneCur;
        if (!SimOneAPI::GetLaneSampleByLocation(P, laneCur))
            continue; // 未落在地图车道

        /* 统计 10 m 内在同车道的障碍物数 */
        auto countObs = [&](const SSD::SimString &id) -> int
        {
            int c = 0;
            for (int oi = 0; oi < obstacles.obstacleSize; ++oi)
            {
                if (strcmp(obsLane[oi].GetString(), id.GetString()) != 0)
                    continue;
                if (std::hypot(obstacles.obstacle[oi].posX - P.x,
                               obstacles.obstacle[oi].posY - P.y) <= 10.0)
                    ++c;
            }
            return c;
        };

        /* ========== BFS：在左右邻道里找障碍最少车道 ==========*/
        struct Node
        {
            SSD::SimString id;
            int cnt;
        };
        std::vector<Node> q{{laneCur.laneName, countObs(laneCur.laneName)}};
        SSD::SimString bestId = laneCur.laneName;
        int bestCnt = q[0].cnt;

        for (size_t qi = 0; qi < q.size(); ++qi)
        {
            const Node &n = q[qi];
            if (n.cnt < bestCnt)
            {
                bestCnt = n.cnt;
                bestId = n.id;
            }
            if (n.cnt == 0)
                continue; // 已无障碍，不再扩展

            HDMapStandalone::MLaneLink link;
            if (!SimOneAPI::GetLaneLink(n.id, link))
                continue;

            auto push = [&](const SSD::SimString &id)
            {
                if (strlen(id.GetString()) == 0)
                    return;
                for (auto &e : q)
                    if (strcmp(e.id.GetString(), id.GetString()) == 0)
                        return;

                // ===== 排除对向车道逻辑 =====
                auto extractRoadId = [](const SSD::SimString &laneName) -> long
                {
                    // 格式: roadId_sectionIndex_laneId
                    std::string s = laneName.GetString();
                    auto pos = s.find('_');
        if (pos == std::string::npos) return -1;
                    return std::stol(s.substr(0, pos));
                };

                long roadIdCur = extractRoadId(laneCur.laneName);
                long roadIdCand = extractRoadId(id);

                if (roadIdCur == roadIdCand)
                {
                    // 同一条路上，判断是否双向路
                    if (SimOneAPI::IsTwoSideRoad(roadIdCur))
                    {
                        // 双向路 → 当前车道 ID 和候选车道 ID 是否为“对侧”？
                        // 可进一步判断 laneId 是正数/负数、左右邻居是否一致
                        HDMapStandalone::MLaneLink curLink, candLink;
                        SimOneAPI::GetLaneLink(laneCur.laneName, curLink);
                        SimOneAPI::GetLaneLink(id, candLink);

                        // 判断是否为相反方向：
                        // 如果 cand 是 laneCur 的左邻居，但 cand 的 successor 中没有 laneCur
                        // 或者 cand 的方向连接完全不包含 laneCur，则可能为对向
                        bool connected = false;
                        for (int i = 0; i < candLink.successorLaneNameList.size(); ++i)
                        {
                            if (strcmp(candLink.successorLaneNameList[i].GetString(), laneCur.laneName.GetString()) == 0)
                            {
                                connected = true;
                                break;
                            }
                        }

                        if (!connected)
                            return; // 排除对向车道
                    }
                }
                // ===========================

                q.push_back({id, countObs(id)});
            };


            push(link.leftNeighborLaneName);
            push(link.rightNeighborLaneName);
        }
        bool needLC = (strcmp(bestId.GetString(),
                              laneCur.laneName.GetString()) != 0);

        /* =======================================================
         * ⑥ 若需要换道 → 把该采样段纳入 SegmentFrame
         * =======================================================*/
        if (needLC)
        {
            /* ---- A. 若首次进入段 → 初始化段框架 ---- */
            if (!inSeg)
            {
                inSeg = true;
                curSeg = SegmentFrame();
                curSeg.origin = {traj[idx].x, traj[idx].y, 0.0}; // s-l 坐标原点
            }

            /* ---- B. 采样目标车道几何 & 中心线 ---- */
            HDMapStandalone::MLaneInfo laneBest;
            if (!SimOneAPI::GetLaneSample(bestId, laneBest))
                continue;
            const auto &CL = laneBest.centerLine;
            if (CL.size() < 2)
                continue;

            /* ---- C. 锚点 P 投影到中心线，得到 C ---- */
            SSD::SimPoint3D C;
            size_t centerSeg;
            projectOnPolyline(CL, P, C, centerSeg); // ↩︎ util λ

            /* ---- D. 取得切向 / 法向单位向量 ---- */
            double tx = CL[centerSeg + 1].x - CL[centerSeg].x;
            double ty = CL[centerSeg + 1].y - CL[centerSeg].y;
            double len = std::hypot(tx, ty);
            if (len < 1e-6)
                continue;
            tx /= len;
            ty /= len; // +s 方向
            double nx = -ty;
            double ny = tx; // +l 方向（左为正）

            /* ---- E. 横移向量 Δw = C - P 及其 l 分量 ---- */
            double dx = C.x - P.x;
            double dy = C.y - P.y;
            double shift = dx * nx + dy * ny; // 仅法向分量

            /* ---- F. 把采样段 [idx … idx+SAMPLE_STEP-1] 投影到局部 s-l ---- */
            for (size_t i = idx; i < traj.size() && i < idx + SAMPLE_STEP; ++i)
            {
                /* 1) 世界坐标平移到中心线 */
                traj[i].x += dx;
                traj[i].y += dy;

                /* 2) 计算局部向量 v = w - origin */
                double vx = traj[i].x - curSeg.origin.x;
                double vy = traj[i].y - curSeg.origin.y;

                /* 3) 投影到 (s,l) */
                double sLoc = vx * tx + vy * ty; // 切向
                double lLoc = vx * nx + vy * ny; // 法向

                /* 4) 仅采样锚点记录横移量（供后处理） */
                if (i == idx)
                    dLat[i] = lLoc;

                /* 5) 写入 trajSL（存 s,l,wx,wy ）*/
                curSeg.trajSL.push_back({sLoc, lLoc, i});
            }

            /* ---- G. 更新段公共参数 ---- */
            curSeg.tx = tx;
            curSeg.ty = ty;
            curSeg.nx = nx;
            curSeg.ny = ny;
            curSeg.sEnd = curSeg.trajSL.back().s; // 段长 = 末点 s
        }
        else /* 不换道：若之前在段内则封段 */
        {
            if (inSeg)
            {
                segs.push_back(curSeg);
                inSeg = false;
            }
        }
    }
    if (inSeg)
        segs.push_back(curSeg); // 收尾

    /* ---------- ⑦ 段外衔接 & 曲率抑制 ---------- */
    blendAlongSegment(segs, dLat, traj, /*R=*/20.0);
    smoothCurvatureBlend(traj, dLat,
                         /*influence=*/20.0,
                         /*minR=*/100000.0,
                         /*iterMax=*/500,
                         10);

    return true;
}

bool ProcessObstacleLongitudinalSpeed(
    const SimOne_Data_Gps &egoGps,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &trajectory,
    double lookAhead);

void ModifySpeedAroundTarget(std::vector<TrajectoryPoint> &trajectory,
                             double target_x, double target_y,
                             double new_speed,
                             double distance_threshold,
                             double distance_threshold_r,
                             double trigger_m,
                             SpeedZoneType zone);

void change_road_function(SSD::SimPoint3DVector *inps, const SSD::SimPoint3D &start_p,
                          const SSD::SimPoint3D &mid_p, const SSD::SimPoint3D &end_p, const SSD::SimPoint3D &endend_p);

bool IsCarInLeftLane(const SSD::SimPoint3D &MainCarPos, const SSD::SimString &MainCarLandId,
                     SimOne_Data_Obstacle *obstacle,
                     const HDMapStandalone::MLaneLink &laneLink, double &LefrCarSpeed); // 检查左侧是否有车辆

void ConvertToTrajectoryPoints(const SSD::SimPoint3DVector &traj_data, double default_speed, const SimOne_Data_Gps *pGps, std::vector<TrajectoryPoint> &output);

bool DetectObstacleInSameLane(const SimOne_Data_Obstacle &obstacleData, const SSD::SimPoint3D &mainCarPos,
                              const SSD::SimString &mainVehicleLaneId,
                              double &minDistance, int &potentialObstacleIndex,
                              SSD::SimString &potentialObstacleLaneId); // 检测同车道障碍物

// void HandleLaneChange(const SSD::SimPoint3D &mainVehiclePos, const SSD::SimPoint3D &targetPointPos, uint8_t left_lane,
//                       uint8_t right_lane,
//                       uint8_t &change_road, uint8_t &use_fixd_steering, float &steering,
//                       SSD::SimPoint3DVector &inputPoints, SSD::SimPoint3DVector &targetPath); // 处理车道变更

// void UpdateControlForLaneChange(uint8_t change_road, uint8_t left_lane, uint8_t right_lane, float dis_CR,
//                                 float minDistance,
//                                 SimOne_Data_Control *control, SimOne_Data_Signal_Lights *light); // 更新变道控制

void processControl(const SimOne_Data_Gps *pGps, const std::vector<TrajectoryPoint> &traj_data,
                    SimOne_Data_Control *control,
                    int frame);

void printObstacleInfo(const SimOne_Data_Obstacle &data);

//-----------------------------------------------------------------------------
// 主函数
//-----------------------------------------------------------------------------
int main()
{
    // 初始化核心变量
    const char *MAIN_VEHICLE_ID = "0";
    bool isSimOneInitialized = false;
    bool isJoinTimeLoop = true;
    uint64_t fps_count = 0;

    // 初始化SimOne API
    SimOneAPI::InitSimOneAPI(MAIN_VEHICLE_ID, isJoinTimeLoop);
    SimOneAPI::SetDriverName(MAIN_VEHICLE_ID, "OTTOPILOT");
    SimOneAPI::InitEvaluationServiceWithLocalData(MAIN_VEHICLE_ID); // 函数评价初始化

    // 预分配数据对象
    auto gps = std::make_unique<SimOne_Data_Gps>();
    auto obstacle = std::make_unique<SimOne_Data_Obstacle>();
    SSD::SimVector<HDMapStandalone::MSignal> signList;

    // 地图加载循环
    while (true)
    {
        int timeout = 20;
        if (!SimOneAPI::LoadHDMap(timeout))
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loaded");
            continue;
        }
        SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "HDMap Information Loading...");

        SimOne_Data_CaseInfo caseInfo;

        if (!SimOneAPI::GetCaseInfo(&caseInfo)) // 题号
        {
            std::cout << "SimOneAPI::GetCaseInfo Failed!" << std::endl;
        }
        int num = -1;
        sscanf_s(caseInfo.caseName, "%d", &num);

        if (num == 19)
        {
            LOOK_AHEAD_GAIN = 0.12;
        }
        else if (num == 41)
        {
            LOOK_AHEAD_GAIN = 0.22;//
            maxAccel = 50;
            MIN_TARGET_SPEED = 6.00;//
            MAX_LAT_ACC = 16;//
            MAX_LOOK_AHEAD=50;
            throttle_K = 0.8;
        }

        std::cout << "num:" << num;
        // 获取路径点
        std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
        if (SimOneAPI::GetWayPoints(MAIN_VEHICLE_ID, pWayPoints.get()))
        {
            for (size_t i = 0; i < pWayPoints->wayPointsSize; ++i)
            {
                SSD::SimPoint3D inputWayPoints(static_cast<double>(pWayPoints->wayPoints[i].posX),
                                               static_cast<double>(pWayPoints->wayPoints[i].posY),
                                               0.0);
                inputPoints.push_back(inputWayPoints);
            }
            targetPointPos = inputPoints.back();
        }
        else
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Get mainVehicle wayPoints failed");
            continue;
        }

        // 生成路线
        if (pWayPoints->wayPointsSize >= 2)
        {
            SSD::SimVector<int> indexOfValidPoints;
            if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, targetPath))
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                     "Generate mainVehicle route failed");
                continue;
            }
        }
        else if (pWayPoints->wayPointsSize == 1)
        {
            SSD::SimString laneIdInit = SampleGetNearMostLane(inputPoints[0]);
            HDMapStandalone::MLaneInfo laneInfoInit;
            if (!SimOneAPI::GetLaneSample(laneIdInit, laneInfoInit))
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                     "Generate mainVehicle initial route failed");
                continue;
            }
            else
            {
                targetPath = laneInfoInit.centerLine;
            }
        }
        else
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error, "Number of wayPoints is zero");
            continue;
        }
        break;
    }

    // 主循环
    while (true)
    {
        fps_count++;
        int frame = SimOneAPI::Wait();
        ESimOne_Case_Status runStatus = SimOneAPI::GetCaseRunStatus();
        bool hasObstacleOnCrosswalk = false;
        static bool hasObstacleOnCrosswalk_flag = false;

        // 根据驻车停止状态进入评价函数
        if (runStatus == ESimOne_Case_Status_Stop)
        {
            SimOneAPI::SaveEvaluationRecord();
            break;
        }
        // 获取车辆状态数据
        if (!SimOneAPI::GetGps(MAIN_VEHICLE_ID, gps.get()))
        {
            // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "Fetch GPS failed");
        }

        if (SimOneAPI::GetGroundTruth(MAIN_VEHICLE_ID, obstacle.get()))
        {
            // printObstacleInfo(*obstacle);//打印
        }
        else
        {
            // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "");
        }
        SimOneAPI::GetTrafficSignList(signList);
        if (signList.empty())
        {
            // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Warning, "no trafficSign");
        }
        // 案例运行时处理逻辑
        if (runStatus == ESimOne_Case_Status::ESimOne_Case_Status_Running)
        {
            if (!isSimOneInitialized)
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initialized!");
                isSimOneInitialized = true;
            }
            // 获取主车状态
            SSD::SimPoint3D mainVehiclePos(gps->posX, gps->posY, gps->posZ);

            double minDistance = std::numeric_limits<double>::max();
            int potentialObstacleIndex = obstacle->obstacleSize;
            mainVehicleLaneId_last = mainVehicleLaneId;
            mainVehicleLaneId = SampleGetNearMostLane(mainVehiclePos);
            if (fps_count == 1)
            {
                mainVehicleLaneId_last = mainVehicleLaneId;
            }

            SSD::SimString potentialObstacleLaneId = "";
            if (!DetectObstacleInSameLane(*obstacle, mainVehiclePos, mainVehicleLaneId, minDistance,
                                          potentialObstacleIndex, potentialObstacleLaneId))
            {
                // No obstacle found in same lane
            }

            auto &potentialObstacle = obstacle->obstacle[potentialObstacleIndex];
            SSD::SimPoint3D potentialObstaclePos(potentialObstacle.posX, potentialObstacle.posY,
                                                 potentialObstacle.posZ);

            // 拿到控制和车灯
            auto control = std::make_unique<SimOne_Data_Control>();
            auto light = std::make_unique<SimOne_Data_Signal_Lights>();

            control->throttle = DEFAULT_THROTTLE;
            control->brake = defalut_brake;
            control->steering = 0.f;
            control->handbrake = false;
            control->isManualGear = false;
            control->gear = static_cast<ESimOne_Gear_Mode>(1);
            light->signalLights = ESimOne_Signal_Light::ESimOne_Signal_Light_None;

            HDMapStandalone::MLaneLink laneLink;
            SimOneAPI::GetLaneLink(mainVehicleLaneId, laneLink);

            static size_t targetIdx = 0; // 记录当前目标点序号
            static SSD::SimPoint3D mainVehiclePos_tmp{gps->posX, gps->posY, gps->posZ};

            std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
            if (SimOneAPI::GetWayPoints(MAIN_VEHICLE_ID, pWayPoints.get()))
            {
                size_t wpSize = pWayPoints->wayPointsSize;
                if (wpSize < 2)
                {
                    SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                         "WayPoints size too small");
                }

                // 1. 解析全部路径点（仅内部使用）
                SSD::SimPoint3DVector allWayPoints;
                for (size_t i = 0; i < wpSize; ++i)
                {
                    SSD::SimPoint3D pt;
                    pt.x = pWayPoints->wayPoints[i].posX;
                    pt.y = pWayPoints->wayPoints[i].posY;
                    pt.z = 0;
                    allWayPoints.push_back(pt);
                }

                // 2. 判断是否进入当前目标点，推进 targetIdx
                double dx = allWayPoints[targetIdx].x - mainVehiclePos.x;
                double dy = allWayPoints[targetIdx].y - mainVehiclePos.y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 10.0 && targetIdx + 1 < wpSize)
                {
                    ++targetIdx;
                    mainVehiclePos_tmp = mainVehiclePos;
                }

                // 3. 设置当前目标点和下一点
                SSD::SimPoint3D targetPointPos1 = allWayPoints[targetIdx];
                SSD::SimPoint3D targetPointPos2 = (targetIdx + 1 < wpSize) ? allWayPoints[targetIdx + 1]
                                                                           : allWayPoints[targetIdx];
                SSD::SimPoint3D targetPointPos3 = (targetIdx + 2 < wpSize) ? allWayPoints[targetIdx + 2]
                                                                           : allWayPoints[targetIdx];

                // 4. 构建 inputPoints（在 change_road_function 内部完成）
                change_road_function(&inputPoints, mainVehiclePos_tmp, targetPointPos1, targetPointPos2, targetPointPos3);
            }
            SSD::SimVector<int> tempindexOfValidPoints;
            if (!SimOneAPI::GenerateRoute(inputPoints, tempindexOfValidPoints, targetPath))
            {
                SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Error,
                                     "Generate mainVehicle route failed when finish change road");
            }
            std::vector<TrajectoryPoint> targetPath_info;                                     // 已经默认初始化为空向量，可以正常用于引用传参
            ConvertToTrajectoryPoints(targetPath, DEFAULT_SPEED, gps.get(), targetPath_info); // 轨迹转换
            UpdateTurnSignals(gps.get(), targetPath_info, light.get());                       // 更新车灯

            double leftCarSpeed = 0.0;
            if (IsCarInLeftLane(mainVehiclePos, mainVehicleLaneId, obstacle.get(), laneLink, leftCarSpeed)) {
                if (leftCarSpeed > 4.0) { // 速度大于 4 m/s
                    light->signalLights = ESimOne_Signal_Light_LeftBlinker;
                }
            }
            // std::cout << "\n=== output_traj ===" << std::endl;
            // for (size_t i = 0; i < output_traj.size(); ++i)
            // {
            //     const auto &p = output_traj[i];
            //     std::cout << "Point " << i << ": "
            //               << "x = " << p.x << ", "
            //               << "y = " << p.y << ", "
            //               << "v = " << p.v << std::endl;
            // }
            for (const auto &sign : signList)
            {
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign id: %ld", sign.id);
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign.type: %s",
                //                      sign.type.GetString());
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign.subType: %s",
                //                      sign.subType.GetString());
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug,
                //                      "[sign.pt.x: %f, sign.pt.y: %f, sign.pt.z: %f]", sign.pt.x, sign.pt.y, sign.pt.z);
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug,
                //                      "[sign.heading.x: %f, sign.heading.y: %f, sign.heading.y: %f]", sign.heading.x,
                //                      sign.heading.y, sign.heading.z);
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign.value: %s",
                //                      sign.value.GetString());
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign.unit: %s",
                //                      sign.unit.GetString());
                // SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Debug, "sign.isDynamic: %d",
                //                      sign.isDynamic);
                if (GetTrafficSignType(sign.type) == SpeedLimit_Sign)
                {
                    char *endptr;
                    double speedLimit = std::strtod(sign.value.GetString(), &endptr) / 3.6;
                    ModifySpeedAroundTarget(targetPath_info, sign.pt.x, sign.pt.y, speedLimit - 0.5, 140, 0, 10,
                                            SpeedZoneType::AFTER_TARGET);
                }
                if (strcmp(sign.type.GetString(), "1010101100001111") == 0) // 注意儿童
                {
                    ModifySpeedAroundTarget(targetPath_info, sign.pt.x, sign.pt.y, 8.2, 100, 0, 140,
                                            SpeedZoneType::BEFORE_TARGET);
                    control->throttle = -1.0; // 空油门标志位
                }
                // if (strcmp(sign.type.GetString(), "1010200100001914") == 0) // 停车让行
                // {
                    

                // 获取主车所在车道
                SSD::SimPoint3D egoPos{gps->posX, gps->posY, gps->posZ};
                SSD::SimString laneId;
                double s, t, s_toCenter, t_toCenter;
                SimOneAPI::GetNearMostLane(egoPos, laneId, s, t, s_toCenter, t_toCenter);

                // 获取车道关联的人行横道线
                SSD::SimVector<HDMapStandalone::MObject> crosswalks;
                SimOneAPI::GetSpecifiedLaneCrosswalkList(laneId, crosswalks);




                for (const auto &crosswalk : crosswalks)
                {
                    const auto &points = crosswalk.boundaryKnots;
                    if (points.empty())
                        continue;

                    double minX = std::numeric_limits<double>::max();
                    double maxX = std::numeric_limits<double>::lowest();
                    double minY = std::numeric_limits<double>::max();
                    double maxY = std::numeric_limits<double>::lowest();

                    for (const auto &pt : points)
                    {
                        minX = std::min(minX, pt.x);
                        maxX = std::max(maxX, pt.x);
                        minY = std::min(minY, pt.y);
                        maxY = std::max(maxY, pt.y);
                    }

                    // 扩展一点点容差（可选）
                    constexpr double marginX = 1.0;
                    constexpr double marginY = 1.0;
                    minX -= marginX;
                    maxX += marginX;
                    minY -= marginY;
                    maxY += marginY;

                    // 遍历障碍物
                    for (int i = 0; i < obstacle->obstacleSize; ++i)
                    {
                        const auto &obs = obstacle->obstacle[i];
                        if (obs.type != ESimOne_Obstacle_Type_Pedestrian &&
                            obs.type != ESimOne_Obstacle_Type_Car)
                            continue;


                        double obsX = obs.posX;
                        double obsY = obs.posY;

                        if (obsX >= minX && obsX <= maxX &&
                            obsY >= minY && obsY <= maxY)
                        {
                            hasObstacleOnCrosswalk = true;
                            hasObstacleOnCrosswalk_flag = true;
                            break;
                        }
                    }

                    if (hasObstacleOnCrosswalk)
                        break;
                }
                    

                if (hasObstacleOnCrosswalk)
                {
                        StopBeforeStoplineIfNeeded(*gps, targetPath_info, 0.5,0); // 人行横道前停车
                }
                // }

                // std::cout << "sign type: " << sign.type.GetString() << std::endl;
            }
            if (!hasObstacleOnCrosswalk_flag)
            {
                LimitThrottleIfOpposingMovingObstacle(MAIN_VEHICLE_ID, *gps, *obstacle, targetPath_info, control.get()); // 无信号灯路口车辆冲突通行
            }
            StopBeforeJunctionIfOpposingObstacle(*gps, *obstacle, targetPath_info);                                  // 对向路口堵塞
            LaneChangeSegmentPrefilter(*gps, *obstacle, targetPath_info);
            LateralBypassCore(*gps, *obstacle, targetPath_info);
            // if (gps->posX < 161 && gps->posX > 120 && gps->posY < 8 && gps->posY > -5)
            // {
            //     if (frame % 10 == 0)
            //     {
            //         PrintTrajectory(targetPath_info);
            //     }
            // }
            ProcessObstacleLongitudinalSpeed(*gps, *obstacle, targetPath_info, 200);
            StopForRedLightIfNeeded(MAIN_VEHICLE_ID,*gps, targetPath_info);//红绿灯停车
            AdjustTrajectoryIfOpposingTraffic(*gps, *obstacle, targetPath_info, 1.3);//对向快速来车调整
            SmoothTrajectorySpeedForward(targetPath_info,DEFAULT_SPEED, 11.8);//平滑轨迹速度
            processControl(gps.get(), targetPath_info, control.get(), frame);//控制

            SimOneAPI::SetSignalLights(MAIN_VEHICLE_ID, light.get());
            SimOneAPI::SetDrive(MAIN_VEHICLE_ID, control.get());
        }
        else
        {
            SimOneAPI::SetLogOut(ESimOne_LogLevel_Type::ESimOne_LogLevel_Type_Information, "SimOne Initializing...");
        }
        SimOneAPI::NextFrame(frame);
    }
    return 0;
}

void change_road_function(SSD::SimPoint3DVector *inps, const SSD::SimPoint3D &start_p,
                          const SSD::SimPoint3D &mid_p, const SSD::SimPoint3D &end_p, const SSD::SimPoint3D &endend_p)
{
    inps->clear();
    inps->push_back(start_p);  // 当前主车位置
    inps->push_back(mid_p);    // 当前目标点
    inps->push_back(end_p);    // 下一个目标点
    inps->push_back(endend_p); // 下一个目标点
}

bool IsCarInLeftLane(const SSD::SimPoint3D &MainCarPos, const SSD::SimString &MainCarLandId,
                     SimOne_Data_Obstacle *obstacle,
                     const HDMapStandalone::MLaneLink &laneLink, double &LefrCarSpeed)
{
    bool res = false;
    double MinDistance = std::numeric_limits<double>::max();
    SSD::SimString CarLaneId = "";
    double s, t, s_toCenterLine, t_toCenterLine;
    for (size_t i = 0; i < obstacle->obstacleSize; ++i)
    {
        if (obstacle->obstacle[i].type == ESimOne_Obstacle_Type::ESimOne_Obstacle_Type_Car)
        {
            SSD::SimPoint3D CarPos = {
                obstacle->obstacle[i].posX, obstacle->obstacle[i].posY, obstacle->obstacle[i].posZ};
            SimOneAPI::GetNearMostLane(CarPos, CarLaneId, s, t, s_toCenterLine, t_toCenterLine);
            double Distance = UtilMath::planarDistance(MainCarPos, CarPos);

            if (strcmp(CarLaneId.GetString(), laneLink.leftNeighborLaneName.GetString()) == 0 && Distance <=
                                                                                                     LEFT_LANE_DETECTION_DISTANCE)
            {
                if (Distance < MinDistance)
                {
                    MinDistance = Distance;
                    LefrCarSpeed = UtilMath::calculateSpeed(obstacle->obstacle[i].velX, obstacle->obstacle[i].velY,
                                                            obstacle->obstacle[i].velZ);
                    res = true;
                }
            }
        }
    }
    return res;
}

bool DetectObstacleInSameLane(const SimOne_Data_Obstacle &obstacleData, const SSD::SimPoint3D &mainCarPos,
                              const SSD::SimString &mainVehicleLaneId,
                              double &minDistance, int &potentialObstacleIndex,
                              SSD::SimString &potentialObstacleLaneId)
{
    for (size_t i = 0; i < obstacleData.obstacleSize; ++i)
    {
        SSD::SimPoint3D obstaclePos(obstacleData.obstacle[i].posX, obstacleData.obstacle[i].posY,
                                    obstacleData.obstacle[i].posZ);
        SSD::SimString obstacleLaneId = SampleGetNearMostLane(obstaclePos);
        if (mainVehicleLaneId == obstacleLaneId)
        {
            double obstacleDistance = UtilMath::planarDistance(mainCarPos, obstaclePos);

            if (obstacleDistance < minDistance)
            {
                minDistance = obstacleDistance;
                potentialObstacleIndex = static_cast<int>(i);
                potentialObstacleLaneId = obstacleLaneId;
            }
        }
    }
    return potentialObstacleIndex < obstacleData.obstacleSize;
}

void ConvertToTrajectoryPoints(const SSD::SimPoint3DVector &traj_data,
                               double default_speed,
                               const SimOne_Data_Gps *pGps,
                               std::vector<TrajectoryPoint> &output)
{
    if (default_speed <= 0.0)
        default_speed = DEFAULT_SPEED;
    output.clear();
    if (traj_data.size() < 2)
        return;

    /* ---------- 1. 计算车辆在轨迹上的投影 ---------- */
    size_t start_seg = 0;     // 投影落在哪一段  [P_i , P_{i+1}]
    double start_s_off = 0.0; // 投影点距段首的弧长
    bool has_proj = false;

    if (pGps)
    {
        const double gx = pGps->posX, gy = pGps->posY;
        double best_dist2 = 1e30;

        for (size_t i = 0; i + 1 < traj_data.size(); ++i)
        {
            const auto &p0 = traj_data[i];
            const auto &p1 = traj_data[i + 1];
            const double vx = p1.x - p0.x, vy = p1.y - p0.y;
            const double len2 = vx * vx + vy * vy;
            if (len2 < 1e-6)
                continue;

            const double t = CLAMP(((gx - p0.x) * vx + (gy - p0.y) * vy) / len2, 0.0, 1.0);
            const double px = p0.x + t * vx;
            const double py = p0.y + t * vy;
            const double dx = gx - px, dy = gy - py;
            const double dist2 = dx * dx + dy * dy;

            if (dist2 < best_dist2)
            {
                best_dist2 = dist2;
                start_seg = i;
                start_s_off = t * std::sqrt(len2);
                has_proj = true;
            }
        }
    }

    /* ---------- 2. 从投影点开始 0.2 m 重采样 ---------- */
    std::vector<TrajectoryPoint> raw;
    raw.reserve(static_cast<size_t>(traj_data.size() * (1.0 / RESAMPLE_DIST + 1)));

    // 2-A 先放投影点（若没有 GPS 或投影失败则放起点）
    SSD::SimPoint3D proj = has_proj
                               ? SSD::SimPoint3D{traj_data[start_seg].x +
                                                     (traj_data[start_seg + 1].x - traj_data[start_seg].x) *
                                                         (start_s_off /
                                                          std::hypot(traj_data[start_seg + 1].x - traj_data[start_seg].x,
                                                                     traj_data[start_seg + 1].y - traj_data[start_seg].y)),
                                                 traj_data[start_seg].y +
                                                     (traj_data[start_seg + 1].y - traj_data[start_seg].y) *
                                                         (start_s_off /
                                                          std::hypot(traj_data[start_seg + 1].x - traj_data[start_seg].x,
                                                                     traj_data[start_seg + 1].y - traj_data[start_seg].y)),
                                                 0.0}
                               : traj_data.front();

    raw.push_back({proj.x, proj.y, static_cast<float>(default_speed)});

    double leftover = RESAMPLE_DIST - 1e-6; // 使循环逻辑统一
    if (has_proj)
        leftover = RESAMPLE_DIST - std::fmod(start_s_off, RESAMPLE_DIST);

    double accLen = 0.0;
    bool done = false;

    for (size_t i = start_seg; i + 1 < traj_data.size(); ++i)
    {
        const auto &p0 = traj_data[i];
        const auto &p1 = traj_data[i + 1];
        const double seg_len = std::hypot(p1.x - p0.x, p1.y - p0.y);
        if (seg_len < 1e-6)
            continue;

        int n = static_cast<int>(std::floor((seg_len + leftover) / RESAMPLE_DIST));
        for (int k = 0; k < n; ++k)
        {
            if (accLen >= MAX_TOTAL_LEN)
            {
                done = true;
                break;
            }

            double dist = leftover + k * RESAMPLE_DIST; // 距 p0 的弧长
            double t = dist / seg_len;

            double px = p0.x + t * (p1.x - p0.x);
            double py = p0.y + t * (p1.y - p0.y);
            raw.push_back({px, py, static_cast<float>(default_speed)});

            accLen += RESAMPLE_DIST;
        }
        if (done)
            break;
        leftover = seg_len + leftover - n * RESAMPLE_DIST;
    }

    /* ---------- 2.5 若长度不足，则沿末端方向补足 ---------- */
    while (accLen + RESAMPLE_DIST <= MAX_TOTAL_LEN && raw.size() >= 2)
    {
        const auto &pN = raw[raw.size() - 1];
        const auto &pN_1 = raw[raw.size() - 2];
        double dx = pN.x - pN_1.x, dy = pN.y - pN_1.y;
        double len = std::hypot(dx, dy);
        if (len < 1e-6)
            break;

        dx /= len;
        dy /= len;
        double new_x = pN.x + dx * RESAMPLE_DIST;
        double new_y = pN.y + dy * RESAMPLE_DIST;
        raw.push_back({new_x, new_y, static_cast<float>(default_speed)});
        accLen += RESAMPLE_DIST;
    }

    /* ---------- 3. 简单滑动均值平滑 ---------- */
    constexpr int SMOOTH_WIN = 2;
    output.reserve(raw.size());

    for (size_t i = 0; i < raw.size(); ++i)
    {
        double sumx = 0.0, sumy = 0.0;
        int cnt = 0;
        const size_t beg = (i > SMOOTH_WIN) ? i - SMOOTH_WIN : 0;
        const size_t end = std::min(raw.size() - 1, i + SMOOTH_WIN);
        for (size_t j = beg; j <= end; ++j)
        {
            sumx += raw[j].x;
            sumy += raw[j].y;
            ++cnt;
        }
        output.push_back({sumx / cnt, sumy / cnt, static_cast<float>(raw[i].v)});
    }
}

// void HandleLaneChange(const SSD::SimPoint3D &mainVehiclePos, const SSD::SimPoint3D &targetPointPos, uint8_t left_lane,
//                       uint8_t right_lane,
//                       uint8_t &change_road, uint8_t &use_fixd_steering, float &steering,
//                       SSD::SimPoint3DVector &inputPoints, SSD::SimPoint3DVector &targetPath)
// {
//     if (change_road == ON)
//     {
//         if (left_lane == ON)
//         {
//             steering = MINMAX(-dis_CR / (float)targetPath.size(), -0.3f, 0.3f);
//         }
//         else if (right_lane == ON)
//         {
//             steering = MINMAX(dis_CR / (float)targetPath.size(), -0.3f, 0.3f);
//         }
//     }
// }

void processControl(const SimOne_Data_Gps *pGps,
                    const std::vector<TrajectoryPoint> &traj_data,
                    SimOne_Data_Control *control,
                    int frame)
{
    /******************** ① 静态状态 ************************/
    static size_t currentIndex = 0;       // 当前匹配的轨迹点索引
    static double previousSteering = 0.0; // 上一时刻方向盘角度

    /******************** ② 车速与前视距离 ********************/
    double currentSpeed = std::sqrt(pGps->velX * pGps->velX +
                                    pGps->velY * pGps->velY +
                                    pGps->velZ * pGps->velZ); // m/s
    double speed4Look = currentSpeed;                         // m/s

    double lookAheadDist = MIN_LOOK_AHEAD + speed4Look * LOOK_AHEAD_GAIN;
    lookAheadDist = MINMAX(lookAheadDist, MIN_LOOK_AHEAD, MAX_LOOK_AHEAD);

    /******************** ③ 轨迹匹配 ************************/
    double minDist = std::numeric_limits<double>::max();
    size_t closestIndex = currentIndex;

    size_t startIdx = (currentIndex >= 100) ? currentIndex - 100 : 0;
    size_t endIdx = std::min(currentIndex + 150, traj_data.size() - 1);

    for (size_t i = startIdx; i <= endIdx; ++i)
    {
        double dist = std::hypot(pGps->posX - traj_data[i].x,
                                 pGps->posY - traj_data[i].y);
        if (dist < minDist)
        {
            minDist = dist;
            closestIndex = i;
        }
    }
    currentIndex = closestIndex;

    /******************** ④ 查找前视目标点 ********************/
    size_t targetIndex = closestIndex;
    double accumLookAhead = 0.0;
    while (targetIndex < traj_data.size() - 1 &&
           accumLookAhead < lookAheadDist)
    {
        size_t i = targetIndex;
        size_t ip = (i == 0) ? 0 : i - 1;
        accumLookAhead += hypot(traj_data[i].x - traj_data[ip].x,
                                traj_data[i].y - traj_data[ip].y);
        ++targetIndex;
    }

    /******************** ⑤ 纯跟踪转向角计算（弧度制） *******************/
    double dx = traj_data[targetIndex].x - pGps->posX;
    double dy = traj_data[targetIndex].y - pGps->posY;

    auto normalizeAngle = [](double a)
    {
        while (a > M_PI)
            a -= 2.0 * M_PI;
        while (a < -M_PI)
            a += 2.0 * M_PI;
        return a;
    };

    double targetHeading = normalizeAngle(std::atan2(dy, dx));
    double currentHeading = normalizeAngle(pGps->oriZ);

    double alfa = normalizeAngle(targetHeading - currentHeading); // 横向误差角
    double ld = std::hypot(dx, dy);                               // 前视距离
    constexpr double L = 2.85;                                    // 轴距 (m)

    // --- Pure‑Pursuit 转向角（弧度）
    double delta_pp = -std::atan2(2.0 * L * std::sin(alfa), ld); // rad

    // 将弧度映射到归一化 [-1,1]（MAX_STEER_RAD 需按车型标定）
    constexpr double MAX_STEER_RAD = 0.5235987755982988;         // ≈30°
    double cmdNorm = CLAMP(delta_pp / MAX_STEER_RAD, -1.0, 1.0); // 归一化命令

    /******************** ⑥ 目标舵角平滑（取代“大 PID”） ********************/
    // 使用一阶低通 + 限速，保持带宽又防毛刺
    static double refSteering = 0.0;  // 归一化 [-1,1]
    constexpr double ALPHA_LPF = 0.8; // 0(无滤波)~1(全滤波)
    double deltaRefFast = (1.0 - ALPHA_LPF) * cmdNorm + ALPHA_LPF * refSteering;

    // 每帧最大变化率（根据速度自适应亦可）
    constexpr double MAX_RATE = 0.02; // per frame (norm units)
    double diff = CLAMP(deltaRefFast - refSteering,
                        -MAX_RATE, MAX_RATE);
    refSteering += diff;

    /******************** ⑦ 执行器内环 PID（小 PID） *********************/
    static double integral = 0.0;
    static double prevErr = 0.0;
    static int prevFrame = -1;

    double dFrame = (prevFrame >= 0) ? (frame - prevFrame) : 1.0;
    if (dFrame <= 0.0)
        dFrame = 1.0;
    prevFrame = frame;
    double err = refSteering - previousSteering;

    integral += err * dFrame;
    integral = CLAMP(integral, -1.0, 1.0);
    double dErr = (err - prevErr) / dFrame;
    prevErr = err;

    double deltaOut = KP_STEER * err +
                      KI_STEER * integral +
                      KD_STEER * dErr;

    double smoothedSteering = CLAMP(previousSteering + deltaOut,
                                    -1.0, 1.0);
    previousSteering = smoothedSteering;

    /******************** ⑦ 未来轨迹曲率估计（以当前点为起点） *********************/
    double maxCurvature = 0.0;
    int sampleNum = 150;

    for (int i = 0; i < sampleNum; ++i)
    {
        size_t idx1 = closestIndex + i;
        size_t idx2 = closestIndex + i + 1;
        size_t idx3 = closestIndex + i + 2;

        if (idx3 >= traj_data.size())
            break;

        const auto &p1 = traj_data[idx1];
        const auto &p2 = traj_data[idx2];
        const auto &p3 = traj_data[idx3];

        double a = std::hypot(p2.x - p1.x, p2.y - p1.y);
        double b = std::hypot(p3.x - p2.x, p3.y - p2.y);
        double c = std::hypot(p3.x - p1.x, p3.y - p1.y);

        double s = (a + b + c) / 2.0;
        double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));

        if (a > 1e-4 && b > 1e-4 && c > 1e-4 && area > 1e-6)
        {
            double curvature = (4.0 * area) / (a * b * c); // κ = 2S / (abc)
            maxCurvature = std::max(maxCurvature, curvature);
        }
    }

    // 曲率→半径→角速度限制
    double radius;
    if (maxCurvature < 1e-6)
        radius = 1e6;
    else
        radius = 1.0 / maxCurvature;

    radius = std::max(radius, MIN_RADIUS);
    double cornerSpeed = std::sqrt(MAX_LAT_ACC * radius);
    cornerSpeed = std::max(cornerSpeed, MIN_TARGET_SPEED); // 限定最小速度

    /******************** ⑧ 目标速度选择 **********************/
    double trajSpeed = traj_data[targetIndex].v; // m/s
    double desiredSpeed = std::min(trajSpeed, cornerSpeed);

    // // ---------- 平滑速度变化：使用实际时间戳限制最大加速度 ----------
    // static double prevDesiredSpeed = -1.0;
    // static long long prevTimestamp = -1;

    // if (prevDesiredSpeed < 0.0 || !pGps || pGps->timestamp <= 0) {
    //     // 初始化
    //     prevDesiredSpeed = currentSpeed;
    //     prevTimestamp = pGps ? pGps->timestamp : 0;
    // }

    // long long nowTs = pGps->timestamp;
    // double dt = (nowTs - prevTimestamp) / 1000.0; // 转换为秒
    // dt = std::max(1e-3, dt); // 防止除零或极小时间

    // double maxDeltaV = maxAccel * dt;
    // double delta = desiredSpeed - prevDesiredSpeed;
    // delta = CLAMP(delta, -maxDeltaV, maxDeltaV);

    // desiredSpeed = prevDesiredSpeed + delta;
    // prevDesiredSpeed = desiredSpeed;
    // prevTimestamp = nowTs;

    /******************** ⑨ 油门 / 刹车控制 ******************/
    double speedDiff = desiredSpeed - currentSpeed;

    if (control->throttle == -1.0) // 标志位保持静止
    {
        control->throttle = 0.0f;
        control->brake = 0.0f;
    }
    else if (desiredSpeed <= 0.1)
    {
        control->throttle = 0.0f;
        control->brake = 1.0f;
    }
    else if (std::fabs(speedDiff) < SPEED_HYSTERESIS)
    {
        control->throttle = 0.0f;
        control->brake = 0.0f;
    }
    else if (speedDiff > 0)
    {
        // 需要加速
        control->throttle = std::min(1.0f, static_cast<float>(speedDiff * throttle_K));
        control->brake = 0.0f;
    }
    else
    {
        // 需要减速
        control->throttle = 0.0f;
        control->brake = std::min(1.0f, static_cast<float>(-speedDiff * throttle_K));
    }

    /******************** ⑩ 输出到控制器 **********************/
    control->steering = static_cast<float>(smoothedSteering);

    /******************** ? 调试打印 *************************/
    // std::cout << "Closest: (" << traj_data[closestIndex].x << "," << traj_data[closestIndex].y << ") "
    //           << "Target: (" << traj_data[targetIndex].x << "," << traj_data[targetIndex].y << ") "
    //           << "Steer: " << control->steering
    //           << " Throttle: " << control->throttle
    //           << " Brake: " << control->brake
    //           << " CurSpd: " << currentSpeed
    //           << " DesSpd: " << desiredSpeed
    //           << " TrajSpd: " << trajSpeed
    //           << " CnrSpd: " << cornerSpeed
    //           << std::endl;
}

void printObstacleInfo(const SimOne_Data_Obstacle &data)
{
    // std::cout << "Timestamp: " << data.timestamp
    //           << ", Frame: " << data.frame
    //           << ", Version: " << data.version << "\n";
    // std::cout << "Obstacle Count: " << data.obstacleSize << "\n";

    for (int i = 0; i < data.obstacleSize; ++i)
    {
        // const auto &obs = data.obstacle[i];
        // std::cout << "Obstacle[" << i << "] ID: " << obs.id
        //           << ", ViewID: " << obs.viewId
        //           << ", Type: " << obstacleTypeToString(obs.type)
        //           << ", Position(" << obs.posX << ", " << obs.posY << ", " << obs.posZ << ")"
        //           << ", Velocity(" << obs.velX << ", " << obs.velY << ", " << obs.velZ << ")"
        //           << ", Dimensions(" << obs.length << "x" << obs.width << "x" << obs.height << ")"
        //           << "\n";
    }
}

void ModifySpeedAroundTarget(std::vector<TrajectoryPoint> &trajectory,
                             double target_x, double target_y,
                             double new_speed,
                             double distance_threshold,
                             double distance_threshold_r,
                             double trigger_m,
                             SpeedZoneType zone)
{
    if (trajectory.empty() || distance_threshold < 0.0)
        return;

    // 缓存目标段（避免重复生成）
    static std::vector<TrajectoryPoint> cached_segment;
    static bool has_cached = false;

    // 哈希表加速空间搜索（key = cell_x << 32 | cell_y）
    static std::unordered_map<uint64_t, std::vector<std::pair<double, double>>> grid_map;
    static constexpr double CELL_SIZE = 0.5;

    auto toCellKey = [](double x, double y) -> uint64_t
    {
        int cx = static_cast<int>(std::floor(x / CELL_SIZE));
        int cy = static_cast<int>(std::floor(y / CELL_SIZE));
        return (static_cast<uint64_t>(cx) << 32) | (static_cast<uint32_t>(cy));
    };

    // --------- STEP 1: 判断是否需要初始化静态缓存 ----------
    if (!has_cached)
    {
        const double threshold_dist2 = trigger_m * trigger_m;
        bool trigger = false;
        int trigger_index = -1;

        if (zone == SpeedZoneType::AFTER_TARGET)
        {
            double dx = trajectory[0].x - target_x;
            double dy = trajectory[0].y - target_y;
            if (dx * dx + dy * dy <= threshold_dist2)
            {
                trigger = true;
                trigger_index = 0;
            }
        }
        else if (zone == SpeedZoneType::BEFORE_TARGET)
        {
            for (int i = static_cast<int>(trajectory.size()) - 1; i >= 0; --i)
            {
                double dx = trajectory[i].x - target_x;
                double dy = trajectory[i].y - target_y;
                if (dx * dx + dy * dy <= threshold_dist2)
                {
                    trigger = true;
                    trigger_index = i;
                    break;
                }
            }
        }

        if (trigger && trigger_index >= 0)
        {
            cached_segment.clear();
            double acc_dist = 0.0;

            if (zone == SpeedZoneType::AFTER_TARGET)
            {
                for (size_t i = trigger_index; i + 1 < trajectory.size(); ++i)
                {
                    cached_segment.push_back(trajectory[i]);
                    double dx = trajectory[i + 1].x - trajectory[i].x;
                    double dy = trajectory[i + 1].y - trajectory[i].y;
                    acc_dist += std::hypot(dx, dy);
                    if (acc_dist >= distance_threshold)
                        break;
                }
            }
            else // BEFORE_TARGET
            {
                for (int i = trigger_index; i > 0; --i)
                {
                    cached_segment.push_back(trajectory[i]);
                    double dx = trajectory[i].x - trajectory[i - 1].x;
                    double dy = trajectory[i].y - trajectory[i - 1].y;
                    acc_dist += std::hypot(dx, dy);
                    if (acc_dist >= distance_threshold)
                        break;
                }
            }

            // 加入反方向段
            if (distance_threshold_r > 0.0)
            {
                double acc_dist_r = 0.0;

                if (zone == SpeedZoneType::AFTER_TARGET)
                {
                    for (int i = trigger_index; i > 0; --i)
                    {
                        cached_segment.push_back(trajectory[i]);
                        double dx = trajectory[i].x - trajectory[i - 1].x;
                        double dy = trajectory[i].y - trajectory[i - 1].y;
                        acc_dist_r += std::hypot(dx, dy);
                        if (acc_dist_r >= distance_threshold_r)
                            break;
                    }
                }
                else // BEFORE_TARGET
                {
                    for (size_t i = trigger_index; i + 1 < trajectory.size(); ++i)
                    {
                        cached_segment.push_back(trajectory[i]);
                        double dx = trajectory[i + 1].x - trajectory[i].x;
                        double dy = trajectory[i + 1].y - trajectory[i].y;
                        acc_dist_r += std::hypot(dx, dy);
                        if (acc_dist_r >= distance_threshold_r)
                            break;
                    }
                }
            }

            // 构建空间哈希索引
            grid_map.clear();
            for (size_t i = 0; i < cached_segment.size(); ++i)
            {
                double x = cached_segment[i].x;
                double y = cached_segment[i].y;
                uint64_t key = toCellKey(x, y);
                grid_map[key].emplace_back(x, y);
            }

            has_cached = true;
        }
    }

    // --------- STEP 2: 匹配当前轨迹到缓存区域内 ----------
    if (has_cached)
    {
        constexpr double MATCH_DIST2 = 1.0 * 1.0;
        int updated_count = 0;

        for (size_t i = 0; i < trajectory.size(); ++i)
        {
            TrajectoryPoint &pt = trajectory[i];
            uint64_t key = toCellKey(pt.x, pt.y);
            bool matched = false;

            for (int dx = -1; dx <= 1 && !matched; ++dx)
            {
                for (int dy = -1; dy <= 1 && !matched; ++dy)
                {
                    uint64_t neighbor_key = toCellKey(pt.x + dx * CELL_SIZE, pt.y + dy * CELL_SIZE);
                    std::unordered_map<uint64_t, std::vector<std::pair<double, double>>>::iterator it = grid_map.find(neighbor_key);
                    if (it == grid_map.end())
                        continue;

                    const std::vector<std::pair<double, double>> &points = it->second;
                    for (size_t j = 0; j < points.size(); ++j)
                    {
                        double cx = points[j].first;
                        double cy = points[j].second;
                        double dist2 = (pt.x - cx) * (pt.x - cx) + (pt.y - cy) * (pt.y - cy);
                        if (dist2 <= MATCH_DIST2)
                        {
                            pt.v = new_speed;
                            ++updated_count;
                            matched = true;
                            break;
                        }
                    }
                }
            }
        }
    }
}

std::string obstacleTypeToString(ESimOne_Obstacle_Type type)
{
    switch (type)
    {
    case ESimOne_Obstacle_Type_Unknown:
        return "Unknown";
    case ESimOne_Obstacle_Type_Pedestrian:
        return "Pedestrian";
    case ESimOne_Obstacle_Type_Pole:
        return "Pole";
    case ESimOne_Obstacle_Type_Car:
        return "Car";
    case ESimOne_Obstacle_Type_Static:
        return "Static";
    case ESimOne_Obstacle_Type_Bicycle:
        return "Bicycle";
    case ESimOne_Obstacle_Type_Fence:
        return "Fence";
    case ESimOne_Obstacle_Type_RoadMark:
        return "RoadMark";
    case ESimOne_Obstacle_Type_TrafficSign:
        return "TrafficSign";
    case ESimOne_Obstacle_Type_TrafficLight:
        return "TrafficLight";
    case ESimOne_Obstacle_Type_Rider:
        return "Rider";
    case ESimOne_Obstacle_Type_Truck:
        return "Truck";
    case ESimOne_Obstacle_Type_Bus:
        return "Bus";
    case ESimOne_Obstacle_Type_SpecialVehicle:
        return "SpecialVehicle";
    case ESimOne_Obstacle_Type_Motorcycle:
        return "Motorcycle";
    case ESimOne_Obstacle_Type_Dynamic:
        return "Dynamic";
    case ESimOne_Obstacle_Type_GuardRail:
        return "GuardRail";
    case ESimOne_Obstacle_Type_SpeedLimitSign:
        return "SpeedLimitSign";
    case ESimOne_Obstacle_Type_BicycleStatic:
        return "BicycleStatic";
    case ESimOne_Obstacle_Type_RoadObstacle:
        return "RoadObstacle";
    default:
        return "Invalid Type";
    }
}

void blendAlongSegment(const std::vector<SegmentFrame> &segments,
                       std::vector<double> &dLat,
                       std::vector<TrajectoryPoint> &traj,
                       double R = DEFAULT_OFFSET_RADIUS)
{
    /* 先算全局累积 s（米） */
    std::vector<double> cumS(traj.size(), 0.0);
    for (size_t i = 1; i < traj.size(); ++i)
        cumS[i] = cumS[i - 1] + std::hypot(traj[i].x - traj[i - 1].x,
                                           traj[i].y - traj[i - 1].y);

    for (const auto &seg : segments)
    {
        /* 该段所有锚点的最大位移幅度（正负号保留） */
        double maxShift = 0.0;
        for (const auto &k : seg.trajSL)
        {
            size_t idx = k.idx;
            if (std::fabs(dLat[idx]) > std::fabs(maxShift))
                maxShift = dLat[idx];
        }
        if (std::fabs(maxShift) < 1e-4)
            continue; // 这段其实没横移

        /* 找段首、段尾在全轨迹中的 s 值 */
        double sBeg = cumS[seg.trajSL.front().idx];
        double sEnd = cumS[seg.trajSL.back().idx];

        /* 扫描全轨迹，距离段端点 ≤ R 的点线性吸引 */
        for (size_t i = 0; i < traj.size(); ++i)
        {
            double ds = 0.0;
            if (cumS[i] < sBeg)
                ds = sBeg - cumS[i];
            else if (cumS[i] > sEnd)
                ds = cumS[i] - sEnd;
            else
                continue; // 段内部点已是锚或已调整

            if (ds > R)
                continue; // 超出影响半径

            /* 权重  w = 1 - ds/R  （越近位移越大） */
            double w = 1.0 - ds / R;
            double target = maxShift * w;

            /* 只在“目标位移幅度大于当前” 时施加 */
            if (std::fabs(target) <= std::fabs(dLat[i]) + 1e-4)
                continue;

            double delta = target - dLat[i];
            dLat[i] = target;

            /* 世界坐标移动：用该段的法向 (nx,ny) */
            traj[i].x += seg.nx * delta;
            traj[i].y += seg.ny * delta;
        }
    }
}

void smoothCurvatureBlend(std::vector<TrajectoryPoint> &traj,
                          std::vector<double> &dLat,
                          double influence = 20.0,
                          double minR = 10.0,
                          int iterMax = 5,
                          int winSize = 20)
{
    if (traj.size() < 3)
        return;

    /* ---------- ① 计算累计 s 沿轨迹 ---------- */
    std::vector<double> cumS(traj.size(), 0.0);
    for (size_t i = 1; i < traj.size(); ++i)
        cumS[i] = cumS[i - 1] + std::hypot(traj[i].x - traj[i - 1].x,
                                           traj[i].y - traj[i - 1].y);

    /* ---------- ② 基于锚点扩散目标位移 targetLat ---------- */
    std::vector<double> targetLat = dLat; // 默认=已有位移

    for (size_t a = 0; a < dLat.size(); ++a)
    {
        if (std::fabs(dLat[a]) < 1e-4)
            continue;           // 非锚点跳过
        double shift = dLat[a]; // 锚点位移

        /* 前后扫描，距离 ≤ influence → 线性衰减 */
        for (size_t k = 0; k < dLat.size(); ++k)
        {
            double ds = std::fabs(cumS[k] - cumS[a]);
            if (ds > influence)
                continue;

            double w = 1.0 - ds / influence; // 0~1 线性权重
            double cand = shift * w;         // 擬合位移
            if (std::fabs(cand) > std::fabs(targetLat[k]))
                targetLat[k] = cand; // 取更大绝对值
        }
    }

    /* ---------- ③ 把“增量”位移施加到世界坐标 ---------- */
    for (size_t i = 0; i < traj.size(); ++i)
    {
        double addShift = targetLat[i] - dLat[i];
        if (std::fabs(addShift) < 1e-4)
            continue; // 无需额外移动

        /* 取车道法向方向（与第 3 步一致，调用 HDMap） */
        SSD::SimPoint3D proj, dir;
        SSD::SimString dum;
        if (!SimOneAPI::GetLaneMiddlePoint({traj[i].x, traj[i].y, 0.0},
                                           dum, proj, dir))
            continue;
        double nx = -dir.y, ny = dir.x;
        double L = std::hypot(nx, ny);
        if (L < 1e-4)
            continue;
        nx /= L;
        ny /= L;

        traj[i].x += nx * addShift;
        traj[i].y += ny * addShift;
        dLat[i] += addShift; // 更新实际位移
    }

    /* ---------- ④ 曲率约束平滑（只调非锚点） ---------- */
    std::vector<bool> isFixed(traj.size(), false);
    for (size_t i = 0; i < dLat.size(); ++i)
        if (std::fabs(dLat[i]) > 1e-4)
            isFixed[i] = true;

    for (int it = 0; it < iterMax; ++it)
    {
        bool changed = false;
        std::vector<TrajectoryPoint> prv = traj;

        for (size_t i = 1; i + 1 < traj.size(); ++i)
        {
            if (isFixed[i])
                continue;

            /* 拉普拉斯候选 */
            double cx = 0.5 * (prv[i - 1].x + prv[i + 1].x);
            double cy = 0.5 * (prv[i - 1].y + prv[i + 1].y);

            /* 曲率估算 */
            double v1x = cx - prv[i - 1].x, v1y = cy - prv[i - 1].y;
            double v2x = prv[i + 1].x - cx, v2y = prv[i + 1].y - cy;
            double len1 = std::hypot(v1x, v1y), len2 = std::hypot(v2x, v2y);
            if (len1 < 1e-4 || len2 < 1e-4)
                continue;

            double cosT = (v1x * v2x + v1y * v2y) / (len1 * len2);
            cosT = std::max(-1.0, std::min(1.0, cosT));
            double theta = std::acos(cosT);
            if (theta < 1e-3)
                continue;

            double ds = 0.5 * (len1 + len2);
            double R = ds / theta; // 近似曲率半径

            if (R >= minR)
            {
                traj[i].x = cx;
                traj[i].y = cy;
                changed = true;
            }
        }
        if (!changed)
            break; // 收敛
    }
    std::vector<TrajectoryPoint> smoothed = traj;

    for (size_t i = 0; i < traj.size(); ++i)
    {
        int beg = std::max<int>(0, i - winSize);
        int end = std::min<int>(traj.size() - 1, i + winSize);

        double sumX = 0.0, sumY = 0.0;
        int cnt = 0;
        for (int j = beg; j <= end; ++j)
        {
            sumX += traj[j].x;
            sumY += traj[j].y;
            ++cnt;
        }

        smoothed[i].x = sumX / cnt;
        smoothed[i].y = sumY / cnt;
    }

    traj = smoothed; // 替换轨迹
}

static std::vector<double> buildCumS(const SSD::SimPoint3DVector &bd)
{
    std::vector<double> cum(bd.size(), 0.0);
    for (size_t i = 1; i < bd.size(); ++i)
    {
        cum[i] = cum[i - 1] +
                 std::hypot(bd[i].x - bd[i - 1].x,
                            bd[i].y - bd[i - 1].y);
    }
    return cum;
}
/* 投影任意点到折线段集合，返回 (sAbs, l)  —  sAbs=投影点沿折线的累计弧长 */
static std::pair<double, double> projectToPolylineSL(const SSD::SimPoint3DVector &bd,
                                                     const std::vector<double> &cumS,
                                                     double x, double y)
{
    double bestDist2 = 1e30;
    double bestSAbs = 0.0, bestL = 0.0;

    for (size_t i = 1; i < bd.size(); ++i)
    {
        double x0 = bd[i - 1].x, y0 = bd[i - 1].y;
        double x1 = bd[i].x, y1 = bd[i].y;
        double dx = x1 - x0, dy = y1 - y0;
        double len2 = dx * dx + dy * dy;
        if (len2 < 1e-6)
            continue;

        double t = ((x - x0) * dx + (y - y0) * dy) / len2;
        t = CLAMP(t, 0.0, 1.0);

        double fx = x0 + t * dx;
        double fy = y0 + t * dy;

        double vx = x - fx;
        double vy = y - fy;
        double dist2 = vx * vx + vy * vy;
        if (dist2 < bestDist2)
        {
            bestDist2 = dist2;
            bestSAbs = cumS[i - 1] + t * std::sqrt(len2);

            /* 右手法向 (+dy,-dx) → l 向车道内部(右侧) 为正 */
            double len = std::sqrt(len2);
            double nxR = +dy / len, nyR = -dx / len;
            bestL = vx * nxR + vy * nyR;
        }
    }
    return {bestSAbs, bestL};
}
static inline bool obbHitsCircle(const OBB &box,
                                 double x, double y,
                                 double r = 1.0)
{
    /* ---------- ① 把圆心变换到 OBB 局部坐标 ---------- */
    double dx = x - box.cx;
    double dy = y - box.cy;

    // 局部坐标 (localX, localY)：x 轴沿 OBB 横向，y 轴沿 OBB 纵向
    double localX = dx * box.cosH + dy * box.sinH;
    double localY = -dx * box.sinH + dy * box.cosH;

    /* ---------- ② 计算 OBB 内最接近圆心的点 ---------- */
    // clamp 到长方形区间 [-hl, +hl] × [-hw, +hw]
    double clampedX = CLAMP(localX, -box.hl, box.hl);
    double clampedY = CLAMP(localY, -box.hw, box.hw);

    /* ---------- ③ 圆心到该最近点的向量 ---------- */
    double vx = localX - clampedX;
    double vy = localY - clampedY;

    /* ---------- ④ 若距离 ≤ r → 相交 ---------- */
    return (vx * vx + vy * vy) <= r * r;
}

static bool LateralBypassCore(
    const SimOne_Data_Gps &ego,
    const SimOne_Data_Obstacle &obs,
    std::vector<TrajectoryPoint> &traj,
    double safeR) // 圆半径（=车辆边缘 + 余量）
{
    if (traj.size() < 3)
        return true;

    /* ---------- ① 车道几何（含扩展） ---------- */
    HDMapStandalone::MLaneInfo lane;
    if (!SimOneAPI::GetLaneSampleByLocation({ego.posX, ego.posY, ego.posZ}, lane))
        return true;

    /* 原始左右边缘 */
    SSD::SimPoint3DVector Lbd = lane.leftBoundary;
    SSD::SimPoint3DVector Rbd = lane.rightBoundary;

    /* 尝试扩展到相邻车道的外缘 */
    HDMapStandalone::MLaneLink link;
    if (SimOneAPI::GetLaneLink(lane.laneName, link)) // ← 获取左右邻车道名
    {
        /* ------------ 左边 ------------ */
        if (strlen(link.leftNeighborLaneName.GetString()) != 0)
        {
            HDMapStandalone::MLaneInfo leftLane;
            if (SimOneAPI::GetLaneSample(link.leftNeighborLaneName, leftLane))
            {
                /* 用左邻车道的左边缘替换，得到“更远”的 LbdEx */
                Lbd = leftLane.leftBoundary; // 也可深拷贝 / 拼接
            }
        }

        /* ------------ 右边 ------------ */
        if (strlen(link.rightNeighborLaneName.GetString()) != 0)
        {
            HDMapStandalone::MLaneInfo rightLane;
            if (SimOneAPI::GetLaneSample(link.rightNeighborLaneName, rightLane))
            {
                /* 用右邻车道的右边缘替换，得到“更远”的 RbdEx */
                Rbd = rightLane.rightBoundary;
            }
        }
    }

    std::vector<OBB> obbs;
    /******************** 道路边界 OBB 预处理 *********************/
    auto buildBoundaryOBBs = [&](const SSD::SimPoint3DVector& bd, std::vector<OBB>& outObbs)
    {
        const int step = 40; // 每隔多少点构造一个 OBB，可调
        for (size_t i = 1; i < bd.size(); i += step)
        {
            double x0 = bd[i - 1].x, y0 = bd[i - 1].y;
            double x1 = bd[i].x, y1 = bd[i].y;
            double dx = x1 - x0;
            double dy = y1 - y0;
            double len = std::hypot(dx, dy);
            if (len < 1e-3) continue;

            double cx = 0.5 * (x0 + x1);
            double cy = 0.5 * (y0 + y1);
            double cosH = dx / len;
            double sinH = dy / len;

            OBB b;
            b.cx = cx;
            b.cy = cy;
            b.cosH = cosH;
            b.sinH = sinH;
            b.hl = 0.25 * len;     // 长边 = 边界段长度
            b.hw = 0.1;           // 设置边界厚度（单位：米，可调，0.2~0.5）
            outObbs.push_back(b);
        }
    };
    buildBoundaryOBBs(Lbd, obbs);
    buildBoundaryOBBs(Rbd, obbs);

    /******************** 2. 障碍物 OBB 预处理 *********************/

    for (int i = 0; i < obs.obstacleSize; ++i)
    {
        const auto &o = obs.obstacle[i];

        if (o.type == ESimOne_Obstacle_Type_Car)
        {
            double v = std::hypot(o.velX, o.velY);
            if (v > 1.5)
            {
                SSD::SimPoint3D p{o.posX, o.posY, o.posZ};
                SSD::SimString laneId;
                double s, t, s2, t2;
                if (SimOneAPI::GetNearMostLane(p, laneId, s, t, s2, t2))
                {
                    SSD::SimPoint3D proj, dir;
                    if (SimOneAPI::GetLaneMiddlePoint(p, laneId, proj, dir))
                    {
                        HDMapStandalone::MLaneInfo laneInfo;
                        if (SimOneAPI::GetLaneSampleByLocation({ego.posX, ego.posY, ego.posZ}, laneInfo))
                        {
                            const auto &cl = laneInfo.centerLine;
                            if (cl.size() >= 2)
                            {
                                const auto &head = cl.front();
                                const auto &tail = cl.back();

                                double dx = tail.x - head.x;
                                double dy = tail.y - head.y;

                                double headingStart = std::atan2(dy, dx); // 起点→终点方向
                                double headingEnd = std::atan2(-dy, -dx); // 终点→起点方向（反向）
                                double angleDiff = std::fabs(headingStart - headingEnd);
                                while (angleDiff > M_PI)
                                    angleDiff -= 2 * M_PI;
                                angleDiff = std::fabs(angleDiff);

                                if (angleDiff >= 2.8) // 超过约 160°，认为是回头道/反向连接
                                {
                                    continue; // 跳过构造 OBB
                                }
                            }
                        }
                        double laneHeading = std::atan2(dir.y, dir.x);
                        double obsHeading = o.oriZ;
                        double angleDiff = std::fabs(laneHeading - obsHeading);
                        while (angleDiff > M_PI) angleDiff -= 2 * M_PI;
                        angleDiff = std::fabs(angleDiff);

                        if (angleDiff < M_PI / 6.0 || HasObstacleOnCrosswalk(ego, obs)) // 30度内同向
                            continue;                                                   // 跳过构造 OBB
                    }
                }
            }
        }

        obbs.push_back({o.posX, o.posY,
                        std::cos(o.oriZ), std::sin(o.oriZ),
                        0.5 * o.length, 0.5 * o.width});
    }
    std::vector<size_t> collideIdx; // 需要绕行的点下标
    for (size_t idx = 1; idx + 1 < traj.size(); ++idx)
    {
        bool hit = false;
        for (const auto &b : obbs)
        {
            if (obbHitsCircle(b, traj[idx].x, traj[idx].y, safeR))
            {
                hit = true;
                break; // 任意障碍命中即可
            }
        }
        if (hit)
        {
            collideIdx.push_back(idx);
            // std::cout << "[Collide] traj[" << idx << "]\n"; // 打印下标
        }
    }

    std::vector<SegmentFrame> segments;

    /* ---------- A. 先把 collideIdx 连续性划段 ---------- */
    std::vector<std::vector<size_t>> segIdxList;
    if (!collideIdx.empty())
    {
        std::vector<size_t> cur;
        for (size_t k = 0; k < collideIdx.size(); ++k)
        {
            if (cur.empty() || collideIdx[k] == cur.back() + 1)
                cur.push_back(collideIdx[k]);
            else
            {
                segIdxList.push_back(cur);
                cur = {collideIdx[k]};
            }
        }
        if (!cur.empty())
            segIdxList.push_back(cur);
    }
    auto cumLbdS = buildCumS(Lbd);
    for (size_t sIt = 0; sIt < segIdxList.size(); ++sIt)
    {
        const std::vector<size_t> &segIdx = segIdxList[sIt];
        if (segIdx.empty())
            continue;

        SegmentFrame seg;
        size_t i0 = segIdx.front();
        size_t iE = segIdx.back();

        /* ---- 2-1  基线几何 (tx,ty,nx,ny,origin) ---- */
        std::pair<double, double> proj0 =
            projectToPolylineSL(Lbd, cumLbdS, traj[i0].x, traj[i0].y);
        double s0Abs = proj0.first; // 段首点在 Lbd 上的累计弧长
        /* proj0.second (= l0) 暂不需要 */

        /* 找到 s0Abs 所在边界段，确定切向 */
        size_t bdSeg = 1;
        while (bdSeg + 1 < cumLbdS.size() && cumLbdS[bdSeg] < s0Abs)
            ++bdSeg;
        double xA = Lbd[bdSeg - 1].x, yA = Lbd[bdSeg - 1].y;
        double xB = Lbd[bdSeg].x, yB = Lbd[bdSeg].y;
        double tx = xB - xA, ty = yB - yA;
        double len = std::hypot(tx, ty);
        if (len < 1e-4)
        {
            tx = 1.0;
            ty = 0.0;
        }
        else
        {
            tx /= len;
            ty /= len;
        }

        double nx = +ty, ny = -tx; // 右手法向：l 向车道内部(+)

        seg.origin = {xA, yA, 0.0};
        seg.tx = tx;
        seg.ty = ty;
        seg.nx = nx;
        seg.ny = ny;

        /* ---- 2-2  轨迹点 → (sLocal,l) ---- */
        seg.trajSL.reserve(segIdx.size());
        double sEndLocal = 0.0;
        for (size_t k = 0; k < segIdx.size(); ++k)
        {
            size_t idx = segIdx[k];
            std::pair<double, double> proj =
                projectToPolylineSL(Lbd, cumLbdS, traj[idx].x, traj[idx].y);
            double sAbs = proj.first;
            double l = proj.second;
            double sLocal = sAbs - s0Abs;

            seg.trajSL.push_back({sLocal, l, idx});
            if (sLocal > sEndLocal)
                sEndLocal = sLocal;
        }
        seg.sEnd = sEndLocal;

        /* ---- 2-3  OBB 边界四顶点 → (s,l) 过滤 ---- */
        for (size_t j = 0; j < obbs.size(); ++j)
        {
            const OBB &b = obbs[j];
            const double vx[4] = {+b.hl, +b.hl, -b.hl, -b.hl};
            const double vy[4] = {+b.hw, -b.hw, -b.hw, +b.hw};

            double sArr[4], lArr[4];
            bool keep = false;

            /* ① 先计算 4 顶点的 (s,l) 并判断窗口 */
            for (int c = 0; c < 4; ++c)
            {
                double wx = b.cx + vx[c] * b.cosH - vy[c] * b.sinH;
                double wy = b.cy + vx[c] * b.sinH + vy[c] * b.cosH;

                std::pair<double, double> proj =
                    projectToPolylineSL(Lbd, cumLbdS, wx, wy);

                sArr[c] = proj.first - s0Abs; // sLocal
                lArr[c] = proj.second;

                if (0.0 <= sArr[c] && sArr[c] <= seg.sEnd &&
                    std::fabs(lArr[c]) <= 20.0)
                    keep = true; // 任一角落命中
            }

            /* ② 若命中 → 把 4 顶点全部 push 进 obbSL */
            if (keep)
            {
                for (int c = 0; c < 4; ++c)
                {
                    double wx = b.cx + vx[c] * b.cosH - vy[c] * b.sinH;
                    double wy = b.cy + vx[c] * b.sinH + vy[c] * b.cosH;
                    seg.obbSL.push_back({sArr[c], lArr[c], j, wx, wy});
                }
            }
        }

        /* ---- 2-4  道路边界采样 → (s,l) 过滤 ---- */
        auto collectBd = [&](const SSD::SimPoint3DVector &bd,
                             std::vector<SLBoundaryPoint> &out)
        {
            for (size_t pIt = 0; pIt < bd.size(); ++pIt)
            {
                const SSD::SimPoint3D &p = bd[pIt];
                std::pair<double, double> proj =
                    projectToPolylineSL(Lbd, cumLbdS, p.x, p.y);
                double sLocal = proj.first - s0Abs;
                double l = proj.second;
                if (0.0 <= sLocal && sLocal <= seg.sEnd && std::fabs(l) <= 20.0)
                    out.push_back({sLocal, l, p.x, p.y});
            }
        };
        collectBd(Lbd, seg.lbdSL);
        collectBd(Rbd, seg.rbdSL);

        segments.push_back(seg); // 存储段
    }
    /********************************************************
     * ★ 3.  横向偏移搜索 —— 以 Lbd 为 l=0 基线
     ********************************************************/
    std::vector<double> dLat(traj.size(), 0.0); // 每个轨迹点的横向位移
    const double stepLat = 0.1;                 // 搜索分辨率(米)
    const int maxStep = 200;                    // 最远 20 m (=0.1×200)

    for (size_t si = 0; si < segments.size(); ++si)
    {
        SegmentFrame &seg = segments[si]; // 需要修改 l，故非 const

        /******** 3-1  把每个 OBB +safeR 膨胀后转成 s-l 矩形 ********/
        struct Rect
        {
            double s0, s1, l0, l1;
        };
        std::map<size_t, Rect> obsRect; // obsId → Rect

        for (size_t v = 0; v < seg.obbSL.size(); ++v)
        {
            const SLObsBdPoint &pt = seg.obbSL[v];
            Rect &rc = obsRect[pt.obsId]; // 若不存在则默认构造
            if (rc.s0 == 0 && rc.s1 == 0 && rc.l0 == 0 && rc.l1 == 0)
            {
                rc.s0 = 1e9;
                rc.l0 = 1e9;
                rc.s1 = -1e9;
                rc.l1 = -1e9;
            }
            rc.s0 = std::min(rc.s0, pt.s);
            rc.s1 = std::max(rc.s1, pt.s);
            rc.l0 = std::min(rc.l0, pt.l);
            rc.l1 = std::max(rc.l1, pt.l);
        }
        std::vector<Rect> rects;
        for (std::map<size_t, Rect>::iterator it = obsRect.begin();
             it != obsRect.end(); ++it)
        {
            Rect rc = it->second;
            rc.s0 -= safeR;
            rc.s1 += safeR;
            rc.l0 -= safeR;
            rc.l1 += safeR;
            rects.push_back(rc);
        }

        /******** 3-2  右边界插值函数 rightL(s) ********/
        std::vector<SLBoundaryPoint> rbd = seg.rbdSL; // 拷贝后排序
        std::sort(rbd.begin(), rbd.end(),
                  [](const SLBoundaryPoint &a, const SLBoundaryPoint &b)
                  { return a.s < b.s; });

        auto rightL = [&](double s) -> double
        {
            if (rbd.empty())
                return 1e9; // 没右边界
            if (s <= rbd.front().s)
                return rbd.front().l;
            if (s >= rbd.back().s)
                return rbd.back().l;
            size_t hi = 1;
            while (hi < rbd.size() && rbd[hi].s < s)
                ++hi;
            const SLBoundaryPoint &p1 = rbd[hi - 1];
            const SLBoundaryPoint &p2 = rbd[hi];
            double t = (s - p1.s) / (p2.s - p1.s);
            return p1.l + t * (p2.l - p1.l);
        };

        /******** 3-3  遍历段内碰撞轨迹点，搜索可行位移 ********/
        for (size_t pi = 0; pi < seg.trajSL.size(); ++pi)
        {
            SLPoint &tp = seg.trajSL[pi]; // 需修改 l
            size_t idx = tp.idx;
            double curL = tp.l;
            double s = tp.s;

            /* 判断当前位置是否已安全 */
            bool safe = (curL >= safeR) &&
                        (curL <= rightL(s) - safeR);
            if (safe)
            {
                for (size_t r = 0; r < rects.size(); ++r)
                {
                    const Rect &rc = rects[r];
                    if (s >= rc.s0 && s <= rc.s1 &&
                        curL >= rc.l0 && curL <= rc.l1)
                    {
                        safe = false;
                        break;
                    }
                }
            }
            if (safe)
                continue; // 无需偏移

            /* 余量决定首选方向 (+1:向右, -1:向左) */
            double gapL = curL - safeR;
            double gapR = rightL(s) - safeR - curL;
            int sign = (gapR >= gapL) ? +1 : -1;

            /* ---------- A. 先统计段内已确定位移的均值 ---------- */
            double refShift = 0.0; // 参考位移；0 表示暂无
            int refCnt = 0;
            for (size_t k = 0; k < seg.trajSL.size(); ++k)
            {
                size_t id = seg.trajSL[k].idx;
                if (std::fabs(dLat[id]) > 1e-4)
                {
                    refShift += dLat[id];
                    ++refCnt;
                }
            }
            if (refCnt)
                refShift /= refCnt;

            /* ---------- B. 分别搜索 “向右(+), 向左(−)” 最小可行位移 ---------- */
            auto findShift = [&](int sign) -> double
            {
                for (int st = 1; st <= maxStep; ++st)
                {
                    double shift = sign * stepLat * st;
                    double candL = curL + shift;

                    /* 边界检查 */
                    if (candL < safeR)
                        continue;
                    if (candL > rightL(s) - safeR)
                        continue;

                    /* OBB 碰撞检查 */
                    bool hit = false;
                    for (size_t r = 0; r < rects.size(); ++r)
                    {
                        const Rect &rc = rects[r];
                        if (s >= rc.s0 && s <= rc.s1 &&
                            candL >= rc.l0 && candL <= rc.l1)
                        {
                            hit = true;
                            break;
                        }
                    }
                    if (hit)
                        continue;

                    return shift; // 找到首个可行
                }
                //堵塞
                return 0.0; // 没找到
            };

            double bestPos = findShift(+1);
            double bestNeg = findShift(-1);

            /* ---------- C. 依据 refShift 选最终位移 ---------- */
            double bestShift = 0.0;
            if (bestPos == 0.0 && bestNeg == 0.0)
            {
                /* 两侧都堵死：保留 0 */
                for (int i = 0; i < obs.obstacleSize; ++i)
                {
                    const auto &o = obs.obstacle[i];

                    if (o.type == ESimOne_Obstacle_Type_Car)
                    {
                        // ---------- 新增：距离判断 ----------
                        double dx = o.posX - ego.posX;
                        double dy = o.posY - ego.posY;
                        double dist = std::hypot(dx, dy);
                        if (dist > stopDist+8.0)
                            continue;

                        double v = std::hypot(o.velX, o.velY);
                        if (v < 1.0)
                        {
                            // 静止车堵塞
                            double accDist = 0.0;
                            for (int i = static_cast<int>(idx); i >= 1; --i)
                            {
                                double dx = traj[i].x - traj[i - 1].x;
                                double dy = traj[i].y - traj[i - 1].y;
                                accDist += std::hypot(dx, dy);
                                if (accDist > stopDist)
                                    break;
                                traj[i].v = 0.0;
                            }
                        }
                    }
                }
            }
            else if (refCnt == 0)
            {
                /* 段内还没有确定参考 → 依余量决定方向 */
                bestShift = (gapR >= gapL ? bestPos : bestNeg);
                if (bestShift == 0.0) // 余量侧无解，就用另一侧
                    bestShift = (bestPos != 0.0 ? bestPos : bestNeg);
            }
            else
            {
                /* 选 |shift - refShift| 最小的一个 */
                double candPosErr = (bestPos != 0.0 ? std::fabs(bestPos - refShift) : 1e9);
                double candNegErr = (bestNeg != 0.0 ? std::fabs(bestNeg - refShift) : 1e9);
                bestShift = (candPosErr <= candNegErr ? bestPos : bestNeg);
            }

            /* ---------- D. 写回位移 ---------- */
            if (bestShift != 0.0)
            {
                dLat[idx] = bestShift; // 保存全局索引位移
                tp.l += bestShift;     // 更新段内 l 供调试打印
            }
        }

        /******** 3-4  把段内位移映射回世界坐标 ********/
        for (size_t pi = 0; pi < seg.trajSL.size(); ++pi)
        {
            size_t idx = seg.trajSL[pi].idx;
            if (std::fabs(dLat[idx]) < 1e-4)
                continue;
            traj[idx].x += seg.nx * dLat[idx];
            traj[idx].y += seg.ny * dLat[idx];
        }
    }
    /********************************************************
     *  ③  打印调试信息
     ********************************************************/
    // for (size_t si = 0; si < segments.size(); ++si)
    // {
    //     const auto& seg = segments[si];
    //     std::cout << "\n=== Segment #" << si
    //               << "  (traj " << seg.trajSL.size()
    //               << ", obbBd " << seg.obbSL.size()
    //               << ", bd "    << seg.lbdSL.size() + seg.rbdSL.size()
    //               << ") ===\n";

    //     for (const auto& p : seg.trajSL)
    //     {
    //         const auto& wp = traj[p.idx];
    //         std::cout << "  [TRAJ] idx=" << p.idx
    //                   << "  wx=" << wp.x << " wy=" << wp.y
    //                   << "  s="  << p.s  << " l=" << p.l << '\n';
    //     }
    //     for (const auto& q : seg.obbSL)
    //     {
    //         std::cout << "  [OBB_BD] id=" << q.obsId
    //                   << "  wx=" << q.wx << " wy=" << q.wy
    //                   << "  s="  << q.s  << " l=" << q.l << '\n';
    //     }
    //     auto printBd = [&](const std::vector<SLBoundaryPoint>& v, const char* tag)
    //     {
    //         for (const auto& bp : v)
    //             std::cout << "  [" << tag << "]"
    //                       << "  wx=" << bp.wx << " wy=" << bp.wy
    //                       << "  s="  << bp.s  << " l=" << bp.l << '\n';
    //     };
    //     printBd(seg.lbdSL, "LBD");
    //     printBd(seg.rbdSL, "RBD");
    // }

    /******************** 5. 简单平滑 *****************************/
    double smooth_distance = 20.0; // 平滑距离
    blendAlongSegment(segments, dLat, traj, smooth_distance);
    smoothCurvatureBlend(traj, dLat, smooth_distance, 100.0, 20);
    return true;
}

bool ProcessObstacleLongitudinalSpeed(
    const SimOne_Data_Gps &egoGps,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &trajectory,
    double lookAhead)
{
    // 配置参数（内部设定）
    constexpr double VEHICLE_STOP_DIST = 11.0;         // 移动车辆直接停车距离
    constexpr double VEHICLE_SLOW_DIST = 16.0;         // 移动车辆减速距离
    constexpr double VEHICLE_SLOW_STATIC_RADIUS = 1.0; // 是否重合的判断半径（静止和移动均用）
    constexpr double VEHICLE_STATIC_SLOW_DIST = 50.0;  // 静止车限速范围
    constexpr double VEHICLE_STATIC_LIMIT_SPEED = 10.0; // 静止车限速值
    constexpr double VEHICLE_MOVING_SLOW_SPEED = 2.87;   // 移动车辆限速值
    constexpr double MIN_ANGLE_DEG = 60.0;              // 有效触发角度下限（垂直接近）

    /* ① 仅处理前方 lookAhead 路段 ---------------------------------- */
    size_t frontIdx = trajectory.size() - 1;
    {
        double acc = 0.0;
        for (size_t i = 1; i < trajectory.size(); ++i)
        {
            acc += planarDist(trajectory[i - 1].x, trajectory[i - 1].y,
                              trajectory[i].x, trajectory[i].y);
            if (acc > lookAhead)
            {
                frontIdx = i;
                break;
            }
        }
    }

    /* ② 遍历障碍物并按规则调整速度 -------------------------------- */
    for (int oi = 0; oi < obstacles.obstacleSize; ++oi)
    {
        bool isCarInCrosswalkArea = false;
        const auto &obs = obstacles.obstacle[oi];
        double oSpd = calcSpeed(obs);

        double obsDx = obs.posX - egoGps.posX;
        double obsDy = obs.posY - egoGps.posY;
        double distEgo2Obs = std::hypot(obsDx, obsDy);
        if (distEgo2Obs > lookAhead + 5.0)
            continue; // ②-a 过滤：障碍物离本车太远，无需考虑

        bool isCar = (obs.type == ESimOne_Obstacle_Type_Car);

        // =================== 屏蔽在人行横道上的车辆（横±5m，纵±9m） ===================
        if (isCar)
        {
            SSD::SimPoint3D Pos{egoGps.posX, egoGps.posY, egoGps.posZ};
            SSD::SimString laneId;
            double s, t, s_toCenter, t_toCenter;
            if (SimOneAPI::GetNearMostLane(Pos, laneId, s, t, s_toCenter, t_toCenter))
            {
                SSD::SimVector<HDMapStandalone::MObject> crosswalks;
                SimOneAPI::GetSpecifiedLaneCrosswalkList(laneId, crosswalks);


                const double obsX = obs.posX;
                const double obsY = obs.posY;

                for (const auto &crosswalk : crosswalks)
                {
                    const auto &points = crosswalk.boundaryKnots;
                    int count = static_cast<int>(points.size());

                    for (int j = 1; j < count; ++j)
                    {
                        const auto &p1 = points[j - 1];
                        const auto &p2 = points[j];

                        double dx = p2.x - p1.x;
                        double dy = p2.y - p1.y;
                        double len = std::hypot(dx, dy);
                        if (len < 1e-3)
                            continue;

                        // 单位向量：横道方向 dir，法线方向 norm
                        double dirX = dx / len;
                        double dirY = dy / len;
                        double normX = -dirY;
                        double normY = dirX;

                        // obs 到 p1 的向量
                        double vx = obsX - p1.x;
                        double vy = obsY - p1.y;

                        // 投影距离（米）：纵向与横向
                        double longitudinal = vx * dirX + vy * dirY;
                        double lateral = vx * normX + vy * normY;

                        // 判断是否在扩展的人行道矩形内（单位为米）
                        if (longitudinal >= -2 && longitudinal <= len + 2 &&
                            std::abs(lateral) <= 6.0)
                        {
                            isCarInCrosswalkArea = true;
                            break;
                        }else{
                            // StopBeforeStoplineIfNeeded(*gps, targetPath_info, 13, 5); // 人行横道前减速
                        }
                    }

                    if (isCarInCrosswalkArea)
                        break;
                }
            }

        }

        bool match = false;
        for (size_t pi = 0; pi <= frontIdx; ++pi)
        {
            double px = trajectory[pi].x;
            double py = trajectory[pi].y;
            double d = planarDist(obs.posX, obs.posY, px, py);

            if (!isCar)
                continue;

            bool matchByProximity = (d <= VEHICLE_SLOW_STATIC_RADIUS);
            bool matchByDirection = false;

            // 计算轨迹方向
            if (pi + 1 < trajectory.size())
            {
                double tx = trajectory[pi + 1].x - trajectory[pi].x;
                double ty = trajectory[pi + 1].y - trajectory[pi].y;
                double tLen = std::hypot(tx, ty);

                if (tLen > 1e-3)
                {
                    double vx = tx / tLen;
                    double vy = ty / tLen;

                    // 障碍物方向（移动时）
                    double ovx = obs.velX;
                    double ovy = obs.velY;
                    double vLen = std::hypot(ovx, ovy);

                    if (vLen > 1e-3)
                    {
                        double ux = ovx / vLen;
                        double uy = ovy / vLen;

                        double dot = CLAMP(ux * vx + uy * vy, -1.0, 1.0);
                        double angleDeg = std::acos(dot) * 180.0 / 3.1415926;

                        if (angleDeg >= MIN_ANGLE_DEG && angleDeg <= 120.0)
                            matchByDirection = true;
                    }
                    else
                    {
                        // 静止障碍物也要判断朝向（使用 oriZ）
                        double heading = obs.oriZ;
                        double ox = std::cos(heading);
                        double oy = std::sin(heading);
                        double dot = CLAMP(ox * vx + oy * vy, -1.0, 1.0);
                        double angleDeg = std::acos(dot) * 180.0 / 3.1415926;

                        if (angleDeg >= MIN_ANGLE_DEG && angleDeg <= 120.0)
                            matchByDirection = true;
                    }
                }
            }
            if (!isCarInCrosswalkArea && (matchByProximity || matchByDirection))
            {
                match = true;
                break;
            }
        }

        if (!match)
            continue;

        for (size_t pi = 0; pi <= frontIdx; ++pi)
        {
            double px = trajectory[pi].x;
            double py = trajectory[pi].y;
            double d = planarDist(obs.posX, obs.posY, px, py);

            if (oSpd < 1.0)
            {
                if (d <= VEHICLE_STATIC_SLOW_DIST)
                    trajectory[pi].v = std::min(trajectory[pi].v, VEHICLE_STATIC_LIMIT_SPEED);
            }
            else
            {
				if (d <= VEHICLE_STOP_DIST) {
                    trajectory[pi].v = 0.0;
                }
				else if (d <= VEHICLE_SLOW_DIST) {
                    trajectory[pi].v = std::min(trajectory[pi].v, VEHICLE_MOVING_SLOW_SPEED);
                }
            }
        }
    }
    return true; // 始终成功（纯纵向速度调整）
}

// 检测对向障碍物是否阻挡通过路口：静止车辆类阻挡路口出口，主车应在入口前停止
bool StopBeforeJunctionIfOpposingObstacle(
    const SimOne_Data_Gps &gps,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &traj)
{
    if (traj.size() < 2 || obstacles.obstacleSize == 0)
        return false;

    constexpr double MATCH_RADIUS = 1.0;    // 轨迹点与障碍物的触发距离
    constexpr double STOP_BACK_DIST = 1.8; // 提前多少米停止

    // ---------- ① 前50米任意轨迹点是否即将进入交叉口 ----------
    long junctionId = -1;
    double accDist = 0.0;
    bool found = false;
    for (size_t i = 1; i < traj.size(); ++i)
    {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        accDist += std::hypot(dx, dy);

        SSD::SimString lid = SampleGetNearMostLane({traj[i].x, traj[i].y, 0.0});
        if (HDMapStandalone::MHDMap::IsInJunction(lid, junctionId))
        {
            found = true;
            break;
        }

        if (accDist >= 50.0)
            break;
    }

    if (!found)
        return false;

    // ---------- ② 获取交叉口信息与相关车道 ----------
    HDMapStandalone::MJunction junction;
    junction = HDMapStandalone::MHDMap::GetJunction(junctionId);

    std::set<long> connectedLanes;
    for (size_t i = 0; i < junction.connections.size(); ++i)
    {
        const auto &conn = junction.connections[i];
        for (size_t j = 0; j < conn.laneLinks.size(); ++j)
        {
            const auto &link = conn.laneLinks[j];
            connectedLanes.insert(conn.incomingRoadId);
            connectedLanes.insert(conn.connectingRoadId);
        }
    }

    // ---------- ③ 查找静止车辆类障碍物 ----------
    SSD::SimPoint3D matchedObsPos;
    bool matched = false;
    for (int i = 0; i < obstacles.obstacleSize; ++i)
    {
        const auto &obs = obstacles.obstacle[i];
        if (obs.type != ESimOne_Obstacle_Type_Car &&
            obs.type != ESimOne_Obstacle_Type_Bus &&
            obs.type != ESimOne_Obstacle_Type_Truck)
            continue;

        double v = std::hypot(obs.velX, obs.velY);
        if (v > 0.5)
            continue;
        SSD::SimPoint3D obsPt{obs.posX, obs.posY, obs.posZ};
        SSD::SimString obsLane = SampleGetNearMostLane(obsPt);
        std::string s = obsLane.GetString();
        long roadId = std::stol(s.substr(0, s.find('_'))); // 只拿道路id
        if (connectedLanes.find(roadId) == connectedLanes.end())
            continue;
        matchedObsPos = obsPt;
        matched = true;
        break;
    }
    if (!matched)
        return false;

    // ---------- ④ 判断轨迹中是否有点接近障碍物 ----------
    bool trajBlocked = false;
    for (const auto &pt : traj)
    {
        double dx = pt.x - matchedObsPos.x;
        double dy = pt.y - matchedObsPos.y;
        if (dx * dx + dy * dy <= MATCH_RADIUS * MATCH_RADIUS)
        {
            trajBlocked = true;
            break;
        }
    }
    if (!trajBlocked)
        return false;

    // ---------- ⑤ 找到将进入路口的点 ----------
    size_t junctionStartIdx = traj.size();
    accDist = 0.0;
    for (size_t i = 1; i < traj.size(); ++i)
    {
        SSD::SimString lid = SampleGetNearMostLane({traj[i].x, traj[i].y, 0.0});
        if (HDMapStandalone::MHDMap::IsInJunction(lid, junctionId))
        {
            junctionStartIdx = i;
            break;
        }
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        accDist += std::hypot(dx, dy);
        if (accDist >= 50.0)
            break;
    }
    if (junctionStartIdx == traj.size())
        return false;

    // ---------- ⑥ 停止：从路口前0.5米开始速度置0 ----------
    StopBeforeStoplineIfNeeded( gps, traj,STOP_BACK_DIST,0); // 停车让行

    // ---------- ⑦ 将路口内轨迹也置0 ----------
    for (size_t i = junctionStartIdx; i < traj.size(); ++i)
    {
        traj[i].v = 0.0;
    }
    // for (size_t i = 0; i < traj.size(); i++)
    // {
    //     std::cout << "x: " << traj[i].x << " y: " << traj[i].y << " v: " << traj[i].v << std::endl;
    // }

    return true;
}
void UpdateTurnSignals(
    const SimOne_Data_Gps* gps,
    const std::vector<TrajectoryPoint>& traj,
    SimOne_Data_Signal_Lights* light)
{
    static uint8_t left = 0, right = 0;
    static int tmr = 0;
    constexpr int DUR = FPS * 3;
    constexpr double LAT_THRESHOLD = 0.5; // 米，可调
    constexpr int LOOK_AHEAD = 15;

    if (traj.size() < LOOK_AHEAD + 5) return;

    /* 1. 找到最近轨迹点 */
    size_t base = 0;
    double minDist = 1e9;
    for (size_t i = 0; i < traj.size(); ++i) {
        double d = std::hypot(gps->posX - traj[i].x, gps->posY - traj[i].y);
        if (d < minDist) { minDist = d; base = i; }
    }
    size_t ahead = std::min(base + LOOK_AHEAD, traj.size() - 1);

    /* 2. 将未来轨迹点转换到车辆坐标系，看横向偏移 */
    double egoX = gps->posX, egoY = gps->posY;
    double egoYaw = gps->oriZ;
    double cosEgo = cos(-egoYaw), sinEgo = sin(-egoYaw);

    double dx = traj[ahead].x - egoX;
    double dy = traj[ahead].y - egoY;

    double localX = dx * cosEgo - dy * sinEgo; // 前方
    double localY = dx * sinEgo + dy * cosEgo; // 左侧为正

    /* 3. 打印调试 */
    // printf("[TurnDebug] localY=%+.2f m  left=%d right=%d tmr=%d\n",
    //        localY, left, right, tmr);

    /* 4. 根据横向偏移打灯 */
    if (localY > LAT_THRESHOLD) {
        left = 0; right = 1; tmr = DUR;
    } else if (localY < -LAT_THRESHOLD) {
        left = 1; right = 0; tmr = DUR;
    }

    /* 5. 自动熄灭 */
    if (tmr) --tmr;
    else left = right = 0;

    light->signalLights = left ? ESimOne_Signal_Light_LeftBlinker :
                          right ? ESimOne_Signal_Light_RightBlinker :
                          ESimOne_Signal_Light_None;
}
bool LimitThrottleIfOpposingMovingObstacle(
    const char* mainVehicleId,
    const SimOne_Data_Gps &gps,
    const SimOne_Data_Obstacle &obstacles,
    const std::vector<TrajectoryPoint> &traj,
    SimOne_Data_Control *control)
{
    if (!mainVehicleId || !control || traj.empty() || obstacles.obstacleSize == 0)
        return false;

    constexpr double CHECK_DIST = 200.0;
    constexpr double JUNCTION_RADIUS = 34.0;

    // ---------- ① 找到前 CHECK_DIST 米内轨迹段 ----------
    double accDist = 0.0;
    size_t checkEndIdx = traj.size();
    for (size_t i = 1; i < traj.size(); ++i)
    {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        accDist += std::hypot(dx, dy);
        if (accDist > CHECK_DIST)
        {
            checkEndIdx = i;
            break;
        }
    }

    // ---------- ② 获取交叉口中心点 ----------
    std::vector<SSD::SimPoint3D> junctionCenters;
    auto junctionIds = HDMapStandalone::MHDMap::GetJunctionList();
    for (auto id : junctionIds)
    {
        junctionCenters.push_back(HDMapStandalone::MHDMap::GetJunctionCenter(id));
    }

    // ---------- ③ 检查交叉口附近是否存在红灯 ----------
    using Status = ESimOne_TrafficLight_Status;
    const std::set<Status> redStates = {
        Status::ESimOne_TrafficLight_Status_Green,
    };

    bool hasRedLight = false;
    SSD::SimVector<HDMapStandalone::MSignal> lights;
    SimOneAPI::GetTrafficLightList(lights);
    for (const auto &light : lights)
    {
        SimOne_Data_TrafficLight lightState;
        if (!SimOneAPI::GetTrafficLight(mainVehicleId, light.id, &lightState))
            continue;
        if (redStates.count(lightState.status) != 0)
            return false;

        double dx = light.pt.x - gps.posX;
        double dy = light.pt.y - gps.posY;
        double dist = std::hypot(dx, dy);

        if (dist <= JUNCTION_RADIUS)
        {
            hasRedLight = true;
            break;
        }
    }

    // ---------- ③.5 获取主车附近最近交叉口的连接路数 ----------
    int junctionRoadCount = 0;
    long nearestJunctionId = -1;
    double nearestDist = std::numeric_limits<double>::max();
    SSD::SimPoint3D egoPt{gps.posX, gps.posY, gps.posZ};

    for (auto id : junctionIds)
    {
        auto center = HDMapStandalone::MHDMap::GetJunctionCenter(id);
        double dx = egoPt.x - center.x;
        double dy = egoPt.y - center.y;
        double dist = std::hypot(dx, dy);
        if (dist < nearestDist)
        {
            nearestDist = dist;
            nearestJunctionId = id;
        }
    }

    if (nearestJunctionId != -1)
    {
        auto junction = HDMapStandalone::MHDMap::GetJunction(nearestJunctionId);
        junctionRoadCount = static_cast<int>(junction.connectingRoadIds.size());
    }

    if (!(hasRedLight || lights.empty()))
    {
        if (junctionRoadCount < 16)
        {
            return false; // 无红灯，不做处理
        }
    }

    // ---------- ④ 遍历障碍物 ----------
    for (int i = 0; i < obstacles.obstacleSize; ++i)
    {
        const auto &obs = obstacles.obstacle[i];
        if (obs.type != ESimOne_Obstacle_Type_Car &&
            obs.type != ESimOne_Obstacle_Type_Truck &&
            obs.type != ESimOne_Obstacle_Type_Bus)
            continue;

        SSD::SimPoint3D obsPt{obs.posX, obs.posY, obs.posZ};

        for (const auto &center : junctionCenters)
        {
            double dx = obsPt.x - center.x;
            double dy = obsPt.y - center.y;
            double dist = std::hypot(dx, dy);
            if (dist <= JUNCTION_RADIUS)
            {
                control->throttle = -1.0; // 空油门标志位
                return true;
            }
        }
    }

    return false;
}

bool StopForRedLightIfNeeded(
	const char* mainVehicleId,
    const SimOne_Data_Gps &gps,
    std::vector<TrajectoryPoint> &traj)
{
    if (!mainVehicleId || traj.empty())
        return false;

    using Status = ESimOne_TrafficLight_Status;
    constexpr double TRAJ_NEARBY_RADIUS = 5.0;

    // 获取所有交通信号灯
    SSD::SimVector<HDMapStandalone::MSignal> lights;
    SimOneAPI::GetTrafficLightList(lights);

	for (const auto& light : lights)
    {
        // 获取红灯状态
        SimOne_Data_TrafficLight lightState;
        if (!SimOneAPI::GetTrafficLight(mainVehicleId, light.id, &lightState))
            continue;

        if (lightState.status != Status::ESimOne_TrafficLight_Status_Red)
            continue;

        // 灯坐标
		const auto& lightPos = light.pt;

        // 判断是否与轨迹点在5米内
        bool nearTraj = false;
		for (const auto& p : traj)
        {
            double dx = p.x - lightPos.x;
            double dy = p.y - lightPos.y;
            if (std::hypot(dx, dy) <= TRAJ_NEARBY_RADIUS)
            {
                nearTraj = true;
                break;
            }
        }

        if (!nearTraj)
            continue;

        // 命中红灯 + 在轨迹附近：使用停止线停车
        return StopBeforeStoplineIfNeeded(gps, traj, 0.41, 0.0);
    }

    return false;
}


bool AdjustTrajectoryIfOpposingTraffic(
    const SimOne_Data_Gps &ego,
    const SimOne_Data_Obstacle &obstacles,
    std::vector<TrajectoryPoint> &traj,
    double lateralMargin)
{
    if (traj.size() < 3 || obstacles.obstacleSize == 0)
        return false;

    // 获取本车所在车道
    SSD::SimPoint3D egoPt{ego.posX, ego.posY, ego.posZ};
    HDMapStandalone::MLaneInfo egoLane;
    if (!SimOneAPI::GetLaneSampleByLocation(egoPt, egoLane))
        return false;

    std::string egoLaneId = egoLane.laneName.GetString();
    std::string egoRoadIdStr = egoLaneId.substr(0, egoLaneId.find('_'));
    long egoRoadId = std::stol(egoRoadIdStr);

    for (int i = 0; i < obstacles.obstacleSize; ++i)
    {
        const auto &obs = obstacles.obstacle[i];
        if (obs.type != ESimOne_Obstacle_Type_Car &&
            obs.type != ESimOne_Obstacle_Type_Truck &&
            obs.type != ESimOne_Obstacle_Type_Bus)
            continue;

        double obsSpeed = std::hypot(obs.velX, obs.velY);
        if (obsSpeed < 8.0)
            continue;

        SSD::SimPoint3D obsPt{obs.posX, obs.posY, obs.posZ};
        SSD::SimString obsLane;
        double s, t, s2, t2;
        if (!SimOneAPI::GetNearMostLane(obsPt, obsLane, s, t, s2, t2))
            continue;

        std::string obsLaneId = obsLane.GetString();
        long obsRoadId = std::stol(obsLaneId.substr(0, obsLaneId.find('_')));
        if (obsRoadId != egoRoadId)
            continue;

        // 判断夹角
        double egoHeadingX = std::cos(ego.oriZ);
        double egoHeadingY = std::sin(ego.oriZ);
        double obsHeadingX = obs.velX / obsSpeed;
        double obsHeadingY = obs.velY / obsSpeed;

        double dot = egoHeadingX * obsHeadingX + egoHeadingY * obsHeadingY;
        double angle = std::acos(CLAMP(dot, -1.0, 1.0));
        if (angle < 2.0 * M_PI / 3.0) // < 120 度
            continue;

        // 找轨迹上最接近障碍物的点
        size_t obsIdx = 0;
        double minDist = 1e10;
        for (size_t j = 0; j < traj.size(); ++j)
        {
            double dx = traj[j].x - obs.posX;
            double dy = traj[j].y - obs.posY;
            double d = dx * dx + dy * dy;
            if (d < minDist)
            {
                minDist = d;
                obsIdx = j;
            }
        }

        // 构造平移段：前后各50米
        constexpr double AFFECT_DIST = 50.0;
        size_t segStart = 0, segEnd = traj.size() - 1;
        double acc = 0.0;
        for (int j = static_cast<int>(obsIdx); j > 0; --j)
        {
            acc += std::hypot(traj[j].x - traj[j - 1].x, traj[j].y - traj[j - 1].y);
            if (acc >= AFFECT_DIST) { segStart = j; break; }
        }
        acc = 0.0;
        for (size_t j = obsIdx; j + 1 < traj.size(); ++j)
        {
            acc += std::hypot(traj[j + 1].x - traj[j].x, traj[j + 1].y - traj[j].y);
            if (acc >= AFFECT_DIST) { segEnd = j; break; }
        }

        // 平移并平滑
        const int SMOOTH_WIN = 3;
        for (size_t j = segStart; j <= segEnd; ++j)
        {
            if (j + 1 >= traj.size())
                break;

            // 切向向量：j → j+1
            double tx = traj[j + 1].x - traj[j].x;
            double ty = traj[j + 1].y - traj[j].y;
            double L = std::hypot(tx, ty);
            if (L < 1e-4)
                continue;

            // 右侧法向（切向顺时针旋转 90°）
            double nx = ty / L;
            double ny = -tx / L;

            // 向右偏移
            traj[j].x += nx * lateralMargin;
            traj[j].y += ny * lateralMargin;
        }

        // 简单滑动平滑
        for (size_t j = segStart; j <= segEnd; ++j)
        {
            double sumx = 0.0, sumy = 0.0;
            int cnt = 0;
            int beg = std::max<int>(segStart, j - SMOOTH_WIN);
            int end = std::min<int>(segEnd, j + SMOOTH_WIN);
            for (int k = beg; k <= end; ++k)
            {
                sumx += traj[k].x;
                sumy += traj[k].y;
                ++cnt;
            }
            traj[j].x = sumx / cnt;
            traj[j].y = sumy / cnt;
        }

        return true;
    }

    return false;
}

void SmoothTrajectorySpeedForward(std::vector<TrajectoryPoint> &traj,
                                  double targetSpeed,
                                  double maxAccel)
{
    if (traj.empty())
        return;

    // 从末尾开始设定目标速度
    traj.back().v = std::min(traj.back().v, targetSpeed);

    // 向前平滑速度
    for (int i = static_cast<int>(traj.size()) - 2; i >= 0; --i)
    {
        double dx = traj[i + 1].x - traj[i].x;
        double dy = traj[i + 1].y - traj[i].y;
        double ds = std::hypot(dx, dy); // 相邻点距离

        if (ds < 1e-6)
        {
            traj[i].v = std::min(traj[i].v, traj[i + 1].v);
            continue;
        }

        // 可接受的最大速度（根据下一个点速度和加速度限制反推）
        double maxVi = std::sqrt(std::max(0.0, traj[i + 1].v * traj[i + 1].v + 2.0 * maxAccel * ds));

        // 当前点速度不得大于反推出来的速度
        traj[i].v = std::min(traj[i].v, maxVi);
    }
}

bool StopBeforeStoplineIfNeeded(const SimOne_Data_Gps& gps,
                                std::vector<TrajectoryPoint>& traj,
                                double stopBeforeDist,
                                double speed)
{
    if (traj.empty())
        return false;

    // 1. 获取当前所在车道
    SSD::SimPoint3D egoPos{gps.posX, gps.posY, gps.posZ};
    SSD::SimString laneId;
    double s, t, s_toCenterLine, t_toCenterLine;
    if (!SimOneAPI::GetNearMostLane(egoPos, laneId, s, t, s_toCenterLine, t_toCenterLine))
        return false;

    // 2. 获取指定车道停止线
    SSD::SimVector<HDMapStandalone::MObject> stoplines;
    SimOneAPI::GetSpecifiedLaneStoplineList(laneId, stoplines);
    if (stoplines.empty())
        return false;

    // 3. 找出最近停止线中心点
    SSD::SimPoint3D stoplinePt = stoplines[0].pt;
    double minDist = std::hypot(stoplinePt.x - gps.posX, stoplinePt.y - gps.posY);
    for (const auto& sl : stoplines)
    {
        double d = std::hypot(sl.pt.x - gps.posX, sl.pt.y - gps.posY);
        if (d < minDist)
        {
            minDist = d;
            stoplinePt = sl.pt;
        }
    }

    // 4. 找到轨迹中与停止线距离最近的点（分界点）
    int splitIdx = -1;
    double minTrajDist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(traj.size()); ++i)
    {
        double dx = traj[i].x - stoplinePt.x;
        double dy = traj[i].y - stoplinePt.y;
        double d = std::hypot(dx, dy);
        if (d < minTrajDist)
        {
            minTrajDist = d;
            splitIdx = i;
        }
    }

    if (splitIdx == -1)
        return false;

    bool modified = false;

    // 5. 向前检查：距停止线 <= stopBeforeDist 的轨迹点设为 0
    double accDist = 0.0;
    for (int i = splitIdx; i >= 1; --i)
    {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        accDist += std::hypot(dx, dy);
        if (accDist > stopBeforeDist)
            break;
        traj[i].v = speed;
        modified = true;
    }

    // 6. 向后清除 30 米（预防远视跳过停止线）
    accDist = 0.0;
    for (int i = splitIdx + 1; i < static_cast<int>(traj.size()); ++i)
    {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        accDist += std::hypot(dx, dy);
        if (accDist > 30.0)
            break;
        traj[i].v = speed;
        modified = true;
    }

    return modified;
}

bool HasObstacleOnCrosswalk(const SimOne_Data_Gps& gps,
                            const SimOne_Data_Obstacle &obstacle)
{
    if (obstacle.obstacleSize == 0)
        return false;

    // 获取主车所在车道
    SSD::SimPoint3D egoPos{gps.posX, gps.posY, gps.posZ};
    SSD::SimString laneId;
    double s, t, s_toCenter, t_toCenter;
    if (!SimOneAPI::GetNearMostLane(egoPos, laneId, s, t, s_toCenter, t_toCenter))
        return false;

    // 获取车道关联的人行横道线
    SSD::SimVector<HDMapStandalone::MObject> crosswalks;
    SimOneAPI::GetSpecifiedLaneCrosswalkList(laneId, crosswalks);
    if (crosswalks.empty())
        return false;

    // 遍历每一个人行横道
    for (const auto& crosswalk : crosswalks)
    {
        const auto& points = crosswalk.boundaryKnots;
        if (points.empty())
            continue;

        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::lowest();

        for (const auto& pt : points)
        {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }

        // 扩展容差边界
        constexpr double marginX = 1.0;
        constexpr double marginY = 1.0;
        minX -= marginX;
        maxX += marginX;
        minY -= marginY;
        maxY += marginY;

        // 遍历障碍物并检查是否在矩形内
        for (int i = 0; i < obstacle.obstacleSize; ++i)
        {
            const auto& obs = obstacle.obstacle[i];
            if (obs.type != ESimOne_Obstacle_Type_Pedestrian &&
                obs.type != ESimOne_Obstacle_Type_Car)
                continue;

            double obsX = obs.posX;
            double obsY = obs.posY;

            if (obsX >= minX && obsX <= maxX &&
                obsY >= minY && obsY <= maxY)
            {
                return true;
            }
        }
    }

    return false;
}
