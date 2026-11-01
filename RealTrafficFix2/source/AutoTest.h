/*
    AutoTest: automated test drive used to find traffic problems with evidence instead of guesses.

    Only active when scripts\RealTrafficFix.autotest.ini exists. The player rides as a passenger of an ordinary
    AI car (wandering, driven by the game AI + this mod) that is moved through several places of the map.
    Every collision between vehicles is logged with the last 3 seconds of both vehicles (speed, target,
    limiter, what the game asked on the pedals and what was applied after the mod, obstacle, light,
    temp action...). Screenshots are taken on collisions near the camera and periodically.

    Mode = mod     -> normal mod behaviour
    Mode = vanilla -> the mod doesn't drive any vehicle (baseline for comparison)
    Included by RealTrafficFix.cpp (uses its helpers and statics).
*/
#pragma once
#include "extensions/ScriptCommands.h"
#include "CSprite2d.h"
#include "CRect.h"
#include "JPegCompress.h"
#include <cstdio>
#include <ctime>
#include <map>

namespace AutoTest {

using namespace plugin;

// A point the route trip goes through. kind: 0 any road, 1 a freeway (the nearest class 2 node), 2 a parking place (the
// nearest parking node: the car stops there)
struct Waypoint { const char* name; float x, y; int kind; };
// highway: start on a class 2 (freeway) node. route: drive from here to (destX, destY) with the mod's route, no resets,
// through the waypoints first; park: the end is a parking place where the car stops
struct Location {
    const char* name; float x, y, z; bool highway = false; bool route = false; float destX = 0.0f, destY = 0.0f;
    const Waypoint* waypoints = nullptr; int numWaypoints = 0; bool park = false;
};
// Bayside -> the toll at the south end of the Gant Bridge (toll_sfw) -> the road past Mount Chiliad with the Whetstone
// truck stop (cw_fuelpay) -> Santa Maria Beach -> the freeway by the LS airport -> a parking place downtown
static const Waypoint baysideToLS[] = {
    { "SF toll (Gant Bridge)",        -2681.0f,  1275.0f, 0 },
    { "Whetstone gas station",        -1622.0f, -2692.0f, 0 },
    { "Santa Maria Beach",              400.0f, -1780.0f, 0 },
    { "LS airport freeway",            1700.0f, -2150.0f, 1 },
};
static const Location locations[] = {
    { "LS Ganton/Grove",        2440.0f, -1660.0f, 13.0f },
    { "LS Downtown",            1480.0f, -1740.0f, 13.0f },
    { "LS Market",              1150.0f, -1400.0f, 13.0f },
    { "LS Rodeo",                420.0f, -1440.0f, 30.0f },
    { "LS Idlewood",            1900.0f, -1750.0f, 13.0f },
    { "Red County Montgomery",  1300.0f,   250.0f, 19.0f },
    { "SF Downtown",           -1800.0f,   700.0f, 30.0f },
    { "SF Doherty",            -2050.0f,   150.0f, 30.0f },
    { "LV The Strip",           2040.0f,  1300.0f, 10.0f },
    { "LV Downtown",            2200.0f,  1700.0f, 10.0f },
    { "LS Commerce",            1500.0f, -1450.0f, 13.0f },
    // Freeways (class 2 path nodes): the car starts on the nearest one within 900 m, or the place is skipped
    { "LS Freeway (Mulholland)", 1620.0f, -1150.0f, 24.0f, true },
    { "LS Freeway (East)",       2650.0f, -1100.0f, 40.0f, true },
    { "LS Freeway (Airport)",    1700.0f, -2150.0f, 15.0f, true },
    { "LV Freeway (Julius E)",   2750.0f,  1700.0f, 10.0f, true },
    { "LV Freeway (Julius N)",   1700.0f,  2400.0f, 10.0f, true },
    { "Red County Freeway",        50.0f, -1500.0f, 15.0f, true },
    { "SF Freeway",             -1700.0f,  -300.0f, 20.0f, true },
    // One long trip through everything: city streets, lights, freeways, ramps, countryside, bridges
    { "Grove Street -> Bayside", 2495.0f, -1668.0f, 13.0f, false, true, -2410.0f, 2330.0f },
    { "Bayside -> LS Downtown",  -2420.0f, 2340.0f, 5.0f, false, true, 1550.0f, -1350.0f, baysideToLS, 4, true },
};
static std::string routeFilter; // Route = part of the trip's name: only that trip (all of them when empty)
static constexpr int NUM_LOCATIONS = sizeof(locations) / sizeof(locations[0]);
static constexpr int HISTORY = 150;              // samples kept per vehicle
static constexpr unsigned int SAMPLE_MS = 100;   // -> 15 s of history

static CVector2D WaypointPos(const Location& l, int i) {
    return i < l.numWaypoints ? CVector2D(l.waypoints[i].x, l.waypoints[i].y) : CVector2D(l.destX, l.destY);
}
static int WaypointKind(const Location& l, int i) { return i < l.numWaypoints ? l.waypoints[i].kind : (l.park ? 2 : 0); }
static const char* WaypointName(const Location& l, int i) { return i < l.numWaypoints ? l.waypoints[i].name : "the end"; }
// Straight-line distance still to go: to the next waypoint, then waypoint to waypoint to the end
static float TripLeft(const Location& l, int wp, const CVector2D& pos) {
    float left = (WaypointPos(l, wp) - pos).Magnitude();
    for (int i = wp; i < l.numWaypoints; i++) left += (WaypointPos(l, i + 1) - WaypointPos(l, i)).Magnitude();
    return left;
}
// The road node a freeway/parking waypoint really is, once the roads there are loaded: the nearest freeway node (class 2)
// or parking node (special function 1/2) within 500 m
static bool SnapWaypoint(CVehicle* veh, const CVector2D& at, int kind) {
    float best = 500.0f;
    CNodeAddress bestAddr;
    for (int a = 0; a < NUM_PATH_MAP_AREAS; a++) {
        if (!ThePaths.m_pPathNodes[a]) continue;
        for (unsigned int i = 0; i < ThePaths.m_dwNumVehicleNodes[a]; i++) {
            CNodeAddress addr(static_cast<short>(a), static_cast<short>(i));
            CPathNode* node = GetNode(addr);
            if (!node || node->WaterNode || node->InteriorNode || node->NumberAdjNodes < 1) continue;
            if (kind == 1 && node->Speed < 2) continue;
            if (kind == 2 && node->SpecialFunction != CPathNode::SPECIAL_PARKING_PARALLEL &&
                node->SpecialFunction != CPathNode::SPECIAL_PARKING_PERPENDICULAR) continue;
            float d = (GetNodePos(node).To2D() - at).Magnitude();
            if (d < best) { best = d; bestAddr = addr; }
        }
    }
    if (bestAddr.IsEmpty()) return false;
    CVector2D pos = GetNodePos(GetNode(bestAddr)).To2D();
    SetFixedDestination(veh, pos);
    vehExData.Get(veh).destNode = bestAddr;
    return true;
}

// Waypoints are areas (120 m, a freeway one through a freeway node within 150 m); the end is a point
static void SetWaypointDestination(CVehicle* veh, const Location& l, int i) {
    int kind = WaypointKind(l, i);
    if (i >= l.numWaypoints) SetFixedDestination(veh, WaypointPos(l, i));
    else SetFixedDestination(veh, WaypointPos(l, i), kind == 1 ? 150.0f : 120.0f, kind == 1);
}

struct Sample {
    unsigned int t;
    float speed, fwd, target, gap, leader, stuck;
    float gameGas, gameBrake, gas, brake;    // FollowPath output before/after the mod
    float pedalGas, pedalBrake;              // vehicle's real pedals
    unsigned char limiter, obstacle, light, temp, style, mission, status;
    char laneCur, laneNext, lanes, laneState, indicator; // lanes: of the current link our way
    int collAge;            // ms since m_nLastCollisionTime
    unsigned char collType; // m_pDamageEntity type (0 none, 1 building, 2 vehicle, 3 ped, 4 object)
    int stuckAge;           // ms since CAutoPilot +0x1C (game stuck timer)
    bool valid, pedalFresh;
    CVector pos;
    float hx, hy;           // forward (2D), for the collision diagrams
    float lightDist;        // m to the light it's watching
};
struct History {
    Sample s[HISTORY];
    int count = 0, head = 0;
    unsigned int lastSample = 0;
    unsigned int firstSeen = 0;     // when the vehicle appeared
    float maxSpeed = 0.0f;          // km/h since it appeared
    // Direction changes (forward <-> backwards), for the stop/reverse/hit cycle
    int lastDir = 0;
    unsigned int flips[4] = {};
    int flipCount = 0;
    unsigned int lastOscillationEvent = 0;
};

static bool enabled = false;
bool vanilla = false;
static std::string dir, shotDir;
static std::fstream out;
static int secondsPerLocation = 66;
static float minIntensity = 0.0f;
static bool exitWhenDone = true;
static int carModel = 426; // Premier
static int maxLocations = NUM_LOCATIONS;
static bool stopOnOscillation = true;
static bool highwayOnly = false; // only the freeway places
static bool routeMode = false;   // only the route trip (Scenario = route)
// Scenario = scan: the route trip without traffic at up to 80 km/h, scanning the road like a laser car: every 2 m a fan of
// rays down across the road (-30..+30 m every 0.5 m) records the ground height and surface. With the lanes the game has
// there, tools\route_scan.py finds where its lane data doesn't match the real road (RealTrafficFix.routefixes.txt)
static bool scanMode = false;
static std::ofstream scanOut;
static CVector lastScanPos;
static int scanSamples = 0;
// The trip: two cars (a normal and an aggressive driver) from the same place to the same destination, at the same time.
// The player rides in one of them; Left/Right arrow switch to the other one
struct TripCar {
    const char* label;
    eDriverProfile profile;
    int handle = -1, driver = -1;
    CVehicle* veh = nullptr;
    CVector lastPos;
    float driven = 0.0f, still = 0.0f, bestLeft = 1e9f, doneMinutes = 0.0f;
    int wp = 0;                  // waypoint it's heading to (numWaypoints: the end)
    bool snapped = false;        // that waypoint's road node chosen (freeway / parking)
    unsigned int snapTry = 0, nearSince = 0;
    unsigned int lastProgress = 0;
    bool done = false;
    const char* result = "";
    int collisions = 0;
    int colour = 1;     // car colour id (1 white, 3 red)
    int blip = -1;      // radar blip
};
static TripCar tripCars[2] = { { "NORMAL", PROFILE_NORMAL }, { "AGGRESSIVE", PROFILE_AGGRESSIVE } };
static int followed = 0;
static int tripCarCount = 1; // TripCars = 2: also the aggressive driver
// Where the time goes: seconds moving with each speed limiter in charge, freeway / everything else (mod-driven, near the camera)
static double limiterTime[2][NUM_LIMITERS] = {};
static int routeMaxMinutes = 35;
static bool noPeds = true;       // no pedestrians: only the traffic is tested
static unsigned int stopAt = 0;

static std::unordered_map<CVehicle*, History> histories;
// Pairs in contact (bounding boxes touching) and when each pair was last seen apart
static std::map<std::pair<CVehicle*, CVehicle*>, unsigned int> contactPairs, apartSince;
static int oscillationCount = 0;

enum eState { WAIT_GAME, TELEPORT, WAIT_STREAM, SPAWN, DRIVE, DONE };
static eState state = WAIT_GAME;
static unsigned int stateTime = 0, startTime = 0, locationStart = 0; // locationStart: arrival at the current place
static int locIndex = -1;
static int carHandle = -1, driverHandle = -1;
static CVehicle* testCar = nullptr;
static float distanceDriven = 0.0f, locationDistance = 0.0f;
static CVector lastCarPos;
static float carStillTime = 0.0f, offHighwayTime = 0.0f;
static int collisionCount = 0, shotsTaken = 0;
static std::string pendingShot;
static unsigned int lastShotTime = 0, lastPeriodicShot = 0, lastPulse = 0;
static std::map<std::string, int> typeCounts;
static int involvingModCount = 0, involvingReverseCount = 0;
static std::map<std::string, int> causeCounts;  // probable cause of each driving collision
// Collision analysis: every collision (an "episode": the same pair touching again within 10 s is a repeat of it, not a
// new one), with how hard (closing speed), where, on which road, which drivers. Also written to collisions.csv
struct CollisionRec { float x, y, closing; std::string cause, road, profA, profB; };
static std::vector<CollisionRec> collisionRecs;
static std::map<std::pair<CVehicle*, CVehicle*>, std::pair<unsigned int, int>> pairEpisodes; // last contact, collision #
static int repeatContacts = 0;
static std::ofstream collisionsCsv;
static const char* Severity(float closing) { return closing < 5.0f ? "touch" : (closing < 20.0f ? "light" : "hard"); }
static std::string lastCause;
static int jamCount = 0;

// Driving quality of the AI vehicles near the camera: lane changes, speed by road class, acceleration
struct DriveTrack {
    bool init = false;
    UInt16 nextLink = 0xFFFF;
    char nextLane = 0;
    CVector pos;
    float prevFwd = 0.0f;
    unsigned int prevT = 0;
};
static std::unordered_map<CVehicle*, DriveTrack> driveTracks;
static float aiKm = 0.0f;                        // km driven by the tracked vehicles
static int laneChanges = 0, laneChangesMidLink = 0;
// Road classes from the raw path nodes (independent of the mod's own classification)
enum { RC_SLOW, RC_STREET, RC_AVENUE, RC_HIGHWAY, RC_COUNT };
static const char* rcNames[RC_COUNT] = { "slow nodes (class 0)", "street (class 1, 1 lane)", "avenue (class 1, 2+ lanes)", "highway (class 2)" };
static double rcSpeedSum[RC_COUNT] = {}, rcTime[RC_COUNT] = {};
static std::map<std::string, std::vector<float>> accelSamples, speedSamples; // per driver type
static std::map<std::string, int> driverTypeCounts;
static std::unordered_map<CVehicle*, bool> countedDriver;

static int RoadClassOf(CVehicle* v) {
    const CAutoPilot& ap = v->m_autoPilot;
    CPathNode* curr = GetNode(ap.m_currentAddress);
    CPathNode* next = GetNode(ap.m_startingRouteNode);
    if (!curr || !next) return -1;
    int cls = std::min(curr->Speed, next->Speed);
    if (cls >= 2) return RC_HIGHWAY;
    if (cls <= 0) return RC_SLOW;
    CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
    return link && LanesToward(link, ap.m_startingRouteNode) >= 2 ? RC_AVENUE : RC_STREET;
}

static float Percentile(std::vector<float> v, float p) {
    if (v.empty()) return 0.0f;
    std::sort(v.begin(), v.end());
    return v[std::min(static_cast<size_t>(p * v.size()), v.size() - 1)];
}

static void TrackDriving(CVehicle* v, unsigned int now) {
    if (!v->m_pDriver || v->m_pDriver->IsPlayer() || v == testCar) return;
    int sc = v->m_nVehicleSubClass;
    if (sc != VEHICLE_AUTOMOBILE && sc != VEHICLE_BIKE && sc != VEHICLE_MTRUCK && sc != VEHICLE_QUAD) return;
    if ((v->GetPosition() - TheCamera.GetPosition()).Magnitude() > 150.0f) return;
    const CAutoPilot& ap = v->m_autoPilot;
    if (ap.m_pTargetCar || v->m_nCreatedBy == MISSION_VEHICLE) return;
    DriveTrack& t = driveTracks[v];
    UInt16 link = *reinterpret_cast<const UInt16*>(&ap.m_nNextPathNodeInfo);
    float fwd = CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS;
    std::string type = DriverTypeName(v);
    if (!t.init) {
        t.init = true; t.nextLink = link; t.nextLane = ap.m_nNextLane; t.pos = v->GetPosition(); t.prevFwd = fwd; t.prevT = now;
        if (!countedDriver[v]) { countedDriver[v] = true; driverTypeCounts[type]++; }
        return;
    }
    float step = (v->GetPosition() - t.pos).Magnitude2D();
    if (step < 20.0f) aiKm += step / 1000.0f;
    t.pos = v->GetPosition();

    // Lane changes: the lane on the same link changed (mid-link), or a new link where the old lane still exists
    if (link == t.nextLink) {
        if (ap.m_nNextLane != t.nextLane) { laneChanges++; laneChangesMidLink++; }
    }
    else {
        CPathNode* curr = GetNode(ap.m_currentAddress);
        CCarPathLink* l = curr ? GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode) : nullptr;
        int lanes = l ? LanesToward(l, ap.m_startingRouteNode) : 0;
        if (ap.m_nNextLane != t.nextLane && t.nextLane < lanes) laneChanges++;
    }
    t.nextLink = link;
    t.nextLane = ap.m_nNextLane;

    float dt = (now - t.prevT) / 1000.0f;
    if (dt >= 0.25f) {
        float accel = (fwd - t.prevFwd) / dt;
        bool grounded = v->GetNumContactWheels() >= (sc == VEHICLE_BIKE ? 2 : 4);
        if (grounded && fwd > 1.0f && accel > 0.3f && accel < 15.0f && fwd < 22.0f) accelSamples[type].push_back(accel);
        t.prevFwd = fwd;
        t.prevT = now;
    }
    if (fwd > 1.5f) {
        int rc = RoadClassOf(v);
        const ExtendedData& lx = vehExData.Get(v);
        if (lx.valid && lx.limiter < NUM_LIMITERS) limiterTime[rc == RC_HIGHWAY ? 0 : 1][lx.limiter] += CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
        float stepTime = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
        if (rc >= 0) { rcSpeedSum[rc] += fwd * MPS_TO_KMH * stepTime; rcTime[rc] += stepTime; }
        if (frameCounter % 10 == 0) speedSamples[type].push_back(fwd * MPS_TO_KMH);
    }
}

static unsigned int Now() { return CTimer::m_snTimeInMilliseconds; }
static float Minutes() { return (Now() - startTime) / 60000.0f; }

static std::string WallClock() {
    time_t t = time(nullptr);
    char buf[32];
    tm local;
    localtime_s(&local, &t);
    strftime(buf, sizeof(buf), "%H:%M:%S", &local);
    return buf;
}

void Init(const std::string& moduleDir) {
    dir = moduleDir;
    std::string iniPath = dir + "RealTrafficFix.autotest.ini";
    if (GetFileAttributesA(iniPath.c_str()) == INVALID_FILE_ATTRIBUTES) return;
    CIniReader ini(iniPath);
    enabled = true;
    vanilla = ini.ReadString("AutoTest", "Mode", "mod") == "vanilla";
    secondsPerLocation = ini.ReadInteger("AutoTest", "SecondsPerLocation", secondsPerLocation);
    minIntensity = ini.ReadFloat("AutoTest", "MinIntensity", minIntensity);
    exitWhenDone = ini.ReadInteger("AutoTest", "ExitWhenDone", 1) != 0;
    carModel = ini.ReadInteger("AutoTest", "CarModel", carModel);
    stopOnOscillation = ini.ReadInteger("AutoTest", "StopOnOscillation", 1) != 0;
    maxLocations = std::clamp(ini.ReadInteger("AutoTest", "MaxLocations", maxLocations), 1, NUM_LOCATIONS);
    highwayOnly = ini.ReadInteger("AutoTest", "HighwayOnly", 0) != 0;
    scanMode = ini.ReadString("AutoTest", "Scenario", "") == "scan";
    routeMode = scanMode || ini.ReadString("AutoTest", "Scenario", "") == "route";
    routeFilter = ini.ReadString("AutoTest", "Route", "");
    noPeds = ini.ReadInteger("AutoTest", "NoPeds", 1) != 0;
    tripCarCount = std::clamp(ini.ReadInteger("AutoTest", "TripCars", 1), 1, 2);
    // Driver of the (first) test car
    {
        static const struct { const char* key; eDriverProfile profile; const char* label; } drivers[] = {
            { "calm", PROFILE_CALM, "CALM" }, { "safe", PROFILE_SAFE, "SAFE" }, { "normal", PROFILE_NORMAL, "NORMAL" },
            { "fast", PROFILE_FAST, "FAST" }, { "aggressive", PROFILE_AGGRESSIVE, "AGGRESSIVE" },
            { "fleeing", PROFILE_FLEEING, "FLEEING" }, { "wheelspin", PROFILE_WHEELSPIN, "WHEELSPIN" } };
        std::string want = ini.ReadString("AutoTest", "TripProfile", "normal");
        for (const auto& d : drivers) if (want == d.key) { tripCars[0].profile = d.profile; tripCars[0].label = d.label; }
    }
    routeMaxMinutes = ini.ReadInteger("AutoTest", "RouteMaxMinutes", routeMaxMinutes);
    if (routeMode) highwayOnly = false;
    std::string tag = ini.ReadString("AutoTest", "Tag", vanilla ? "vanilla" : "mod");
    shotDir = dir + "autotest_" + tag + "\\";
    CreateDirectoryA(shotDir.c_str(), nullptr);
    out.open(shotDir + "autotest.log", std::fstream::out | std::fstream::trunc);
    if (scanMode) scanOut.open(shotDir + "scan.txt", std::ios::trunc);
    collisionsCsv.open(shotDir + "collisions.csv", std::ios::trunc);
    collisionsCsv << "id;minute;x;y;type;cause;closing_kmh;severity;road;junction_m;"
                     "A_who;A_speed;A_target;A_limiter;A_lane_state;A_indicator;B_who;B_speed;B_target;B_limiter;B_lane_state;B_indicator;diagram\n";
    out << "AutoTest " << tag << " mode=" << (vanilla ? "vanilla" : "mod") << " secondsPerLocation=" << secondsPerLocation
        << " started " << WallClock() << "\n";
    out.flush();
    cfgTestMode = true; // debug overlay on the screenshots
}

static void TakeShot(const std::string& name) {
    if (!pendingShot.empty()) return;
    pendingShot = shotDir + name + ".jpg";
}

void Draw() {
    if (!enabled) return;
    if (state != WAIT_GAME && state != DONE) {
        CFont::SetBackground(false, false);
        CFont::SetJustify(false);
        CFont::SetProportional(true);
        CFont::SetOrientation(ALIGN_LEFT);
        CFont::SetFontStyle(FONT_SUBTITLES);
        CFont::SetColor({ 255, 255, 0, 255 });
        CFont::SetEdge(1);
        CFont::SetScale(0.4f, 0.9f);
        CFont::PrintString(10.0f, 40.0f, Format("AUTOTEST %s  %s  t=%.1f min  driven %.2f km  contacts %d  oscillations %d  jams %d  bypasses %d  %s",
            vanilla ? "VANILLA" : "MOD", locIndex >= 0 && locIndex < NUM_LOCATIONS ? locations[locIndex].name : "-", Minutes(),
            distanceDriven / 1000.0f, collisionCount, oscillationCount, jamCount, bypassCount, WallClock().c_str()).c_str());
    }
    if (!pendingShot.empty()) {
        JPegCompressScreenToFile(pendingShot.c_str());
        shotsTaken++;
        lastShotTime = Now();
        pendingShot.clear();
    }
}

// Following distance actually kept: mod-driven vehicles above 30 km/h behind a vehicle going about their speed
// (within 3 m/s). Time gaps by bin, freeways vs. elsewhere, and what the pedals were doing then
static const float followBins[] = { 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 1e9f };
static int followCount[2][6] = {}, followBraking[2] = {}, followTotal[2] = {};
static double followGapSum[2] = {}, followGasSum[2] = {};
static void RecordFollowing(CVehicle* veh, const ExtendedData& x) {
    float v = veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
    if (Now() - x.lastProcessed > 500 || x.obstacle != OBSTACLE_VEHICLE || v < 30.0f / MPS_TO_KMH || std::abs(v - x.leaderSpeed) > 3.0f) return;
    int road = (x.roadType == ROAD_HIGHWAY || x.roadType == ROAD_FAST) ? 0 : 1;
    float tg = x.gap / v;
    int b = 0;
    while (tg >= followBins[b]) b++;
    followCount[road][b]++;
    followTotal[road]++;
    followGapSum[road] += tg;
    followGasSum[road] += veh->m_fGasPedal;
    if (veh->m_fBreakPedal > 0.1f) followBraking[road]++;
}

static void RecordSample(CVehicle* veh, History& h) {
    ExtendedData& x = vehExData.Get(veh);
    RecordFollowing(veh, x);
    Sample& s = h.s[h.head];
    s.t = Now();
    s.pos = veh->GetPosition();
    s.hx = veh->GetForward().x;
    s.hy = veh->GetForward().y;
    s.speed = veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH;
    s.fwd = CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS * MPS_TO_KMH;
    s.valid = x.valid;
    s.target = x.targetSpeed * MPS_TO_KMH;
    s.gap = x.gap;
    s.leader = x.leaderSpeed * MPS_TO_KMH;
    s.stuck = x.stuckTime;
    s.pedalFresh = Now() - x.dbgPedalTime < 200;
    s.gameGas = x.dbgGameGas; s.gameBrake = x.dbgGameBrake; s.gas = x.dbgGas; s.brake = x.dbgBrake;
    s.pedalGas = veh->m_fGasPedal; s.pedalBrake = veh->m_fBreakPedal;
    s.limiter = x.limiter; s.obstacle = x.obstacle; s.light = x.lightColor; s.lightDist = x.lightDist;
    s.temp = veh->m_autoPilot.m_nTempAction;
    s.style = veh->m_autoPilot.m_nCarDrivingStyle;
    s.mission = veh->m_autoPilot.m_nCarMission;
    s.status = veh->m_nStatus;
    s.laneCur = veh->m_autoPilot.m_nCurrentLane;
    s.laneNext = veh->m_autoPilot.m_nNextLane;
    s.lanes = static_cast<char>(x.lanesHere);
    s.laneState = x.laneState;
    s.indicator = Now() < x.indicatorUntil ? static_cast<char>(x.indicator) : 0;
    s.collAge = std::min((int)(Now() - veh->m_nLastCollisionTime), 99999);
    s.collType = veh->m_pDamageEntity ? veh->m_pDamageEntity->m_nType : 0;
    s.stuckAge = std::min((int)(Now() - veh->m_autoPilot.m_nTimeToStartMission), 99999);
    h.head = (h.head + 1) % HISTORY;
    h.count = std::min(h.count + 1, HISTORY);
}

static const Sample* Latest(const History& h) {
    return h.count ? &h.s[(h.head + HISTORY - 1) % HISTORY] : nullptr;
}

// Which side of 'veh' the point is on: front, rear, left, right
static const char* SideOf(CVehicle* veh, const CVector& point) {
    CVector mn, mx;
    GetEntityDimensions(veh, mn, mx);
    CVector d = point - veh->GetPosition();
    float lx = CVector::Dot(d, veh->GetRight()), ly = CVector::Dot(d, veh->GetForward());
    float fx = lx / std::max(mx.x, 0.5f), fy = ly / std::max(mx.y, 0.5f);
    if (std::abs(fy) * 1.0f >= std::abs(fx) * 0.8f) return ly > 0.0f ? "front" : "rear";
    return lx > 0.0f ? "right" : "left";
}

static std::string Describe(CVehicle* veh) {
    ExtendedData& x = vehExData.Get(veh);
    const char* who = veh == testCar ? "TESTCAR" : (!veh->m_pDriver ? "parked/empty" :
        (veh->m_pDriver->IsPlayer() ? "PLAYER" : (veh->m_nCreatedBy == MISSION_VEHICLE ? "mission" : "traffic")));
    return Format("veh#%d model %d %s%s", CPools::GetVehicleRef(veh), veh->m_nModelIndex, who, x.bRacer ? " racer" : "");
}

static void DumpHistory(CVehicle* veh, const History& h) {
    out << "    t(ms)  speed  fwd  target limiter        obst  gap  lead | gameGas gameBrk -> gas  brake | pedG pedB | temp style miss stat light stuck valid  collAge type stuckAge | lane cur>next/of st ind\n";
    for (int i = 0; i < h.count; i++) {
        const Sample& s = h.s[(h.head + HISTORY - h.count + i) % HISTORY];
        out << Format("   %6d %6.1f %5.1f %6.1f %-14s %-8s %4.1f %5.1f | %5.2f   %5.2f  -> %5.2f %5.2f%s| %4.2f %4.2f | %3d %4d %4d %4d  %-6s %4.1f %d  %6d %d %6d | %d>%d/%d %d %+d\n",
            (int)s.t - (int)Now(), s.speed, s.fwd, s.target, limiterNames[s.limiter], obstacleNames[s.obstacle], s.gap, s.leader,
            s.gameGas, s.gameBrake, s.gas, s.brake, s.pedalFresh ? " " : "*", s.pedalGas, s.pedalBrake, s.temp, s.style, s.mission,
            s.status, lightNames[s.light], s.stuck, s.valid ? 1 : 0, s.collAge, s.collType, s.stuckAge,
            s.laneCur, s.laneNext, s.lanes, s.laneState, s.indicator);
    }
}

// Point of the vehicle's 2D bounding box closest to p
static CVector ClosestPointOnBox(CVehicle* veh, const CVector& p) {
    CVector mn, mx;
    GetEntityDimensions(veh, mn, mx);
    CVector d = p - veh->GetPosition();
    float lx = std::clamp(CVector::Dot(d, veh->GetRight()), mn.x, mx.x);
    float ly = std::clamp(CVector::Dot(d, veh->GetForward()), mn.y, mx.y);
    return veh->GetPosition() + veh->GetRight() * lx + veh->GetForward() * ly;
}

// Do the 2D bounding boxes of both vehicles overlap (grown by 'margin')? Separating axis test
static bool BoxesTouch(CVehicle* a, CVehicle* b, float margin) {
    if (std::abs(a->GetPosition().z - b->GetPosition().z) > 3.0f) return false;
    CVector amn, amx, bmn, bmx;
    GetEntityDimensions(a, amn, amx);
    GetEntityDimensions(b, bmn, bmx);
    CVector2D ac = (a->GetPosition() + a->GetRight() * ((amn.x + amx.x) * 0.5f) + a->GetForward() * ((amn.y + amx.y) * 0.5f)).To2D();
    CVector2D bc = (b->GetPosition() + b->GetRight() * ((bmn.x + bmx.x) * 0.5f) + b->GetForward() * ((bmn.y + bmx.y) * 0.5f)).To2D();
    CVector2D axes[4] = { a->GetRight().To2D(), a->GetForward().To2D(), b->GetRight().To2D(), b->GetForward().To2D() };
    for (CVector2D& ax : axes) {
        float len = ax.Magnitude();
        if (len > 0.01f) ax *= 1.0f / len;
    }
    float ahx = (amx.x - amn.x) * 0.5f + margin, ahy = (amx.y - amn.y) * 0.5f + margin;
    float bhx = (bmx.x - bmn.x) * 0.5f, bhy = (bmx.y - bmn.y) * 0.5f;
    CVector2D d = bc - ac;
    for (const CVector2D& ax : axes) {
        float ra = ahx * std::abs(CVector2D::Dot(axes[0], ax)) + ahy * std::abs(CVector2D::Dot(axes[1], ax));
        float rb = bhx * std::abs(CVector2D::Dot(axes[2], ax)) + bhy * std::abs(CVector2D::Dot(axes[3], ax));
        if (std::abs(CVector2D::Dot(d, ax)) > ra + rb) return false;
    }
    return true;
}

// The last 5 s around a collision, seen from above: road lanes, and the tracks of both vehicles and of the ones near
// them. tools\collision_diagram.py draws it (collision_NNN.png). The camera follows the test car, so a screenshot
// almost never shows the collision; this works anywhere on the map
static void WriteCollisionTrack(const std::string& name, CVehicle* a, CVehicle* b, const CVector& impact,
    const std::string& header)
{
    std::ofstream f(shotDir + name + ".trk");
    if (!f) return;
    f << header;
    f << Format("IMPACT %.2f %.2f\n", impact.x, impact.y);
    const float radius = 70.0f;
    // Lanes: centre line of every lane of every link with a node near the impact
    for (int area = 0; area < NUM_PATH_MAP_AREAS; area++) {
        if (!ThePaths.m_pPathNodes[area]) continue;
        for (unsigned int i = 0; i < ThePaths.m_dwNumVehicleNodes[area]; i++) {
            CNodeAddress addr(static_cast<short>(area), static_cast<short>(i));
            CPathNode* node = GetNode(addr);
            if (!node) continue;
            CVector p = GetNodePos(node);
            if ((p - impact).Magnitude2D() > radius || std::abs(p.z - impact.z) > 15.0f) continue;
            for (int k = 0; k < node->NumberAdjNodes; k++) {
                CNodeAddress other = GetAdjacentNode(node, addr, k);
                CPathNode* on = GetNode(other);
                if (!on) continue;
                CCarPathLink* link = GetLinkBetween(addr, node, other);
                if (!link) continue;
                int lanes = LanesToward(link, other);
                CVector q = GetNodePos(on);
                CVector2D d = (q - p).To2D();
                float len = d.Magnitude();
                if (lanes == 0 || len < 0.5f) continue;
                d *= 1.0f / len;
                const CVector2D r(d.y, -d.x);
                const CVector2D lp(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
                float ow = link->OneWayLaneOffset();
                f << Format("ROAD %.2f %.2f %.2f %.2f %d %d\n", p.x, p.y, q.x, q.y, node->Speed, node->NumberAdjNodes > 2 || on->NumberAdjNodes > 2 ? 1 : 0);
                for (int l = 0; l < lanes; l++) {
                    // Lane centre: through the link position, (ow + l) lanes to the right, from node p to node q
                    CVector2D c = lp + r * ((ow + l) * LANE_WIDTH);
                    CVector2D s0 = c + d * CVector2D::Dot(p.To2D() - c, d), s1 = c + d * CVector2D::Dot(q.To2D() - c, d);
                    f << Format("LANE %.2f %.2f %.2f %.2f %d\n", s0.x, s0.y, s1.x, s1.y, l);
                }
            }
        }
    }
    // Vehicles: A, B and the others that were near
    for (auto& [v, h] : histories) {
        if (!CPools::ms_pVehiclePool->IsObjectValid(v) || !h.count) continue;
        const char* role = v == a ? "A" : (v == b ? "B" : "O");
        if (v != a && v != b && (v->GetPosition() - impact).Magnitude() > 45.0f) continue;
        CVector mn, mx;
        GetEntityDimensions(v, mn, mx);
        const ExtendedData& x = vehExData.Get(v);
        std::string label = !v->m_pDriver ? "empty" : (Now() - x.lastProcessed > 500 ? "gameAI" : profileNames[x.profile]);
        for (char& ch : label) if (ch == ' ') ch = '_';
        f << Format("VEH %d %s %.2f %.2f %.2f %.2f %d %s%s\n", CPools::GetVehicleRef(v), role, mn.x, mx.x, mn.y, mx.y,
            v->m_nModelIndex, label.c_str(), v == testCar ? "_TESTCAR" : "");
        for (int i = 0; i < h.count; i++) {
            const Sample& s = h.s[(h.head + HISTORY - h.count + i) % HISTORY];
            int age = (int)s.t - (int)Now();
            if (age < -5000) continue;
            f << Format("S %d %d %.2f %.2f %.3f %.3f %.1f %.1f %d %d %d %d %d %s %.1f\n", CPools::GetVehicleRef(v), age, s.pos.x, s.pos.y,
                s.hx, s.hy, s.fwd, s.target, s.limiter, s.laneState, s.indicator, s.laneCur, s.laneNext,
                obstacleNames[s.obstacle], s.gap);
        }
    }
}

static void OnCollision(CVehicle* a, CVehicle* b, float intensity) {
    History& ha = histories[a];
    History& hb = histories[b];
    RecordSample(a, ha);
    RecordSample(b, hb);
    const Sample* sa = Latest(ha);
    const Sample* sb = Latest(hb);
    CVector impact = (ClosestPointOnBox(a, b->GetPosition()) + ClosestPointOnBox(b, a->GetPosition())) * 0.5f;
    const char* sideA = SideOf(a, impact);
    const char* sideB = SideOf(b, impact);

    // Classify
    std::string type;
    bool reversing = sa->fwd < -1.0f || sb->fwd < -1.0f || sa->temp == TEMPACT_REVERSE_LEFT || sa->temp == TEMPACT_REVERSE_RIGHT ||
        sb->temp == TEMPACT_REVERSE_LEFT || sb->temp == TEMPACT_REVERSE_RIGHT;
    if ((a == testCar || b == testCar) && Now() - stateTime < 5000 && state == DRIVE) type = "test car spawn";
    else if (!b->m_pDriver || !a->m_pDriver) type = "hit parked/empty";
    else if (Now() - ha.firstSeen < 3000 || Now() - hb.firstSeen < 3000) type = "just spawned (< 3 s)"; // the game's car generation
    else if (reversing) type = "reversing";
    else if ((!strcmp(sideA, "front") && !strcmp(sideB, "rear")) || (!strcmp(sideA, "rear") && !strcmp(sideB, "front"))) type = "rear-end";
    else if (!strcmp(sideA, "front") && !strcmp(sideB, "front")) type = "head-on";
    else type = "side";

    collisionCount++;
    typeCounts[type]++;
    for (TripCar& c : tripCars) if (c.veh && (c.veh == a || c.veh == b)) c.collisions++;
    if (sa->valid || sb->valid) involvingModCount++;
    if (reversing) involvingReverseCount++;

    // Probable cause, from what both drivers were doing (now and in the last 2 s)
    auto recently = [&](const History& h, auto pred) {
        for (int i = 0; i < h.count; i++) {
            const Sample& smp = h.s[(h.head + HISTORY - 1 - i) % HISTORY];
            if (Now() - smp.t > 2000) break;
            if (pred(smp)) return true;
        }
        return false;
    };
    auto junctionDist = [](CVehicle* v) {
        CPathNode* next = GetNode(v->m_autoPilot.m_startingRouteNode);
        CPathNode* curr = GetNode(v->m_autoPilot.m_currentAddress);
        float d = 1000.0f;
        if (next && next->NumberAdjNodes > 2) d = std::min(d, (GetNodePos(next) - v->GetPosition()).Magnitude2D());
        if (curr && curr->NumberAdjNodes > 2) d = std::min(d, (GetNodePos(curr) - v->GetPosition()).Magnitude2D());
        return d;
    };
    auto who = [&](CVehicle* v, const Sample* smp, const History& h) {
        const ExtendedData& x = vehExData.Get(v);
        std::string st = Now() - x.lastProcessed > 500 ? "game-AI" : profileNames[x.profile]; // stopped mod cars aren't 'valid' but are ours
        if (recently(h, [](const Sample& q) { return q.laneState == LANE_CHANGING; })) st += " changing-lane";
        else if (recently(h, [](const Sample& q) { return q.laneState == LANE_SIGNAL_OVERTAKE || q.laneState == LANE_SIGNAL_MOVE; })) st += " signalling";
        if (recently(h, [](const Sample& q) { return q.laneState == LANE_OVERTAKING; })) st += " overtaking";
        if (smp->indicator) st += smp->indicator > 0 ? " ind>" : " ind<";
        return st + Format(" %.0fkm/h target %.0f (%s) %s gap %.0f, junction %.0fm, yaw %.2f", smp->fwd, smp->target, limiterNames[smp->limiter],
            obstacleNames[smp->obstacle], smp->gap, junctionDist(v), std::abs(v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS));
    };
    std::string cause;
    if (type == "rear-end" || type == "side" || type == "head-on") {
        bool aFront = !strcmp(sideA, "front"), bFront = !strcmp(sideB, "front");
        CVehicle* striker = aFront && !bFront ? a : (bFront && !aFront ? b : (sa->fwd >= sb->fwd ? a : b));
        const Sample* ss = striker == a ? sa : sb;
        const History& hs = striker == a ? ha : hb;
        auto changing = [](const Sample& q) { return q.laneState == LANE_CHANGING; }; // moving over (only signalling doesn't count)
        bool laneChange = recently(ha, changing) || recently(hb, changing);
        bool atJunction = junctionDist(a) < 20.0f || junctionDist(b) < 20.0f;
        bool turning = std::abs(a->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS) > 0.25f || std::abs(b->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS) > 0.25f;
        bool sawIt = recently(hs, [](const Sample& q) { return q.obstacle == OBSTACLE_VEHICLE; });
        bool modA = Now() - vehExData.Get(a).lastProcessed < 500, modB = Now() - vehExData.Get(b).lastProcessed < 500;
        if (!modA && !modB) cause = "game AI only (neither driven by the mod)";
        // Through a red light: the one that hits was moving with its light red (queued cars waiting at a red one aren't)
        else if (ss->light == LIGHT_RED && ss->fwd > 10.0f && ss->lightDist < 15.0f && atJunction) cause = "red light run";
        else if (laneChange && type != "rear-end") cause = "lane change";
        else if (type == "rear-end" && turning) cause = sawIt ? "rear-end while turning: saw it" : "rear-end while turning: didn't see it";
        else if (atJunction && std::abs(sa->fwd) < 10.0f && std::abs(sb->fwd) < 10.0f) cause = "junction: low-speed squeeze (queue/turn)";
        else if (atJunction && turning && (std::abs(sa->fwd) < 2.0f || std::abs(sb->fwd) < 2.0f)) cause = "junction: turning into a stopped car";
        else if (atJunction && turning) cause = "junction: turning";
        else if (atJunction) cause = "junction: crossing";
        else if (type == "rear-end") cause = sawIt ? "rear-end: saw it, too fast/close" : "rear-end: didn't see it";
        else if (turning) cause = "curve: drifted into the other car";
        else cause = "side contact on a straight";
        if (Now() - vehExData.Get(striker).lastProcessed > 500) cause += " [striker: game AI]";
    }
    else cause = type; // spawn, parked, reversing: not a driving cause
    causeCounts[cause]++;
    lastCause = Format("#%d %s", collisionCount, cause.c_str());
    {
        auto whoShort = [&](CVehicle* v) {
            const ExtendedData& x = vehExData.Get(v);
            if (!v->m_pDriver) return std::string("empty");
            return std::string(Now() - x.lastProcessed > 500 ? "game-AI" : profileNames[x.profile]);
        };
        CVehicle* hitter = sa->fwd >= sb->fwd ? a : b;
        const ExtendedData& hx = vehExData.Get(hitter);
        CollisionRec rec{ impact.x, impact.y, intensity, cause, roadNames[hx.roadType], whoShort(a), whoShort(b) };
        collisionRecs.push_back(rec);
        if (collisionsCsv) {
            auto side = [&](CVehicle* v, const Sample* sm) {
                return Format("%s;%.0f;%.0f;%s;%d;%d", whoShort(v).c_str(), sm->fwd, sm->target, limiterNames[sm->limiter], sm->laneState, sm->indicator);
            };
            collisionsCsv << Format("%d;%.2f;%.0f;%.0f;%s;%s;%.1f;%s;%s;%.0f;", collisionCount, Minutes(), impact.x, impact.y, type.c_str(),
                cause.c_str(), intensity, Severity(intensity), roadNames[hx.roadType], std::min(junctionDist(a), junctionDist(b)))
                << side(a, sa) << ";" << side(b, sb) << Format(";collision_%03d\n", collisionCount);
            collisionsCsv.flush();
        }
    }

    float distCam = TheCamera.GetPosition().Distance(impact);
    // Diagram from above for every collision; a screenshot only when the test car is in it (it's what the camera shows)
    std::string shot = Format("collision_%03d", collisionCount);
    const std::string whoA = who(a, sa, ha), whoB = who(b, sb, hb);
    WriteCollisionTrack(shot, a, b, impact, Format("TITLE #%d [%s] t=%.2f min, closing %.0f km/h\nCAUSE %s\nWHO_A %s\nWHO_B %s\n",
        collisionCount, type.c_str(), Minutes(), intensity, cause.c_str(), whoA.c_str(), whoB.c_str()));
    if ((a == testCar || b == testCar) && distCam < 70.0f && Now() - lastShotTime > 1500) TakeShot(shot);

    out << "\n=== COLLISION #" << collisionCount << " [" << type << "] t=" << Format("%.2f", Minutes()) << " min (" << WallClock()
        << ") at " << locations[std::max(locIndex, 0)].name << Format(" (%.0f, %.0f, %.0f) closing %.1f km/h camDist %.0fm",
            impact.x, impact.y, impact.z, intensity, distCam) << " diagram " << shot << "\n";
    out << "  CAUSE: " << cause << "\n";
    out << "    A: " << whoA << "\n";
    out << "    B: " << whoB << "\n";
    out << "  A " << Describe(a) << " hit on its " << sideA << Format(", fwd %.1f km/h", sa->fwd) << "\n";
    DumpHistory(a, ha);
    out << "  B " << Describe(b) << " hit on its " << sideB << Format(", fwd %.1f km/h", sb->fwd) << "\n";
    DumpHistory(b, hb);
    out.flush();
}

static int PlayerHandle() { return CPools::GetPedRef(FindPlayerPed()); }

static void RemoveTestCar() {
    if (carHandle >= 0 && testCar && CPools::ms_pVehiclePool->IsObjectValid(testCar)) {
        const Location& l = locations[std::max(locIndex, 0)];
        Command<COMMAND_WARP_CHAR_FROM_CAR_TO_COORD>(PlayerHandle(), l.x, l.y, l.z + 1.0f);
        if (driverHandle >= 0) Command<COMMAND_DELETE_CHAR>(driverHandle);
        Command<COMMAND_DELETE_CAR>(carHandle);
    }
    carHandle = driverHandle = -1;
    testCar = nullptr;
}

// Car path node closest to (x, y) with a legal direction to drive away from it
static bool FindStartNode(float x, float y, float z, bool highway, CVector& outPos, float& outHeading) {
    float best = 1e9f;
    for (int a = 0; a < NUM_PATH_MAP_AREAS; a++) {
        CPathNode* nodes = ThePaths.m_pPathNodes[a];
        if (!nodes) continue;
        unsigned int num = ThePaths.m_dwNumVehicleNodes[a];
        for (unsigned int i = 0; i < num; i++) {
            CPathNode* node = &nodes[i];
            if (node->NumberAdjNodes != 2 || node->WaterNode || node->SwitchedOff || node->InteriorNode) continue;
            if (highway && node->Speed < 2) continue;
            CVector p = GetNodePos(node);
            float d = (p - CVector(x, y, z)).Magnitude2D() + std::abs(p.z - z) * 2.0f;
            if (d >= best) continue;
            CNodeAddress addr;
            addr.Region = static_cast<UInt16>(a);
            addr.Index = static_cast<UInt16>(i);
            for (int k = 0; k < 2; k++) {
                CNodeAddress nextAddr = GetAdjacentNode(node, addr, k);
                CPathNode* next = GetNode(nextAddr);
                CCarPathLink* link = next ? GetLinkBetween(addr, node, nextAddr) : nullptr;
                if (!link || LanesToward(link, nextAddr) <= 0) continue;
                // Freeway: a real carriageway (2+ lanes our way, next node also a freeway one), not a stray class 2 node
                if (highway && (next->Speed < 2 || LanesToward(link, nextAddr) < 2)) continue;
                CVector dir = GetNodePos(next) - p;
                best = d;
                outPos = p;
                outHeading = CGeneral::GetATanOfXY(dir.x, dir.y) - 1.5707964f;
                break;
            }
        }
    }
    return best < (highway ? 900.0f : 250.0f);
}

// One scan line across the road where the test car is, perpendicular to the car (it follows the road; the node-to-node
// line cuts ramps and curves diagonally). "S" header: where, which link, the lanes the game has there, where the car is
// across that link, its lanes and lane state, and where the game's lane 0 is (world); "R" the 121 ground readings
// (-30..+30 m every 0.5 m to the right), from 2 m above the car (higher, the rays hit the overpasses)
static void ScanSample(CVehicle* veh) {
    const CAutoPilot& ap = veh->m_autoPilot;
    CPathNode* curr = GetNode(ap.m_currentAddress);
    CPathNode* next = GetNode(ap.m_startingRouteNode);
    if (!curr || !next) return;
    CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
    if (!link) return;
    CVector2D a = GetNodePos(curr).To2D(), dl = GetNodePos(next).To2D() - a;
    float len = dl.Magnitude();
    if (len < 0.5f) return;
    dl *= 1.0f / len;
    const CVector2D rl(dl.y, -dl.x);
    const CVector2D lp(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
    const CVector pos = veh->GetPosition();
    CVector2D d = veh->GetForward().To2D();
    float dlen = d.Magnitude();
    if (dlen < 0.1f) return;
    d *= 1.0f / dlen;
    const CVector2D r(d.y, -d.x);
    const float ow = link->OneWayLaneOffset();
    const float carLat = CVector2D::Dot(pos.To2D() - lp, rl);
    const CVector2D lane0 = lp + dl * CVector2D::Dot(pos.To2D() - lp, dl) + rl * (ow * LANE_WIDTH);
    const ExtendedData& x = vehExData.Get(veh);
    scanOut << Format("S %.2f %.2f %.2f %.4f %.4f %d:%d %d:%d %d %d %.2f %.2f %.3f %d %d %d %d %.2f %d %d %d %.2f %.2f %.4f %.4f\n",
        pos.x, pos.y, pos.z, d.x, d.y,
        ap.m_currentAddress.Region, ap.m_currentAddress.Index, ap.m_startingRouteNode.Region, ap.m_startingRouteNode.Index,
        LanesToward(link, ap.m_startingRouteNode), LanesToward(link, ap.m_currentAddress), lp.x, lp.y, ow,
        link->m_nPathNodeWidth, next->Speed, curr->NumberAdjNodes, next->NumberAdjNodes, carLat,
        ap.m_nCurrentLane, ap.m_nNextLane, x.laneState, lane0.x, lane0.y, dl.x, dl.y);
    scanOut << "R";
    CWorld::pIgnoreEntity = veh;
    for (int i = 0; i <= 120; i++) {
        float o = -30.0f + i * 0.5f;
        CVector2D p = pos.To2D() + r * o;
        CColPoint colPoint;
        CEntity* entity = nullptr;
        if (CWorld::ProcessLineOfSight({ p.x, p.y, pos.z + 2.0f }, { p.x, p.y, pos.z - 8.0f }, colPoint, entity,
            true, false, false, false, false, false, false, false))
            scanOut << Format(" %.2f", colPoint.m_vecPoint.z - pos.z);
        else scanOut << " x";
    }
    CWorld::pIgnoreEntity = nullptr;
    scanOut << "\n";
    if (++scanSamples % 50 == 0) scanOut.flush();
}

void Update() {
    if (!enabled || state == DONE) return;
    CPlayerPed* player = FindPlayerPed();
    if (!player) return;
    unsigned int now = Now();
    // The scan runs with the normal traffic: its rays only see the ground (buildings), never vehicles
    if (noPeds) {
        // Every frame (the game resets it); the ones already there are removed every 2 s
        Command<COMMAND_SET_PED_DENSITY_MULTIPLIER>(0.0f);
        static unsigned int lastClear = 0;
        if (now - lastClear > 2000) {
            lastClear = now;
            // Only pedestrians on foot (CLEAR_AREA_OF_CHARS may take the drivers out of the traffic too)
            for (int i = 0; i < CPools::ms_pPedPool->m_nSize; i++) {
                CPed* ped = CPools::ms_pPedPool->GetAt(i);
                if (!ped || ped == player || ped->bInVehicle || ped->m_pVehicle || ped->m_nCreatedBy != 1) continue; // 1: random
                if ((ped->GetPosition() - player->GetPosition()).Magnitude() > 250.0f) continue;
                Command<COMMAND_DELETE_CHAR>(CPools::GetPedRef(ped));
            }
        }
    }

    switch (state) {
    case WAIT_GAME:
        if (now < 8000) return; // let the save load settle
        startTime = now;
        Command<COMMAND_SET_MAX_WANTED_LEVEL>(0);
        Command<COMMAND_SET_EVERYONE_IGNORE_PLAYER>(0, 1);
        Command<COMMAND_SET_CHAR_PROOFS>(PlayerHandle(), 1, 1, 1, 1, 1);
        Command<COMMAND_SET_TIME_OF_DAY>(13, 0);
        Command<COMMAND_FORCE_WEATHER_NOW>(0);
        out << "Game ready, starting at " << WallClock() << "\n";
        state = TELEPORT;
        return;

    case TELEPORT: {
        RemoveTestCar();
        locIndex++;
        while (highwayOnly && locIndex < NUM_LOCATIONS && !locations[locIndex].highway) locIndex++;
        while (routeMode && locIndex < NUM_LOCATIONS && (!locations[locIndex].route ||
            (!routeFilter.empty() && std::string(locations[locIndex].name).find(routeFilter) == std::string::npos))) locIndex++;
        if (!routeMode) while (locIndex < NUM_LOCATIONS && locations[locIndex].route) locIndex++;
        if (locIndex >= (highwayOnly || routeMode ? NUM_LOCATIONS : maxLocations)) { state = DONE; break; }
        const Location& l = locations[locIndex];
        if (player->m_pVehicle) Command<COMMAND_WARP_CHAR_FROM_CAR_TO_COORD>(PlayerHandle(), l.x, l.y, l.z + 1.0f);
        else Command<COMMAND_SET_CHAR_COORDINATES>(PlayerHandle(), l.x, l.y, l.z + 1.0f);
        Command<COMMAND_LOAD_SCENE>(l.x, l.y, l.z);
        Command<COMMAND_REQUEST_MODEL>(carModel);
        out << "\n--- Location " << locIndex + 1 << "/" << maxLocations << ": " << l.name << " at " << Format("%.2f", Minutes()) << " min\n";
        out.flush();
        state = WAIT_STREAM; stateTime = now; locationStart = now;
        return;
    }

    case WAIT_STREAM:
        Command<COMMAND_SET_TIME_OF_DAY>(13, 0);
        if (now - stateTime < 3000) return;
        Command<COMMAND_LOAD_ALL_MODELS_NOW>();
        state = SPAWN;
        return;

    case SPAWN: {
        const Location& l = locations[locIndex];
        CVector pos; float heading;
        if (!Command<COMMAND_HAS_MODEL_LOADED>(carModel) || !FindStartNode(l.x, l.y, l.z, l.highway, pos, heading)) {
            if (now - stateTime > 15000) { out << "  could not spawn here, skipping\n"; state = TELEPORT; }
            return;
        }
        Command<COMMAND_CLEAR_AREA>(pos.x, pos.y, pos.z, 6.0f, 0);
        Command<COMMAND_CREATE_CAR>(carModel, pos.x, pos.y, pos.z + 1.0f, &carHandle);
        testCar = CPools::GetVehicle(carHandle);
        if (!testCar) { state = TELEPORT; return; }
        testCar->SetHeading(heading);
        Command<COMMAND_CREATE_RANDOM_CHAR_AS_DRIVER>(carHandle, &driverHandle);
        Command<COMMAND_WARP_CHAR_INTO_CAR_AS_PASSENGER>(PlayerHandle(), carHandle, 0);
        Command<COMMAND_SET_CAR_PROOFS>(carHandle, 1, 1, 1, 1, 1);
        Command<COMMAND_CAR_WANDER_RANDOMLY>(carHandle);
        Command<COMMAND_SET_CAR_DRIVING_STYLE>(carHandle, 0);
        Command<COMMAND_SET_CAR_CRUISE_SPEED>(carHandle, 20.0f);
        testCar->m_nCreatedBy = RANDOM_VEHICLE; // an ordinary traffic car for the mod
        if (l.route) {
            // Car 0 (normal driver) is the one just created; car 1 (aggressive driver) starts 14 m behind it
            tripCars[0].handle = carHandle;
            tripCars[0].driver = driverHandle;
            tripCars[0].veh = testCar;
            CVector back = pos - CVector(-std::sin(heading), std::cos(heading), 0.0f) * 14.0f;
            bool ground = false;
            float gz = FindGroundZ({ back.x, back.y, back.z + 3.0f }, &ground);
            if (ground) back.z = gz;
            if (tripCarCount > 1) {
                Command<COMMAND_CREATE_CAR>(carModel, back.x, back.y, back.z + 1.0f, &tripCars[1].handle);
                tripCars[1].veh = CPools::GetVehicle(tripCars[1].handle);
            }
            else { tripCars[1].done = true; tripCars[1].result = "(not used)"; }
            if (tripCars[1].veh) {
                tripCars[1].veh->SetHeading(heading);
                Command<COMMAND_CREATE_RANDOM_CHAR_AS_DRIVER>(tripCars[1].handle, &tripCars[1].driver);
                Command<COMMAND_SET_CAR_PROOFS>(tripCars[1].handle, 1, 1, 1, 1, 1);
                Command<COMMAND_CAR_WANDER_RANDOMLY>(tripCars[1].handle);
                Command<COMMAND_SET_CAR_DRIVING_STYLE>(tripCars[1].handle, 0);
                Command<COMMAND_SET_CAR_CRUISE_SPEED>(tripCars[1].handle, 20.0f);
            }
            tripCars[0].colour = 1; // NORMAL white
            tripCars[1].colour = 3; // AGGRESSIVE red
            for (TripCar& c : tripCars) {
                if (c.done) continue;
                if (!c.veh) { c.done = true; c.result = "FAIL: could not be created"; continue; }
                // Easy to tell apart, and always on the radar
                Command<COMMAND_CHANGE_CAR_COLOUR>(c.handle, c.colour, c.colour);
                Command<COMMAND_ADD_BLIP_FOR_CAR>(c.handle, &c.blip);
                // A mission vehicle, so the game never removes it (it deleted the one the player wasn't in, off screen, and
                // its pool slot went to another car); the mod drives it anyway
                c.veh->m_nCreatedBy = MISSION_VEHICLE;
                ExtendedData& x = vehExData.Get(c.veh);
                x.forceDrive = true;
                x.bInitialized = true;
                x.profile = c.profile;
                x.bRacer = profiles[c.profile].racingLogic;
                x.aggression = profiles[c.profile].aggression;
                x.speedVariation = 1.0f;
                c.wp = 0;
                c.snapped = false;
                SetWaypointDestination(c.veh, l, 0);
                c.lastPos = c.veh->GetPosition();
                c.lastProgress = now;
            }
            followed = 0;
            out << Format("  route trip to (%.0f, %.0f), %.0f m in a straight line: %s and %s drivers, Left/Right arrow to switch\n",
                l.destX, l.destY, TripLeft(l, 0, pos.To2D()), tripCars[0].label, tripCars[1].label);
            for (int i = 0; i < l.numWaypoints; i++)
                out << Format("    waypoint %d: %s (%.0f, %.0f)\n", i + 1, l.waypoints[i].name, l.waypoints[i].x, l.waypoints[i].y);
        }
        Command<COMMAND_SET_CAMERA_BEHIND_PLAYER>();
        lastCarPos = testCar->GetPosition();
        locationDistance = 0.0f;
        carStillTime = 0.0f;
        out << Format("  test car spawned at (%.0f, %.0f, %.0f)\n", pos.x, pos.y, pos.z);
        out.flush();
        state = DRIVE; stateTime = now;
        return;
    }

    case DRIVE: {
        if (stopAt && now >= stopAt) { state = DONE; break; }
        if (locations[locIndex].route) {
            // The trip: no resets. Each car arrives, gets lost or stuck on its own; the test ends when both are done
            const Location& l = locations[locIndex];
            const float dt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
            bool allDone = true;
            for (TripCar& c : tripCars) {
                if (c.done) continue;
                bool lost = !c.veh || CPools::GetVehicle(c.handle) != c.veh || c.veh->m_nStatus == STATUS_WRECKED;
                float left = 0.0f;
                if (lost) c.result = "FAIL: lost/wrecked";
                else {
                    c.veh->m_fHealth = 1000.0f;
                    float step = (c.veh->GetPosition() - c.lastPos).Magnitude2D();
                    if (step < 50.0f) c.driven += step;
                    c.lastPos = c.veh->GetPosition();
                    c.still = c.veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS < 0.5f ? c.still + dt : 0.0f;
                    const CVector2D carPos = c.veh->GetPosition().To2D();
                    const CVector2D wpPos = WaypointPos(l, c.wp);
                    const float toWp = (wpPos - carPos).Magnitude();
                    const int kind = WaypointKind(l, c.wp);
                    // The parking place at the end: its node once the roads there are loaded (every 2 s while within 700 m)
                    if (kind == 2 && !c.snapped && toWp < 700.0f && now - c.snapTry > 2000) {
                        c.snapTry = now;
                        c.snapped = SnapWaypoint(c.veh, wpPos, kind);
                    }
                    // Through a waypoint (within 150 m): on to the next one
                    if (c.wp < l.numWaypoints && toWp < 150.0f) {
                        out << Format("  [%s] waypoint %d (%s) at %.2f min, %.2f km driven\n", c.label, c.wp + 1, WaypointName(l, c.wp),
                            Minutes(), c.driven / 1000.0f);
                        c.wp++;
                        c.snapped = false;
                        c.snapTry = 0;
                        SetWaypointDestination(c.veh, l, c.wp);
                    }
                    left = TripLeft(l, c.wp, carPos);
                    if (left < c.bestLeft - 100.0f) { c.bestLeft = left; c.lastProgress = now; }
                    else if (now - c.lastProgress > 180000) {
                        out << Format("  [%s] NO PROGRESS: 3 min without getting 100 m closer (%.0f m left), at (%.0f, %.0f)\n",
                            c.label, left, c.veh->GetPosition().x, c.veh->GetPosition().y);
                        c.lastProgress = now;
                    }
                    if (c.wp >= l.numWaypoints && l.park) {
                        // The parking place: stop there (within 12 m of its node, or 60 m of the end when there's none)
                        ExtendedData& px = vehExData.Get(c.veh);
                        CPathNode* parkNode = c.snapped ? GetNode(px.destNode) : nullptr;
                        float toPark = parkNode ? (GetNodePos(parkNode).To2D() - carPos).Magnitude() : 1e9f;
                        bool there = c.snapped ? toPark < 12.0f : (c.snapTry != 0 && toWp < 60.0f); // no parking node found: the end point
                        if (there) {
                            px.speedCap = 0.01f;
                            if (!c.nearSince) c.nearSince = now;
                            if (c.veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS < 0.5f || now - c.nearSince > 20000) c.result = "ARRIVED (parked)";
                        }
                    }
                    else if (c.wp >= l.numWaypoints && left < 200.0f) c.result = "ARRIVED";
                    else if (c.still > 90.0f) { c.result = "FAIL: stopped for 90 s"; DumpHistory(c.veh, histories[c.veh]); }
                }
                if (*c.result) {
                    c.done = true;
                    c.doneMinutes = Minutes();
                    out << Format("  [%s] %s after %.2f min, %.2f km driven, %d collisions, %.0f m left\n", c.label, c.result,
                        c.doneMinutes, c.driven / 1000.0f, c.collisions, left);
                }
                else allDone = false;
            }
            if (allDone || Minutes() > routeMaxMinutes) {
                for (TripCar& c : tripCars) if (!c.done) { c.done = true; c.result = "FAIL: did not arrive in time"; c.doneMinutes = Minutes(); }
                state = DONE;
                break;
            }
            // The player rides in the followed car: Left/Right arrow switch
            static bool leftWas = false, rightWas = false;
            bool leftNow = KeyPressed(VK_LEFT), rightNow = KeyPressed(VK_RIGHT);
            int want = followed;
            if ((leftNow && !leftWas) || (rightNow && !rightWas)) want = 1 - followed;
            leftWas = leftNow;
            rightWas = rightNow;
            TripCar& f = tripCars[followed];
            auto alive = [](const TripCar& c) { return c.veh && CPools::GetVehicle(c.handle) == c.veh; };
            if (!alive(f)) want = 1 - followed;
            if (want != followed && alive(tripCars[want])) {
                followed = want;
                TripCar& t = tripCars[followed];
                CVector p = t.veh->GetPosition();
                if (player->m_pVehicle) Command<COMMAND_WARP_CHAR_FROM_CAR_TO_COORD>(PlayerHandle(), p.x, p.y, p.z + 3.0f);
                Command<COMMAND_WARP_CHAR_INTO_CAR_AS_PASSENGER>(PlayerHandle(), t.handle, 0);
                Command<COMMAND_SET_CAMERA_BEHIND_PLAYER>();
                out << Format("  following %s at %.2f min\n", t.label, Minutes());
            }
            testCar = tripCars[followed].veh;
            if (scanMode && testCar && CPools::ms_pVehiclePool->IsObjectValid(testCar) &&
                (testCar->GetPosition() - lastScanPos).Magnitude2D() >= 2.0f) {
                lastScanPos = testCar->GetPosition();
                ScanSample(testCar);
            }
            carHandle = tripCars[followed].handle;
            driverHandle = tripCars[followed].driver;
            lastCarPos = testCar ? testCar->GetPosition() : lastCarPos;
        }
        if (!testCar || !CPools::ms_pVehiclePool->IsObjectValid(testCar) || testCar->m_nStatus == STATUS_WRECKED) {
            out << "  test car lost, respawning\n";
            testCar = nullptr; carHandle = driverHandle = -1;
            state = SPAWN; stateTime = now;
            return;
        }
        testCar->m_fHealth = 1000.0f;
        float step = (testCar->GetPosition() - lastCarPos).Magnitude2D();
        if (step < 50.0f) { distanceDriven += step; locationDistance += step; }
        lastCarPos = testCar->GetPosition();
        float speed = testCar->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
        carStillTime = speed < 0.5f ? carStillTime + CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS : 0.0f;

        if (now - lastPulse > 10000) {
            lastPulse = now;
            int around = 0, moving = 0;
            for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
                CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
                if (!v || !v->m_pDriver || v == testCar) continue;
                if ((v->GetPosition() - testCar->GetPosition()).Magnitude() > 120.0f) continue;
                around++;
                if (v->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS > 1.0f) moving++;
            }
            ExtendedData& x = vehExData.Get(testCar);
            int all250 = 0, noDriver250 = 0;
            for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
                CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
                if (!v || (v->GetPosition() - testCar->GetPosition()).Magnitude2D() > 250.0f) continue;
                all250++;
                if (!v->m_pDriver) noDriver250++;
            }
            out << Format("  [pulse] vehicles within 250 m: %d, %d of them without a driver; bad spawns removed %d\n", all250, noDriver250, spawnRemovedCount);
            out << Format("  [pulse %.2f min] test car %.0f km/h target %.0f (%s) road %s, %d AI vehicles within 120m (%d moving), driven %.2f km | random cars %d (game limit %u), within 250 m %d (roads hold %d), ambient created %d\n",
                Minutes(), speed * MPS_TO_KMH, x.targetSpeed * MPS_TO_KMH, limiterNames[x.limiter], roadNames[x.roadType], around, moving,
                distanceDriven / 1000.0f, CCarCtrl::NumRandomCars, CCarCtrl::MaxNumberOfCarsInUse, ambientNear, ambientCapacity, ambientSpawnCount);
            // Running totals, to follow the test live and stop it early
            int spawnCollisions = typeCounts["just spawned (< 3 s)"] + typeCounts["test car spawn"] + typeCounts["hit parked/empty"];
            int drivingCollisions = collisionCount - spawnCollisions;
            out << Format("  [live %.2f min] driving collisions %d (%.2f/min), spawn %d, oscillations %d | lane changes %.1f/veh-km, "
                "overtakes %d started %d done %d aborted, board %d/%d denied | off route %d (no plan %d, other exit %d, not junction %d) | freeway mean %.0f km/h\n",
                Minutes(), drivingCollisions, drivingCollisions / std::max(Minutes(), 0.01f), spawnCollisions, oscillationCount,
                laneChanges / std::max(aiKm, 0.01f), overtakeCount, overtakeDoneCount, overtakeAbortCount, boardDenied, boardGranted + boardDenied, offRouteCount, offRouteNoPlan, offRouteRejected,
                offRouteNotJunction, rcTime[RC_HIGHWAY] > 1.0 ? rcSpeedSum[RC_HIGHWAY] / rcTime[RC_HIGHWAY] : 0.0);
            if (!lastCause.empty()) out << "  [live] last collision " << lastCause << "\n";
            if (locations[locIndex].route) {
                const Location& l = locations[locIndex];
                out << "  [trip]";
                for (int k = 0; k < tripCarCount; k++) {
                    TripCar& c = tripCars[k];
                    if (!c.veh || !CPools::ms_pVehiclePool->IsObjectValid(c.veh)) { out << Format(" %s: gone |", c.label); continue; }
                    ExtendedData& cx = vehExData.Get(c.veh);
                    out << Format(" %s%s: %.0f m left, %.0f km/h (%s), %.2f km, %d coll, route %d/%d, road %s, waypoint %d/%d |", k == followed ? ">" : "",
                        c.label, TripLeft(l, c.wp, c.veh->GetPosition().To2D()),
                        c.veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH, limiterNames[cx.limiter], c.driven / 1000.0f,
                        c.collisions, cx.routePos, (int)cx.route.size(), roadNames[cx.roadType], std::min(c.wp + 1, l.numWaypoints + 1), l.numWaypoints + 1);
                }
                out << "\n";
            }
            out.flush();
        }
        if (now - lastPeriodicShot > 30000 && now - lastShotTime > 3000) {
            lastPeriodicShot = now;
            TakeShot(Format("periodic_%05.2fmin", Minutes()));
        }

        // Freeway test: took an exit -> back to the freeway
        if (highwayOnly && locations[locIndex].highway) {
            offHighwayTime = RoadClassOf(testCar) == RC_HIGHWAY ? 0.0f : offHighwayTime + CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
            if (offHighwayTime > 8.0f) {
                out << "  test car left the freeway, putting it back\n";
                offHighwayTime = 0.0f;
                RemoveTestCar();
                state = SPAWN; stateTime = now;
                return;
            }
        }
        if (carStillTime > 30.0f && !locations[locIndex].route) {
            out << "  test car stopped for 30 s, re-placing it\n";
            RemoveTestCar();
            state = SPAWN; stateTime = now;
            return;
        }
        if (!locations[locIndex].route && now - locationStart > static_cast<unsigned int>(secondsPerLocation) * 1000) {
            out << Format("  left %s after %d s, %.2f km driven here\n", locations[locIndex].name, secondsPerLocation, locationDistance / 1000.0f);
            state = TELEPORT;
        }
        break;
    }
    default: break;
    }

    if (state == DONE) {
        if (routeMode) {
            for (TripCar& c : tripCars) {
                if (c.blip >= 0) Command<COMMAND_REMOVE_BLIP>(c.blip);
                if (c.veh && CPools::ms_pVehiclePool->IsObjectValid(c.veh) && c.veh != testCar) {
                    if (c.driver >= 0) Command<COMMAND_DELETE_CHAR>(c.driver);
                    Command<COMMAND_DELETE_CAR>(c.handle);
                }
            }
        }
        RemoveTestCar();
        out << "\n===== SUMMARY =====\n";
        out << Format("mode %s, %.2f min of game time, %d locations, test car drove %.2f km, %d screenshots\n",
            vanilla ? "vanilla" : "mod", Minutes(), maxLocations, distanceDriven / 1000.0f, shotsTaken);
        out << Format("collisions between vehicles: %d (%.2f per minute)\n", collisionCount, collisionCount / std::max(Minutes(), 0.01f));
        for (auto& tc : typeCounts) out << "  " << tc.first << ": " << tc.second << "\n";
        out << "  involving a mod-driven vehicle: " << involvingModCount << "\n";
        out << "  involving a reversing vehicle: " << involvingReverseCount << "\n";
        out << "collisions by probable cause:\n";
        for (auto& [cause, n] : causeCounts) out << Format("  %3d  %s\n", n, cause.c_str());
        out << Format("contacts again of the same collision (same pair within 10 s, not counted): %d\n", repeatContacts);
        if (!collisionRecs.empty()) {
            // How hard
            std::map<std::string, std::map<std::string, int>> sevByCause;
            std::map<std::string, int> sev, byRoad, byProfile;
            for (auto& r : collisionRecs) {
                sev[Severity(r.closing)]++;
                sevByCause[r.cause][Severity(r.closing)]++;
                byRoad[r.road]++;
                byProfile[r.profA]++;
                byProfile[r.profB]++;
            }
            out << Format("how hard (closing speed): touch <5 km/h %d, light 5-20 km/h %d, hard >20 km/h %d\n",
                sev["touch"], sev["light"], sev["hard"]);
            out << "by cause (touch/light/hard):\n";
            for (auto& [cause, m] : sevByCause)
                out << Format("  %-45s %3d /%3d /%3d\n", cause.c_str(), m["touch"], m["light"], m["hard"]);
            out << "by road (of the faster one):";
            for (auto& [road, n] : byRoad) out << Format(" %s %d,", road.c_str(), n);
            out << "\nby driver (either car):";
            for (auto& [prof, n] : byProfile) out << Format(" %s %d,", prof.c_str(), n);
            out << "\n";
            // Hot spots: 40 m cells, the 8 with most collisions
            std::map<std::pair<int, int>, std::vector<const CollisionRec*>> cells;
            for (auto& r : collisionRecs) cells[{ static_cast<int>(std::floor(r.x / 40.0f)), static_cast<int>(std::floor(r.y / 40.0f)) }].push_back(&r);
            std::vector<std::pair<int, std::pair<int, int>>> ranked;
            for (auto& [cell, v] : cells) ranked.push_back({ static_cast<int>(v.size()), cell });
            std::sort(ranked.rbegin(), ranked.rend());
            out << "hot spots (40 m cells, most collisions):\n";
            for (size_t i = 0; i < ranked.size() && i < 8 && ranked[i].first >= 2; i++) {
                auto& v = cells[ranked[i].second];
                std::map<std::string, int> c;
                for (auto* r : v) c[r->cause]++;
                auto top = std::max_element(c.begin(), c.end(), [](auto& x, auto& y) { return x.second < y.second; });
                out << Format("  %2d at (%.0f, %.0f), mostly \"%s\" (%d)\n", ranked[i].first, (ranked[i].second.first + 0.5f) * 40.0f,
                    (ranked[i].second.second + 0.5f) * 40.0f, top->first.c_str(), top->second);
            }
        }
        out << "oscillations (forward/back/forward within 8 s): " << oscillationCount << "\n";
        out << "jams (a mod-driven vehicle stopped > 60 s, not at a red light): " << jamCount << "\n";
        out << "blocked lanes bypassed: " << bypassCount << "\n";
        out << Format("\nAI vehicles near the camera drove %.1f km\n", aiKm);
        out << Format("lane changes: %d (%.1f per vehicle-km), %d of them in the middle of a road segment\n", laneChanges,
            laneChanges / std::max(aiKm, 0.01f), laneChangesMidLink);
        out << Format("overtakes started by the mod: %d, finished (passed the slower vehicle): %d, aborted: %d, back to the right: %d\n",
            overtakeCount, overtakeDoneCount, overtakeAbortCount, keepRightCount);
        out << Format("  overtakes given up: lane changes stopped (lane drop/junction) %d, almost stopped or hit while moving over %d, "
            "someone beside %d, someone fast coming in that lane %d, timed out or the slower one sped up %d\n", overtakeAbortWhy[0],
            overtakeAbortWhy[1], overtakeAbortWhy[2], overtakeAbortWhy[3], overtakeAbortWhy[4]);
        out << Format("lane changes for the route (exit/turn lane): %d, to the fast lane: %d, indicator on but cancelled: %d, "
            "drivers that made room for a signalling vehicle: %d, lane set by position after a junction: %d\n", prepareLaneCount,
            fastLaneCount, signalCancelCount, yieldCount, junctionLaneCount);
        out << Format("lane changes aborted by the side sensors: %d\n", sideAbortCount);
        out << Format("bikes held off a vehicle beside them: %d\n", bikeHoldCount);
        out << Format("spawned cars given the speed of the traffic behind them: %d\n", spawnSpeedMatchCount);
        out << Format("game lane set to the lane the car is really in: %d\n", laneSyncCount);
        out << Format("cars created sunk into the road, put on their wheels: %d\n", sunkSpawnFixCount);
        out << Format("lane board: signalled lane changes given up for a higher priority one: %d\n", boardPreempted);
        out << Format("cars created where they'd be hit and couldn't go with the traffic, removed: %d\n", spawnRemovedCount);
        out << Format("handbrake turns: %d\n", handbrakeTurnCount);
        out << Format("turns tightened for a vehicle on the outside: %d\n", turnGuardCount);
        out << Format("waited before a junction (the road beyond it full): %d\n", boxWaitCount);
        out << Format("waited their turn at a junction without lights (first come, first served): %d\n", firstComeWaitCount);
        out << Format("low-speed full brakes behind a vehicle: %d\n", lowSpeedBrakeCount);
        out << Format("bypasses given up (rubbing against a vehicle): %d\n", bypassScrapeCount);
        out << Format("bypasses by changing lanes (2+ lanes): %d; broken-down cars removed out of sight: %d\n", bypassLaneCount, brokenRemovedCount);
        out << Format("circles of waiting cars broken (deadlocks): %d\n", deadlockBreakCount);
        out << Format("actuated lights: %d junctions; greens ended for nobody arriving %d, at the max green %d\n",
            (int)junctionSignals.size(), signalGapOutCount, signalMaxOutCount);
        out << Format("actuated lights: switched by the safety valve (a road red too long with someone waiting) %d\n", signalForcedCount);
        out << Format("smooth lanes: steering on %d vehicle-frames\n", smoothLaneFrames);
        out << Format("sensor hits on a car in another lane, not taken as the one ahead: %d\n", otherLaneHitCount);
        out << Format("held at a light/by the game 25 s with the road clear, went through (broken light): %d\n", brokenLightCount);
        out << Format("going round through the oncoming lanes refused (two-way roads): %d\n", noOncomingBypassCount);
        out << Format("route exits the game would have refused, taken anyway: %d\n", forcedExitCount);
        out << Format("speed control: gas<->brake switches %.1f per minute of driving (%d in %.0f min)\n",
            pedalSeconds > 1.0f ? pedalSwitchCount / (pedalSeconds / 60.0f) : 0.0f, pedalSwitchCount, pedalSeconds / 60.0f);
        out << Format("ambient cars created around the player out of sight: %d\n", ambientSpawnCount);
        out << Format("lane data corrections (routefixes) loaded %d, applied %d times\n", (int)routeFixes.size(), routeFixesApplied);
        if (scanMode) out << Format("road scan: %d lines across the road in scan.txt\n", scanSamples);
        out << "asked for the way but didn't move over:";
        for (int i = 0; i < 8; i++) if (passRefusals[i]) out << Format(" %s %d,", passRefusalNames[i], passRefusals[i]);
        out << "\n";
        out << Format("routes planned again after leaving them (back onto the old route preferred): %d\n", rejoinRouteCount);
        for (int g = 1; g >= 0; g--) {
            if (!passGainCount[g]) { out << Format("pass requests (%s): no measures\n", g ? "way given" : "all"); continue; }
            out << Format("pass requests (%s): %d, speed of the one asking %.1f -> %.1f km/h 8 s later (%+.1f km/h)\n",
                g ? "way given" : "all", passGainCount[g], passStartSum[g] / passGainCount[g] * MPS_TO_KMH,
                passEndSum[g] / passGainCount[g] * MPS_TO_KMH, passGainSum[g] / passGainCount[g] * MPS_TO_KMH);
        }
        out << Format("overtakes ended by: the route's lane %d, a turn %d, target gone %d; not started (route turn too close) %d\n",
            overtakeEndRoute, overtakeEndTurn, overtakeEndLost, overtakeSkippedForRoute);
        out << Format("asked for the way: %d, moved over for a faster driver: %d; zipper merges (let one in): %d\n",
            passRequestCount, passGivenCount, zipperYieldCount);
        out << Format("lane change board: %d granted, %d requests denied (%d waits: someone else had that space); overtakes that "
            "waited their turn: %d\n", boardGranted, boardDenied, boardDeniedAttempts, overtakeWaitCount);
        out << Format("routes calculated: %d (failed %d), vehicles that left their route: %d\n", routeSearchCount, routeFailCount, offRouteCount);
        out << Format("  left the route: no plan %d, the game took another exit %d, not a junction %d; junction exits forced %d, "
            "junctions without a plan %d\n", offRouteNoPlan, offRouteRejected, offRouteNotJunction, seqPlanApplied, seqNoPlan);
        for (int r = 0; r < 2; r++) {
            if (!followTotal[r]) continue;
            int n = followTotal[r];
            out << Format("following a vehicle at about its speed (%s, %d samples): mean time gap %.2f s | <0.5 s %d%%, 0.5-1 %d%%, "
                "1-1.5 %d%%, 1.5-2 %d%%, 2-3 %d%%, >3 s %d%% | braking %d%%, mean gas %.2f\n", r == 0 ? "freeways" : "other roads", n,
                followGapSum[r] / n, followCount[r][0] * 100 / n, followCount[r][1] * 100 / n, followCount[r][2] * 100 / n,
                followCount[r][3] * 100 / n, followCount[r][4] * 100 / n, followCount[r][5] * 100 / n, followBraking[r] * 100 / n,
                followGasSum[r] / n);
        }
        out << "mean speed by road class (moving vehicles):\n";
        for (int rc = 0; rc < RC_COUNT; rc++) {
            if (rcTime[rc] > 1.0) out << Format("  %-28s %5.1f km/h (%.0f s)\n", rcNames[rc], rcSpeedSum[rc] / rcTime[rc], rcTime[rc]);
        }
        out << "drivers by type (vehicles seen / speed p50 p90 km/h / acceleration p50 p90 m/s2):\n";
        for (auto& [type, n] : driverTypeCounts) {
            auto& sp = speedSamples[type];
            auto& ac = accelSamples[type];
            out << Format("  %-12s %4d   speed %5.1f %5.1f   accel %4.2f %4.2f (%d samples)\n", type.c_str(), n,
                Percentile(sp, 0.5f), Percentile(sp, 0.9f), Percentile(ac, 0.5f), Percentile(ac, 0.9f), (int)ac.size());
        }
        if (routeMode) {
            out << "trip results:\n";
            for (int k = 0; k < tripCarCount; k++) {
                TripCar& c = tripCars[k];
                out << Format("  %-10s %s at %.2f min, %.2f km driven, %d collisions, average %.0f km/h\n", c.label, c.result,
                    c.doneMinutes, c.driven / 1000.0f, c.collisions, c.doneMinutes > 0.0f ? c.driven / 1000.0f / (c.doneMinutes / 60.0f) : 0.0f);
            }
        }
        for (int r = 0; r < 2; r++) {
            double total = 0.0;
            for (int k = 0; k < NUM_LIMITERS; k++) total += limiterTime[r][k];
            if (total < 1.0) continue;
            out << (r == 0 ? "what limits the speed on freeways (share of the time moving):" : "what limits the speed elsewhere:");
            std::vector<std::pair<double, int>> order;
            for (int k = 0; k < NUM_LIMITERS; k++) order.push_back({ limiterTime[r][k], k });
            std::sort(order.rbegin(), order.rend());
            for (int k = 0; k < 6 && k < (int)order.size(); k++) {
                if (order[k].first > 0.0) out << Format(" %s %.0f%%,", limiterNames[order[k].second], 100.0 * order[k].first / total);
            }
            out << "\n";
        }
        out << "finished " << WallClock() << "\n";
        out.flush();
        out.close();
        if (exitWhenDone) ExitProcess(0);
        return;
    }

    // Telemetry for every vehicle with a driver, and collision detection
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!v) continue;
        History& h = histories[v];
        // Pool slot reused by a new vehicle (it "jumped" far away): a new history, it just spawned
        if (const Sample* l = Latest(h)) {
            if ((v->GetPosition() - l->pos).Magnitude() > 40.0f && now - l->t < 1500) h = History();
        }
        if (!h.firstSeen) h.firstSeen = now;
        h.maxSpeed = std::max(h.maxSpeed, v->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH);
        if (now - h.lastSample >= SAMPLE_MS) { h.lastSample = now; RecordSample(v, h); }
        TrackDriving(v, now);

        // Oscillation: 3 changes between forward and backwards within 8 s
        if (v->m_pDriver && v->m_nVehicleSubClass == VEHICLE_AUTOMOBILE) {
            float fwd = CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS * MPS_TO_KMH;
            int dir = fwd > 1.5f ? 1 : (fwd < -1.5f ? -1 : 0);
            if (dir != 0 && dir != h.lastDir) {
                if (h.lastDir != 0) {
                    for (int k = 3; k > 0; k--) h.flips[k] = h.flips[k - 1];
                    h.flips[0] = now;
                    h.flipCount++;
                    if (h.flipCount >= 3 && now - h.flips[2] < 8000 && now - h.lastOscillationEvent > 10000 &&
                        (v->GetPosition() - TheCamera.GetPosition()).Magnitude() < 150.0f)
                    {
                        h.lastOscillationEvent = now;
                        // Created stuck (the game put it against a wall) and never drove: rocking there isn't the AI
                        // changing its mind on the road. Logged, but not a failure
                        bool stuckSinceSpawn = h.maxSpeed < 10.0f;
                        if (stuckSinceSpawn) {
                            typeCounts["STUCK SINCE SPAWN (never > 10 km/h, rocking)"]++;
                            out << "\n=== STUCK SINCE SPAWN t=" << Format("%.2f", Minutes()) << " min: " << Describe(v)
                                << Format(" changed direction 3 times, max speed since it appeared %.1f km/h", h.maxSpeed) << "\n";
                            DumpHistory(v, h);
                            out.flush();
                            h.flipCount = 0;
                            h.lastDir = dir;
                            continue;
                        }
                        oscillationCount++;
                        typeCounts["OSCILLATION (fwd/back/fwd)"]++;
                        out << "\n=== OSCILLATION #" << oscillationCount << " t=" << Format("%.2f", Minutes()) << " min (" << WallClock()
                            << ") at " << locations[std::max(locIndex, 0)].name << ": " << Describe(v)
                            << " changed direction 3 times in " << Format("%.1f", (now - h.flips[2]) / 1000.0f) << " s\n";
                        DumpHistory(v, h);
                        out.flush();
                        if ((v->GetPosition() - TheCamera.GetPosition()).Magnitude() < 70.0f) TakeShot(Format("oscillation_%03d", oscillationCount));
                        if (stopOnOscillation && !stopAt) {
                            stopAt = now + 1500; // time for the screenshot
                            out << "FAIL: oscillation found, stopping the test\n";
                        }
                    }
                }
                h.lastDir = dir;
            }
        }
    }

    // Jams: a vehicle driven by the mod stopped for over a minute, not waiting at a red/yellow light
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!v || !v->m_pDriver) continue;
        ExtendedData& x = vehExData.Get(v);
        if (now - x.lastProcessed > 500 || x.stillTime < 60.0f || x.waitState == 1) continue;
        if ((v->GetPosition() - TheCamera.GetPosition()).Magnitude() > 150.0f) continue;
        History& h = histories[v];
        if (now - h.lastOscillationEvent < 60000) continue;
        h.lastOscillationEvent = now;
        jamCount++;
        typeCounts["JAM (stopped > 60 s)"]++;
        out << "\n=== JAM #" << jamCount << " t=" << Format("%.2f", Minutes()) << " min (" << WallClock() << ") at "
            << locations[std::max(locIndex, 0)].name << ": " << Describe(v) << Format(" stopped %.0f s, waitState %d, queued %.1f s",
                x.stillTime, x.waitState, x.queuedTime) << "\n";
        DumpHistory(v, h);
        // Who waits for whom: the chain of leaders from this car (where each is from the one before it: ahead/side in m)
        out << "  waiting chain:";
        CVehicle* prev = v;
        CVehicle* seen[10] = { v };
        int seenCount = 1;
        for (int k = 0; k < 8; k++) {
            ExtendedData& px = vehExData.Get(prev);
            CVehicle* l = px.leaderVeh;
            if (!l || !CPools::ms_pVehiclePool->IsObjectValid(l)) { out << " -> (" << limiterNames[px.limiter] << ", no vehicle)"; break; }
            CVector2D rel = (l->GetPosition() - prev->GetPosition()).To2D();
            CVector2D f = prev->GetForward().To2D();
            ExtendedData& lx = vehExData.Get(l);
            out << Format(" -> veh#%d (%.1f ahead %.1f side, %.0f km/h, still %.0f s, %s)", CPools::GetVehicleRef(l),
                CVector2D::Dot(rel, f), f.x * rel.y - f.y * rel.x, l->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH,
                lx.stillTime, now - lx.lastProcessed < 500 ? limiterNames[lx.limiter] : "not ours");
            if (std::find(seen, seen + seenCount, l) != seen + seenCount) { out << " CIRCLE"; break; }
            seen[seenCount++] = l;
            prev = l;
        }
        out << "\n";
        out.flush();
        if ((v->GetPosition() - TheCamera.GetPosition()).Magnitude() < 70.0f) TakeShot(Format("jam_%03d", jamCount));
        if (stopOnOscillation && !stopAt) {
            stopAt = now + 1500;
            out << "FAIL: jam found, stopping the test\n";
        }
    }

    // Contacts: bounding boxes touching. A new contact of a pair counts once they were apart for 0.3 s
    auto testable = [](CVehicle* v) {
        int sc = v->m_nVehicleSubClass;
        return sc != VEHICLE_TRAIN && sc != VEHICLE_HELI && sc != VEHICLE_PLANE && sc != VEHICLE_BOAT;
    };
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* a = CPools::ms_pVehiclePool->GetAt(i);
        if (!a || !testable(a) || (a->GetPosition() - TheCamera.GetPosition()).Magnitude() > 150.0f) continue;
        for (int j = i + 1; j < CPools::ms_pVehiclePool->m_nSize; j++) {
            CVehicle* b = CPools::ms_pVehiclePool->GetAt(j);
            if (!b || !testable(b) || (!a->m_pDriver && !b->m_pDriver)) continue;
            if ((a->GetPosition() - b->GetPosition()).Magnitude2D() > 14.0f) continue;
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            bool touching = BoxesTouch(a, b, 0.05f);
            bool wasTouching = contactPairs.count(key) != 0;
            if (touching && !wasTouching) {
                contactPairs[key] = now;
                auto ap = apartSince.find(key);
                if (ap == apartSince.end() || now - ap->second > 300) {
                    CVector rel = (a->m_vecMoveSpeed - b->m_vecMoveSpeed) * GAME_SPEED_TO_MPS * MPS_TO_KMH;
                    // The same pair again within 10 s: the same collision going on (two cars stuck against each other
                    // counted six times), only noted
                    auto ep = pairEpisodes.find(key);
                    if (ep != pairEpisodes.end() && now - ep->second.first < 10000) {
                        ep->second.first = now;
                        repeatContacts++;
                        out << Format("  (contact again of collision #%d, %.1f km/h, %.2f min)\n", ep->second.second, rel.Magnitude2D(), Minutes());
                    }
                    else {
                        OnCollision(a, b, rel.Magnitude2D());
                        pairEpisodes[key] = { now, collisionCount };
                    }
                }
            }
            else if (!touching && wasTouching) {
                contactPairs.erase(key);
                apartSince[key] = now;
            }
        }
    }
    // Forget removed vehicles
    if (frameCounter % 300 == 0) {
        for (auto it = histories.begin(); it != histories.end();) {
            if (!CPools::ms_pVehiclePool->IsObjectValid(it->first)) it = histories.erase(it);
            else ++it;
        }
        // Pool slots are reused: a removed vehicle's track must not continue on the next one
        for (auto it = driveTracks.begin(); it != driveTracks.end();) {
            if (!CPools::ms_pVehiclePool->IsObjectValid(it->first)) { countedDriver.erase(it->first); it = driveTracks.erase(it); }
            else ++it;
        }
        for (auto it = pairEpisodes.begin(); it != pairEpisodes.end();) {
            if (!CPools::ms_pVehiclePool->IsObjectValid(it->first.first) || !CPools::ms_pVehiclePool->IsObjectValid(it->first.second) ||
                now - it->second.first > 10000) it = pairEpisodes.erase(it);
            else ++it;
        }
        for (auto* m : { &contactPairs, &apartSince }) {
            for (auto it = m->begin(); it != m->end();) {
                if (!CPools::ms_pVehiclePool->IsObjectValid(it->first.first) || !CPools::ms_pVehiclePool->IsObjectValid(it->first.second) ||
                    (m == &apartSince && now - it->second > 5000)) it = m->erase(it);
                else ++it;
            }
        }
    }
}

} // namespace AutoTest
