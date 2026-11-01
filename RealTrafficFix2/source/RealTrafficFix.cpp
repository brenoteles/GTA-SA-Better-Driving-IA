/*
    Real Traffic Fix (by Junior_Djjr / MixMods)
    https://www.mixmods.com.br/2019/12/Real-Traffic-Fix.html

    Port for the current Plugin-SDK (GTA SA 1.0 US only).

    The mod never touches physics or handling: it only changes what the game's own AI wants to do
    (cruise speed, driving style and lanes). Engine, brakes and tyres are 100% vanilla.
*/
#include "plugin.h"
#include "CAutomobile.h"
#include "CBike.h"
#include "CCamera.h"
#include "CCarCtrl.h"
#include "CCoronas.h"
#include "CFont.h"
#include "CGeneral.h"
#include "CModelInfo.h"
#include "CPathFind.h"
#include "CPools.h"
#include "CSprite.h"
#include "CStreaming.h"
#include "CTimer.h"
#include "CWeather.h"
#include "CWorld.h"
#include "SurfaceInfos_c.h"
#include "CGame.h"
#include "CCutsceneMgr.h"
#include "CVehicleModelInfo.h"
#include "extensions/ScriptCommands.h"
#include "GameVersion.h"
#include "extender/VehicleExtender.h"
#include "IniReader/IniReader.h"

#include <algorithm>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <queue>
#include <sstream>
#include <cstring>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace plugin;

static constexpr const char* MOD_VERSION = "v4.0.0 (Plugin-SDK port)";

// Unit conversions. Game speed is in units per 1/50 s; CAutoPilot cruise speed is game speed * 60
static constexpr float GAME_SPEED_TO_MPS = 50.0f;
static constexpr float MPS_TO_CRUISE = 60.0f / 50.0f;
static constexpr float MPS_TO_KMH = 3.6f;
static constexpr float GRAVITY = 9.81f;
static constexpr float MAGIC = 50.0f / 30.0f;
static constexpr int MAX_LOOKAHEAD_NODES = 32;
static constexpr float MAX_LOOKAHEAD_DIST = 450.0f;
static constexpr float LANE_WIDTH = 5.4f;  // CCarCtrl lane width @ 0x858C50
static constexpr int MAX_RAYS = 10;
static constexpr int MAX_LINE_POINTS = 2 * MAX_LOOKAHEAD_NODES + 2;
static constexpr int MAX_ROUTE_PLANS = 16; // junctions ahead with a chosen exit (the route is read ~400 m ahead)

enum eSpeedLimiter : unsigned char {
    LIMIT_NONE, LIMIT_DESIRED, LIMIT_LEADER, LIMIT_CURVE, LIMIT_JUNCTION, LIMIT_BUMP, LIMIT_BICYCLE, LIMIT_HEADING, LIMIT_LIGHT,
    LIMIT_CREST, LIMIT_CROSS_TRAFFIC, LIMIT_DAMAGE, LIMIT_ROAD, LIMIT_CAR_TOP, LIMIT_BYPASS, LIMIT_YIELD, LIMIT_OVERTAKE,
    NUM_LIMITERS
};
static const char* limiterNames[] = { "-", "desired", "leader", "curve", "junction", "bump", "bicycle", "heading", "light",
    "crest", "cross traffic", "damage", "road", "car top", "bypass", "yield", "overtaking" };

enum eRoadType : unsigned char { ROAD_UNKNOWN, ROAD_DIRT, ROAD_NARROW, ROAD_STREET, ROAD_RURAL, ROAD_AVENUE, ROAD_FAST, ROAD_HIGHWAY };
static const char* roadNames[] = { "?", "dirt", "narrow", "street", "rural", "avenue", "fast road", "highway" };

// Driver profiles. Each driver gets one when the vehicle is created (weights in the ini, biased by vehicle type and driver)
enum eDriverProfile : unsigned char {
    PROFILE_CALM, PROFILE_SAFE, PROFILE_NORMAL, PROFILE_FAST, PROFILE_AGGRESSIVE, PROFILE_FLEEING, PROFILE_MAX_PERFORMANCE, NUM_PROFILES
};
static const char* profileNames[NUM_PROFILES] = { "calm", "safe", "normal", "fast", "aggressive", "fleeing", "max perf" };
static const char* profileSections[NUM_PROFILES] = { "Profile.Calm", "Profile.Safe", "Profile.Normal", "Profile.Fast",
    "Profile.Aggressive", "Profile.Fleeing", "Profile.MaxPerformance" };

struct DriverProfile {
    int chance;                 // relative weight
    float speedMult;            // x the road's speed
    float accel;                // m/s2 the driver accelerates with. >= FULL_THROTTLE_ACCEL: no limit (pedal to the metal)
    float decel;                // m/s2 comfortable braking (curves, lights, car ahead)
    float timeGap;              // s to the vehicle ahead
    float minGap;               // m standing still
    float cornerGrip;           // lateral g in corners (x tyre traction); racing logic: fraction of the learned grip
    float topSpeedUse;          // fraction of the car's top speed it will use
    float overtakeRatio;        // overtakes vehicles slower than this fraction of the speed it wants. 0 = never
    float laneChangeInterval;   // s at least between two lane changes of its own
    float aggression;           // 0..1: yellow lights, how hard it uses the pedals, safety margins of lane changes
    int keepRight;              // 2 = always on the right (left only for the route), 1 = goes back to the right (slow) lanes,
                                // 0 = stays, -1 = prefers the fast lane
    float lightApproach;        // km/h max through a green light
    bool racingLogic;           // route planning, racing line, learned limits
    bool ignoreRoadLimit;       // only curves and the car limit its speed
    bool runRedLights;          // doesn't stop at lights (still slows down for cross traffic)
};
static constexpr float FULL_THROTTLE_ACCEL = 6.0f;
static constexpr float MIN_LANE_CHANGE_SPEED = 8.0f;  // m/s (29 km/h)
static constexpr float MIN_PREPARE_SPEED = 4.0f;      // m/s, changing to the lane the route needs
static constexpr float RED_LIGHT_RUN_SPEED = 35.0f / 3.6f; // m/s, drivers that run red lights, through them
static DriverProfile profiles[NUM_PROFILES] = {
    //  chance speed accel decel gap  min  grip  top   overtake  interval aggr keepR light  racing ignoreRoad redLights
    {   10,   0.85f, 1.2f, 2.0f, 0.78f, 1.5f, 0.36f, 0.78f, 0.65f, 25.0f, 0.00f,  2, 40.0f, false, false, false }, // calm
    {   25,   0.95f, 1.6f, 2.5f, 0.63f, 1.2f, 0.44f, 0.86f, 0.80f, 15.0f, 0.25f,  2, 45.0f, false, false, false }, // safe
    {   35,   1.00f, 2.0f, 3.0f, 0.53f, 1.0f, 0.52f, 0.94f, 0.85f, 10.0f, 0.50f,  1, 50.0f, false, false, false }, // normal
    {   15,   1.15f, 2.8f, 3.5f, 0.4f, 0.9f, 0.62f, 0.98f, 0.92f,  8.0f, 0.70f,  1, 60.0f, false, false, false }, // fast
    {   10,   1.30f, 3.5f, 4.5f, 0.3f, 0.8f, 0.72f, 1.00f, 0.97f,  6.0f, 0.95f,  1, 70.0f, false, false, false }, // aggressive
    {    2,   1.70f, 9.0f, 6.0f, 0.3f, 2.0f, 0.88f, 1.00f, 0.98f,  4.0f, 1.00f,  1, 90.0f, true,  false, true  }, // fleeing
    {    3,   1.00f, 9.0f, 7.0f, 0.3f, 3.0f, 0.95f, 1.00f, 0.95f,  6.0f, 0.80f,  1, 75.0f, true,  true,  false }, // max performance
};

// Lane changes of the mod (overtaking, keeping right)
enum eLaneState : unsigned char { LANE_KEEP, LANE_SIGNAL_OVERTAKE, LANE_OVERTAKING, LANE_SIGNAL_MOVE, LANE_CHANGING };
static const char* laneStateNames[] = { "", "signal (overtake)", "OVERTAKING", "signal", "CHANGING LANE" };

// CAutoPilot::m_nTempAction values handled by CCarCtrl::SteerAICarWithPhysics (jump table @ 0x438310)
static constexpr char TEMPACT_REVERSE_LEFT = 13;  // reverse with full left lock, gas -0.75
static constexpr char TEMPACT_REVERSE_RIGHT = 14; // reverse with full right lock

enum eAidFlags : unsigned char { AID_ABS = 1, AID_TC = 2, AID_ESC = 4, AID_COUNTERSTEER = 8, AID_PEDALS = 16, AID_ACCEL = 32 };

enum eLightColor : unsigned char { LIGHT_GREEN, LIGHT_YELLOW, LIGHT_RED, LIGHT_NONE };
static const char* lightNames[] = { "GREEN", "YELLOW", "RED", "-" };

enum eObstacleType : unsigned char {
    OBSTACLE_NONE, OBSTACLE_VEHICLE, OBSTACLE_PED, OBSTACLE_OBJECT, OBSTACLE_BUILDING, OBSTACLE_FORCED
};
static const char* obstacleNames[] = { "free", "vehicle", "ped", "object", "building", "FORCED" };

class ExtendedData {
public:
    bool bInitialized = false;
    bool forceDrive = false;        // driven by the mod although it's a mission vehicle (AutoTest's trip cars: the game never removes those)
    bool bRacer = false;            // profile with the racing logic (route planning, racing line, learned limits)
    eDriverProfile profile = PROFILE_NORMAL;
    float speedVariation = 1.0f;    // this driver's own spread around the profile's speed
    bool bPhysics = false;
    bool bRunGroundStuckFix = true;
    bool bFoundBump = false;
    int framesDelayCountSearchBump = 0;

    // Speed control, all in m/s
    float targetSpeed = 0.0f;
    float desiredSpeed = 0.0f;
    eSpeedLimiter limiter = LIMIT_NONE;

    // Cached sensing (refreshed every few frames)
    eObstacleType obstacle = OBSTACLE_NONE;
    float gap = 0.0f;           // m, front bumper to obstacle
    float leaderSpeed = 0.0f;   // m/s, along our forward
    // Last real vehicle/ped seen ahead: one sensor reading that misses it (ray grazing a bumper) doesn't make it vanish
    CVehicle* leaderVeh = nullptr;      // vehicle seen ahead (check it's still in the pool before using it)
    CVehicle* memLeaderVeh = nullptr;
    CNodeAddress forcedExit;           // the route's exit at the junction being decided (see ForceRouteExit)
    CVehicle* deadlockIgnore = nullptr; // the vehicle it waits for in a circle of waiting cars: not seen for a while
    unsigned int deadlockUntil = 0;
    unsigned char noseReverses = 0;     // nose-to-nose standoffs it backed out of
    unsigned int ignoreLightsUntil = 0; // a light that never turns green (or the game holding it for no reason): ignored
    eObstacleType memObstacle = OBSTACLE_NONE;
    float memGap = 0.0f, memLeaderSpeed = 0.0f;
    unsigned int memTime = 0;
    float curveSpeed = 1000.0f; // m/s, max speed allowed now because of curves/junctions ahead
    float curveDist = 0.0f;     // m, distance to the limiting point
    bool curveIsJunction = false;
    bool curveIsCrest = false;
    float lightSpeed = 1000.0f; // m/s, max speed allowed now because of the next traffic light
    float lightDist = 0.0f;     // m, distance to the stop line
    eLightColor lightColor = LIGHT_NONE;
    CVector lightPos;
    float headingSpeed = 1000.0f; // m/s, max speed for the turn the vehicle is doing right now
    bool valid = false;

    // Destination: every driver is going somewhere, chosen when it appears, and its route there is calculated (A* on
    // the game's road graph). The route decides the exit of every junction and the lane to be in before it
    CVector2D destination;
    CNodeAddress destNode;
    bool hasDestination = false;
    bool fixedDestination = false;    // given from outside (AutoTest): kept until it gets there, even far beyond the loaded roads
    float goalRadius = 0.0f;          // > 0: any node this close to the destination is the end of the route (a waypoint)
    bool goalFreeway = false;         // ...and it must be a freeway node
    unsigned int destinationTime = 0;
    std::vector<CNodeAddress> route;  // from the node it came from to the destination (or as close as the loaded roads go)
    std::vector<CNodeAddress> oldRoute; // what was left of the route when it went off it (to get back onto it)
    int routePos = 0;                 // index of the current node in the route
    bool routeComplete = false;       // the route reaches the destination
    unsigned int routeTime = 0;       // when it was calculated

    // Route planning: exit chosen for each junction ahead
    struct RoutePlan { CNodeAddress junction, from, exit; };
    RoutePlan plans[MAX_ROUTE_PLANS];
    int planCount = 0;

    // Racing line: taut string inside our own lanes
    CVector line[MAX_LINE_POINTS];
    int lineCount = 0;
    float lineCurveSpeed = 1000.0f;
    float lineCurveDist = 0.0f;
    bool inCorridor = false;
    float lineBlend = 0.0f;       // 0 = game steering, 1 = racing line steering

    // Adaptive limits / driver aids
    float prevSpeed = 0.0f;
    CVector2D prevVel;
    unsigned int prevSampleFrame = 0;
    float filtLat = 0.0f;   // m/s2, smoothed
    float filtLong = 0.0f;  // m/s2, smoothed
    float filtYawRate = 0.0f; // rad/s, smoothed (heading limiter)
    float tcCut = 0.0f;       // traction control: share of the gas taken away (0..0.8)
    unsigned char aidFlags = 0;   // eAidFlags

    // Acceleration limit of the profile (pedal control)
    float accelLimit = 0.0f;      // m/s2, 0 = no limit
    float closeUpTarget = -1.0f;  // m/s, creeping up to a stopped vehicle in a queue (< 0: not)
    float measAccel = 0.0f;       // m/s2 along the forward, smoothed
    float prevFwdSpeed = 0.0f;
    unsigned int prevFwdFrame = 0;
    float gasTrim = 0.0f;         // gas above the cruise gas that gives accelLimit
    float speedTrim = 0.0f;       // speed control: gas the cruise estimate is off by (integral term)
    float pedalGas = 0.0f;        // speed control: last gas given (moved gradually)
    bool pedalBraking = false;    // speed control: braking for being too fast (hysteresis)
    signed char pedalSide = 0;    // speed control: last pedal really pressed (1 gas, -1 brake), to count switches
    bool accelLimiting = false;   // the limiter was running last frame

    // Personality: 0 = calm, 1 = very aggressive
    // Waiting, and getting around a vehicle that blocks the lane (crashed, broken down, abandoned)
    unsigned char waitState = 0;        // 0 not waiting, 1 red light/cross traffic, 2 queued behind a vehicle
    float queuedTime = 0.0f;            // s queued behind a stopped vehicle
    float stillTime = 0.0f;             // s stopped
    unsigned int lastProcessed = 0;     // last frame this mod drove it
    unsigned int bypassUntil = 0;       // != 0: getting around 'bypassBlocker' with the bike logic (driving style 6)
    unsigned int noBypassUntil = 0;     // gave up a bypass (rubbing against someone): not again before this
    CVehicle* bypassBlocker = nullptr;
    unsigned char savedStyle = 0;

    float aggression = 0.5f;      // 0 = calm, 1 = very aggressive (from the profile)
    bool runsRedLights = false;

    // Grip/speed modifiers this frame
    float surfaceFactor = 1.0f;   // surface + rain
    float damageFactor = 1.0f;    // < 1 when damaged/burst tyre
    float groundGrip = 1.0f;      // surface only (no rain), smoothed: < 1 off tarmac

    // Road being driven on (refreshed with the sensors)
    bool bRoadChecked = false;
    eRoadType roadType = ROAD_UNKNOWN;
    float roadLimit = 1000.0f;    // m/s, speed this road allows for this vehicle/driver
    float carTopSpeed = 1000.0f;  // m/s, speed this vehicle can keep

    // Pedals of the last FollowPath call: what the game asked and what was applied after the mod (AutoTest telemetry)
    float dbgGameGas = 0.0f, dbgGameBrake = 0.0f, dbgGas = 0.0f, dbgBrake = 0.0f;
    unsigned int dbgPedalTime = 0;

    // Cross traffic at the junction ahead
    float crossSpeed = 1000.0f;
    float crossDist = 0.0f;
    CVector crossPos;

    // Lane changes: overtaking and keeping right
    eLaneState laneState = LANE_KEEP;
    int pendingLane = -1;             // lane it's signalling to go to
    unsigned char pendingAction = 0;  // laneAction of the pending move
    unsigned int signalUntil = 0;     // earliest moment to change (the indicator has been on long enough)
    unsigned int signalStart = 0;
    // Lane change in progress (LANE_CHANGING): the mod steers it smoothly from where it was to the target lane
    int changeTarget = -1, changeOrigin = -1;
    float changeFromF = 0.0f;         // lane position (in lanes) when it started
    unsigned int changeStart = 0;
    float changeDuration = 4.0f;      // s
    bool spawnChecked = false;        // MatchSpawnSpeed done
    float speedCap = 0.0f;            // m/s, > 0: never faster (AutoTest's road scan)
    unsigned int lastPassRequest = 0; // asked the one ahead for the way
    unsigned int newLinkTime = 0;     // when it went onto the current road segment
    CVehicle* passRequester = nullptr; // the faster one behind that asked us for the way
    unsigned int lastPassRefusal = 0;
    unsigned int laneMismatchSince = 0;
    unsigned int bikeHoldUntil = 0;   // bike weaving towards a vehicle beside it: held parallel to the road until then
    CVector2D bikeRoadDir;
    eLaneState changeAfter = LANE_KEEP;
    float blockedTime = 0.0f;         // s behind a vehicle slower than it wants
    unsigned int overtakePrepUntil = 0; // held up with a lane to pass in: closes up on the one ahead (slipstream, momentum)
    unsigned int lastBoardDenied = 0;
    unsigned int lastLaneTick = 0;
    // Side sensors (debug)
    CVector sideRayA[3], sideRayB[3];
    CRGBA sideRayColor[3];
    int numSideRays = 0;
    unsigned int sideRayTime = 0;
    int returnLane = -1;              // lane to go back to after passing
    CVehicle* overtakeTarget = nullptr;
    unsigned int overtakeStart = 0;
    unsigned int lastLaneChange = 0;
    bool trafficAround = false;       // any vehicle beside/close: racing line off
    unsigned char laneAction = 0;     // debug: 1 = overtaking, 2 = back to the right, 3 = to the fast lane
    int lanesHere = 0;
    float distToJunctionAhead = 1000.0f; // m along the road (for rural roads)
    // Lane needed for the next turn/exit of the route: -1 = leftmost lane, 1 = rightmost, 0 = any
    int requiredSide = 0;
    float requiredDist = 1000.0f;     // m to the junction where it's needed
    int turnSide = 0;                 // the next planned turn (> 30 degrees): -1 left, 1 right
    float turnDist = 1000.0f;
    float turnAngle = 0.0f;           // rad, of that turn
    CVector2D turnExitDir;            // direction of the road it turns into
    bool handbrakeCapable = false;    // fast and aggressive drivers of ordinary cars: handbrake turns
    unsigned int handbrakeUntil = 0;  // handbrake turn in progress until then
    unsigned int handbrakeLast = 0;
    unsigned int turnGuardTime = 0;   // last time the turn side guard acted
    unsigned int boxWaitTime = 0;     // last time it waited before a junction blocked beyond it
    unsigned int firstComeWaitTime = 0; // last time it waited its turn at a junction without lights
    unsigned int lowBrakeTime = 0;    // last time the low-speed brake acted
    int dropLanes = 0;                // the road ahead loses lanes: lanes left, and where
    float dropDist = 1000.0f;
    UInt16 lastNextLink = 0xFFFF;     // to see when it starts a new road segment
    // Turn signals: -1 left, 1 right
    int indicator = 0;
    unsigned int indicatorUntil = 0;
    // Letting a vehicle that signals into our lane in (cooperative drivers)
    CVehicle* yieldTo = nullptr;
    float yieldGap = 1000.0f, yieldSpeed = 0.0f;
    unsigned int yieldTime = 0;

    // Crash recovery
    float stuckTime = 0.0f;
    float flippedTime = 0.0f;
    int recoveries = 0;
    unsigned int lastRecoveryTime = 0;

    // Debug
    CVector rayA[MAX_RAYS], rayB[MAX_RAYS];
    CRGBA rayColor[MAX_RAYS];
    int numRays = 0;
    CVector path[MAX_LOOKAHEAD_NODES + 1];
    int pathCount = 0;
    // Car path link of each segment path[i] -> path[i+1] of the route ahead: lanes are measured from it
    CVector2D pathLinkPos[MAX_LOOKAHEAD_NODES];
    float pathLaneOffset[MAX_LOOKAHEAD_NODES];
    bool pathLinkOk[MAX_LOOKAHEAD_NODES];
    CCarPathLink* pathLink[MAX_LOOKAHEAD_NODES];
    int limitingNode = -1;

    ExtendedData(CVehicle*) {}
};
static VehicleExtendedData<ExtendedData> vehExData;

////////////////////////
static bool cfgTestMode = true;
static int cfgDebugLevel = 1;   // 1 = compact (one line per vehicle, details for the one closest to the camera), 2 = everything
static int cfgDebugToggleKey = VK_F9;
static float cfgDebugDistance = 40.0f;
static bool cfgOnlyInNormal = false;
static bool cfgFixGroundStuck = true;
static int cfgGroundStuckAttempts = 6;
static bool cfgUseBikeLogicOnCars = false;
static bool cfgUseBikeLogicOnBicycles = true;
static bool cfgBicyclesDontStopForRed = false;
static bool cfgNoBicycleHorns = true;
static bool cfgCheckObjects = false;
static bool cfgSlowVehiclesSlowLane = true;
static bool cfgAvoidVehiclesChangingLane = true;
static float cfgTurningSpeedDecrease = 15.0f;
static float cfgCruiseMinSpeed = 3.0f;
static float cfgCruiseMaxSpeed = 30.0f; // only without RoadSpeedLimits
static float cfgFrontMultDist = 50.0f;
static float cfgSidesSpeedOffsetDiv = 70.0f;
static float cfgCheckGroundHeight = 0.5f;
static float cfgFinalGroundHeight = 0.5f;
static float cfgHeightDiffLimit = 3.0f;
static float cfgBicycleSpeedMult = 0.5f;
static float cfgBicycleCruiseMaxSpeed = 10.0f;
static int cfgHornsThreshold = 100;
static int cfgBumpModelStart = 0;
static int cfgBumpModelEnd = 0;
static int cfgSearchBumpFramesDelay = 30;
static float cfgBumpSearchRadius = 6.0f;
static float cfgBumpSearchDistMult = 0.3f;
static float cfgBumpSpeedDiv = 1.5f;
// Game AI replacement
static bool cfgReplaceGameCurveBraking = true;
static bool cfgHeadingLimiter = true;
// Traffic lights
static bool cfgTrafficLightDetection = true;
static float cfgLightStopMargin = 1.5f;          // m
static float cfgHeadingGripMargin = 1.35f;       // heading limiter grip = curve grip x this
// Ambient traffic: cars created on the roads all around the player, out of sight (behind, beside, in the side streets),
// not only in front of the camera at the end of the draw distance like the game does
static bool cfgAmbientSpawn = true;
static float cfgAmbientMinDist = 80.0f;      // m from the player
static float cfgAmbientMaxDist = 220.0f;
static int cfgAmbientInterval = 100;         // ms between two
static float cfgAmbientFill = 1.0f;          // (old: share of the game's random car limit, unused)
static int cfgAmbientTarget = 70;            // cars with a driver within 250 m of the player (at most)
static float cfgAmbientLaneSpacing = 45.0f;  // m of lane per car: the target is also what the roads within 250 m hold
static int ambientNear = 0, ambientCapacity = 0;
static bool cfgHandbrakeTurns = true;            // fast/aggressive drivers use the handbrake in tight turns
static float cfgHandbrakeEntryBoost = 1.0f;     // x the speed into tight junction turns for them
static float cfgJunctionGripFactor = 0.75f;      // x the curve grip in junction turns
static bool cfgV2VOvertakes = true;              // overtakes planned with every car's position/speed/intent
static bool cfgTurnSideGuard = true;             // turning: watch the outside of the turn for vehicles there
static int turnGuardCount = 0, boxWaitCount = 0, firstComeWaitCount = 0, lowSpeedBrakeCount = 0, bypassScrapeCount = 0;
static int bypassLaneCount = 0, brokenRemovedCount = 0, deadlockBreakCount = 0, noOncomingBypassCount = 0, brokenLightCount = 0;
static int otherLaneHitCount = 0; // sensor hits on a car in another lane, not taken as the one ahead
static bool cfgJunctionFirstCome = true;          // junctions without lights: first come, first served
// One list for all junctions: a big intersection is several junction nodes close together (each way across it has its
// own), the claims of all of them within 30 m count together
struct JunctionClaim { CVehicle* veh; unsigned int arrival, lastSeen, stillSince; CVector2D dir; CVector2D pos; };
static std::vector<JunctionClaim> junctionClaims;
static float cfgPassRequestMinDiff = 20.0f;      // km/h: asks the one ahead for the way only wanting this much more
static float cfgCurveUseMin = 0.55f, cfgCurveUseMax = 0.85f;          // share of the car's real grip in curves (calm..aggressive)
static float cfgCurveBrakeUseMin = 0.45f, cfgCurveBrakeUseMax = 0.65f; // share of its real braking before curves/lights
static float cfgBikeGapExtra = 1.5f;             // m, extra stop gap behind motorbikes and bicycles
// Sensors
static float cfgSensorHeight = 0.8f;   // m above the ground for the terrain-following center line
static float cfgWallNormalZ = 0.6f;    // hits on surfaces facing up more than this are ground/ramps, not walls
// Racer intelligence
static bool cfgRoutePlanning = true;
static bool cfgRacingLine = true;
static float cfgRacingLineMargin = 0.8f;   // m kept from the lane edges (besides half the vehicle width)
static bool cfgAdaptiveLimits = true;
static float cfgLearnedDecelUse = 0.85f;
// Driver aids
static bool cfgABS = true;
static bool cfgTractionControl = true;
static bool cfgStabilityControl = true;
static bool cfgCounterSteer = true;
// Driving model
static bool cfgPedalControl = true;
static bool cfgFrictionCircle = true;
static bool cfgSlopeAware = true;
static float cfgCrestFactor = 0.85f;
static bool cfgSurfaceGrip = true;
static float cfgRainGripLoss = 0.3f;
static bool cfgDamageAware = true;
// Traffic
static bool cfgProfiles = true;            // 0 = every driver is "normal"
static float cfgPersonalitySpread = 1.0f;  // variation between drivers of the same profile (0..1)
static bool cfgJunctionProtection = true;
static bool cfgCrashRecovery = true;
static float cfgStuckTime = 3.0f;      // s trying to move without moving before reversing
static float cfgFlipResetTime = 4.0f;  // s upside down before being put back on the road (only off screen)
// Performance
static int cfgSensorInterval = 2;
static int cfgSensorIntervalFar = 5;
static float cfgSensorFarDistance = 60.0f;
// Curves
static bool cfgCurveLookahead = true;
static float cfgMaxLookaheadDist = 150.0f;
// Profiles with the racing logic (fleeing, max performance)
static float cfgRacerMaxSpeed = 320.0f; // km/h
static float cfgRacerJunctionSpeed = 80.0f; // km/h
// A junction ahead it has no plan for (off its route): not faster than its sharpest exit allows, nor slower than this.
// Without it a fast driver came into an unplanned turn at 108 km/h and hit the queue round the corner
static float cfgUnplannedJunctionSpeed = 45.0f; // km/h
static float cfgRacerSideRayLength = 12.0f;
static int cfgRacerDrivingStyle = DRIVINGSTYLE_SLOW_DOWN_FOR_CARS; // cars: no weaving, overtaking is done by changing lanes
static int cfgRacerBikeDrivingStyle = 5;                           // bikes: DRIVINGSTYLE_AVOID_CARS_OBEY_LIGHTS (weave through traffic)
static bool cfgOvertaking = true;
static float cfgOvertakeRearTime = 3.5f;    // s a faster vehicle coming from behind in the target lane needs to reach us
static bool cfgKeepRight = true;            // drivers go back to the right lanes when they're free (by profile)
static bool cfgDestinations = true;         // every driver has a destination; junction exits and lanes are chosen for it
static bool cfgTurnSignals = true;          // indicators before lane changes and turns
static bool cfgYieldToSignals = true;       // cooperative drivers make room for a vehicle signalling into their lane
static float cfgLanePrepareTime = 12.0f;    // s before a turn/exit to be in its lane (at least 80 m)
static int cfgDestinationWideRoads = 80;    // % of drivers going somewhere on a road with 2+ lanes
static bool cfgLoadAllPathNodes = true;     // keep the road nodes of the whole map loaded, so routes can be long
static unsigned char allPathAreas[1024];    // "to be streamed" flags the game reads: all areas (see constructor)
static bool cfgLaneDiscipline = true;       // no random lane changes of the game AI: only the mod's (overtaking, keeping right)
static bool cfgLaneBoard = true;            // drivers reserve the road their lane change needs (no two going for the same space)
// Getting around a blocked lane
static bool cfgBypassBlocked = true;
static float cfgBypassWait = 6.0f;     // s queued behind a blocking vehicle before going around it
static float cfgBypassSpeed = 20.0f;   // km/h
static int bypassCount = 0;
// Lane change statistics (AutoTest)
static int overtakeCount = 0, overtakeDoneCount = 0, overtakeAbortCount = 0, keepRightCount = 0;
static int prepareLaneCount = 0, fastLaneCount = 0, signalCancelCount = 0, yieldCount = 0, junctionLaneCount = 0;
static int sideAbortCount = 0, pathLeaderCount = 0, predictedConflictCount = 0, overtakeWaitCount = 0, bikeHoldCount = 0, spawnSpeedMatchCount = 0, laneSyncCount = 0, sunkSpawnFixCount = 0;
static int spawnRemovedCount = 0;
static int handbrakeTurnCount = 0;
// Why a driver asked for the way didn't move over (once per request): no right lane, junction/light close, just changed
// lane, no room on the right, the board said no, busy (signalling/overtaking), in the lane its route needs, can't change lanes
static int passRefusals[8] = {};
static const char* passRefusalNames[8] = { "no lane on the right", "junction/light close", "just changed lane",
    "no room on the right", "lane board said no", "busy (signalling/overtaking)", "keeping the lane its route needs",
    "doesn't change lanes (big vehicle/bike)" };
static int rejoinRouteCount = 0;
static int ambientSpawnCount = 0;
static float cfgRejoinRouteFactor = 0.6f;        // cost of the old route's roads when planning again after leaving it
static std::vector<CVehicle*> spawnRemovals;
// Does asking for the way work? Speed of the one that asked, when it asked / when the way was given, and 8 s later
struct PassMeasure { CVehicle* veh; float startSpeed; unsigned int start; bool given; };
static std::vector<PassMeasure> passMeasures;
static double passGainSum[2] = {}, passStartSum[2] = {}, passEndSum[2] = {};
static int passGainCount[2] = {};  // [0] every request, [1] way given

static void StartPassMeasure(CVehicle* veh, bool given) {
    if (!veh || !CPools::ms_pVehiclePool->IsObjectValid(veh)) return;
    passMeasures.push_back({ veh, veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS, CTimer::m_snTimeInMilliseconds, given });
}

static void UpdatePassMeasures() {
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    for (auto it = passMeasures.begin(); it != passMeasures.end();) {
        if (now - it->start < 8000) { ++it; continue; }
        if (CPools::ms_pVehiclePool->IsObjectValid(it->veh)) {
            float end = it->veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
            int g = it->given ? 1 : 0;
            passGainSum[g] += end - it->startSpeed;
            passStartSum[g] += it->startSpeed;
            passEndSum[g] += end;
            passGainCount[g]++;
        }
        it = passMeasures.erase(it);
    }
}
static int passRequestCount = 0, passGivenCount = 0, zipperYieldCount = 0;
static int overtakeEndRoute = 0, overtakeEndTurn = 0, overtakeEndLost = 0, overtakeSkippedForRoute = 0;
// Max speed by road type (km/h) and vehicle capability
static bool cfgRoadSpeedLimits = true;
static float cfgRoadSpeedHighway = 150.0f;
static float cfgRoadSpeedFastRoad = 130.0f;
static float cfgRoadSpeedAvenue = 110.0f;
static float cfgRoadSpeedRural = 120.0f;
static float cfgRoadSpeedStreet = 70.0f;
static float cfgRoadSpeedNarrow = 50.0f;
static float cfgRoadSpeedDirt = 60.0f;
static float cfgRuralJunctionDist = 250.0f; // m without junctions ahead: a 1+1 lane road is a rural road
static float cfgRoadSpeedBigMult = 0.85f;   // buses, trucks and slow vehicles
// Acceleration by road type (x the profile's)
static float cfgAccelHighway = 1.35f, cfgAccelFastRoad = 1.25f, cfgAccelRural = 1.15f, cfgAccelAvenue = 1.1f;
static float cfgAccelNarrow = 0.85f, cfgAccelDirt = 0.9f;
// Overtaking: faster and harder while passing
static float cfgFastRoadCornerGrip = 1.2f;  // x the profile's corner grip on freeways, fast and rural roads
static bool cfgGreenLightFullSpeed = true;  // through a green light at the road's speed (yellow/red still handled)
static bool cfgOvertakeBoost = true;
static float cfgOvertakeSpeedBoost = 50.0f; // km/h above what it wants (at least 40%)
static float cfgFullGapSpeed = 15.0f;       // m/s from which the profile's full time gap is kept
static float cfgMinGapFactor = 0.4f;        // fraction of the time gap left when crawling
static bool cfgCloseUpInQueues = true;      // creep up to the minimum gap behind stopped vehicles (queues at lights)
static bool cfgOvertakeFullThrottle = true; // the car's full acceleration while passing (else the two below)
static float cfgOvertakeAccelMult = 2.0f;   // x the profile's acceleration while passing...
static float cfgOvertakeMinAccel = 4.0f;    // ...and at least this (m/s2). Fast and aggressive drivers: full throttle

static bool playerAutoDrive = false;
////////////////////////

static CPed* playerPed = nullptr;
static int checkBumps = -1; // -1 = not checked yet
static unsigned int frameCounter = 0;
static int debugVehiclesProcessed = 0;
static int debugRacers = 0;
static std::fstream lg;

static std::string GetModuleDir() {
    HMODULE module = nullptr;
    char path[MAX_PATH] = {};
    GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        reinterpret_cast<LPCSTR>(&GetModuleDir), &module);
    GetModuleFileNameA(module, path, MAX_PATH);
    std::string dir = path;
    return dir.substr(0, dir.find_last_of("\\/") + 1);
}

static void GetEntityDimensions(CEntity* entity, CVector& outMin, CVector& outMax) {
    CColModel* colModel = entity->GetColModel();
    outMin = colModel->m_boundBox.m_vecMin;
    outMax = colModel->m_boundBox.m_vecMax;
}

static CVector GetWorldCoordWithOffset(CEntity* entity, const CVector& offset) {
    return CVector::Multiply3x3(*entity->m_matrix, offset) + entity->GetPosition();
}

static float FindGroundZ(const CVector& coord, bool* found) {
    return CWorld::FindGroundZFor3DCoord(coord.x, coord.y, coord.z, found, nullptr);
}

// Gap (m) from our front bumper to the closest part of another vehicle's body that is in front of us (within our width)
static float BodyGap(CVehicle* veh, CVehicle* other) {
    CVector mn, mx, omn, omx;
    GetEntityDimensions(veh, mn, mx);
    GetEntityDimensions(other, omn, omx);
    const CVector2D fwd = veh->GetForward().To2D(), right = veh->GetRight().To2D();
    const CVector2D front = veh->GetPosition().To2D() + fwd * mx.y;
    const CVector2D opos = other->GetPosition().To2D(), ofwd = other->GetForward().To2D(), oright = other->GetRight().To2D();
    float best = 1000.0f;
    // Corners and edge midpoints of the other body
    const float xs[3] = { omn.x, 0.0f, omx.x }, ys[3] = { omn.y, 0.0f, omx.y };
    for (float x : xs) {
        for (float y : ys) {
            if (x == 0.0f && y == 0.0f) continue;
            CVector2D p = opos + oright * ((x == 0.0f) ? (omn.x + omx.x) * 0.5f : x) + ofwd * y;
            float lat = CVector2D::Dot(p - front, right);
            float lon = CVector2D::Dot(p - front, fwd);
            if (std::abs(lat) < std::max(mx.x, -mn.x) + 0.3f && lon > -1.0f) best = std::min(best, lon);
        }
    }
    return best;
}

static bool IsBigOrSlow(CVehicle* veh) {
    return veh->bIsBig || veh->bIsBus || veh->m_pHandlingData->m_transmissionData.m_fMaxGearVelocity < 0.62f;
}

static bool IsEmergency(CVehicle* veh) {
    return veh->bIsLawEnforcer || veh->bIsAmbulanceOnDuty || veh->bIsFireTruckOnDuty || veh->IsLawEnforcementVehicle();
}

// Max speed the gearbox allows, m/s
static float GetVehicleTopSpeed(CVehicle* veh) {
    return veh->m_pHandlingData->m_transmissionData.m_fMaxGearVelocity * GAME_SPEED_TO_MPS;
}

static float Lerp(float a, float b, float t) { return a + (b - a) * t; }

static float WrapAngle(float a) {
    while (a > 3.1415927f) a -= 6.2831853f;
    while (a < -3.1415927f) a += 6.2831853f;
    return a;
}

// Picks the driver's profile by the weights of the ini, biased by what is being driven and by whom:
// buses, trucks and bicycles are never in a hurry, emergency vehicles (without siren) are driven safely,
// gangs drive harder and sports cars attract fast drivers
static eDriverProfile PickProfile(CVehicle* veh) {
    if (!cfgProfiles) return PROFILE_NORMAL;
    float weights[NUM_PROFILES];
    for (int i = 0; i < NUM_PROFILES; i++) weights[i] = static_cast<float>(std::max(profiles[i].chance, 0));

    bool bicycle = veh->m_nVehicleSubClass == VEHICLE_BMX;
    if (IsBigOrSlow(veh) || bicycle) {
        for (int i = PROFILE_FAST; i < NUM_PROFILES; i++) weights[i] = 0.0f;
    }
    else if (IsEmergency(veh)) {
        weights[PROFILE_CALM] = 0.0f;
        for (int i = PROFILE_FAST; i < NUM_PROFILES; i++) weights[i] *= 0.2f;
    }
    else {
        if (GetVehicleTopSpeed(veh) * MPS_TO_KMH > 190.0f) { // sports cars
            for (int i = PROFILE_FAST; i < NUM_PROFILES; i++) weights[i] *= 2.0f;
            weights[PROFILE_CALM] *= 0.5f;
        }
        if (veh->m_pDriver) {
            int pedType = veh->m_pDriver->m_nPedType;
            if (pedType >= PED_TYPE_GANG1 && pedType <= PED_TYPE_GANG10) {
                for (int i = PROFILE_FAST; i < NUM_PROFILES; i++) weights[i] *= 3.0f;
                weights[PROFILE_CALM] *= 0.3f;
            }
        }
    }
    float total = 0.0f;
    for (float w : weights) total += w;
    if (total <= 0.0f) return PROFILE_NORMAL;
    float r = CGeneral::GetRandomNumberInRange(0.0f, total);
    for (int i = 0; i < NUM_PROFILES; i++) {
        if (r < weights[i]) return static_cast<eDriverProfile>(i);
        r -= weights[i];
    }
    return PROFILE_NORMAL;
}

// A car the game just created standing (almost) still in a lane, with someone coming from behind too fast to stop for
// it (one appeared 8 m in front of a car at 110 km/h on the freeway): it starts already moving with the traffic
static void MatchSpawnSpeed(CVehicle* veh) {
    if (veh->m_nCreatedBy != RANDOM_VEHICLE) return;
    const float speed = veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
    if (speed > 3.0f) return;
    const CVector2D pos = veh->GetPosition().To2D(), fwd = veh->GetForward().To2D();
    float need = 0.0f;
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* other = CPools::ms_pVehiclePool->GetAt(i);
        if (!other || other == veh || std::abs(other->GetPosition().z - veh->GetPosition().z) > 4.0f) continue;
        CVector2D d = other->GetPosition().To2D() - pos;
        float lon = -CVector2D::Dot(d, fwd), lat = std::abs(CVector2D::Cross(fwd, d));
        if (lon < 3.0f || lon > 120.0f || lat > 4.0f) continue;                 // behind us, in our lane
        float otherSpeed = CVector2D::Dot(other->m_vecMoveSpeed.To2D(), fwd) * GAME_SPEED_TO_MPS;
        if (otherSpeed <= 5.0f) continue;
        // Can it stop comfortably (3 m/s2, 1 s to react) before us? A car appearing still in front of one at 50-70 km/h
        // was hit even when that one could just about stop
        if (otherSpeed * 1.0f + otherSpeed * otherSpeed / 6.0f + 15.0f > lon) need = std::max(need, otherSpeed * 0.9f);
    }
    if (need <= speed) return;
    // Not into something ahead
    CColPoint colPoint;
    CEntity* entity = nullptr;
    CWorld::pIgnoreEntity = veh;
    bool blocked = CWorld::ProcessLineOfSight(veh->GetPosition(), veh->GetPosition() + veh->GetForward() * (need * 1.5f + 10.0f),
        colPoint, entity, false, true, false, false, false, false, false, false);
    CWorld::pIgnoreEntity = nullptr;
    if (!blocked) {
        veh->m_vecMoveSpeed = veh->GetForward() * (need / GAME_SPEED_TO_MPS);
        spawnSpeedMatchCount++;
        return;
    }
    // It can't go with the traffic (something ahead of it): it shouldn't be there at all. Out of sight, the game didn't
    // really create it (it's removed next frame, outside the vehicle loop)
    if (!veh->GetIsOnScreen() && (veh->GetPosition() - TheCamera.GetPosition()).Magnitude() > 30.0f &&
        std::find(spawnRemovals.begin(), spawnRemovals.end(), veh) == spawnRemovals.end())
        spawnRemovals.push_back(veh);
}

// Lane data corrections (RealTrafficFix.routefixes.txt, made by tools\route_scan.py from a road scan): lanes towards node B
// on the link A -> B. The game's road network has lanes that end for one segment and come back, lanes counted where the
// road is narrower, etc: the AI had to change lanes where the road doesn't change. Applied (again) whenever the road data
// is loaded
static CPathNode* GetNode(const CNodeAddress& addr);
static CVector GetNodePos(const CPathNode* node);
static CNodeAddress GetAdjacentNode(const CPathNode* node, const CNodeAddress& nodeAddr, int i);
static int LanesToward(const CCarPathLink* link, const CNodeAddress& towards);
static CCarPathLink* GetLinkBetween(const CNodeAddress& a, const CPathNode* nodeA, const CNodeAddress& b);
// "A B lanes": lanes towards B on the link A->B. "A B shift dx dy": the link position moved (m), from where the game had it
struct RouteFix {
    CNodeAddress a, b;
    int lanes = -1;
    float dx = 0.0f, dy = 0.0f;
    bool shift = false;
    short origX = 0, origY = 0;   // the game's position (1/8 m), kept on the first apply
    bool haveOrig = false;
};
static std::vector<RouteFix> routeFixes;
static bool cfgRouteFixes = true;
static int routeFixesApplied = 0;

static void LoadRouteFixes(const std::string& file) {
    std::ifstream in(file);
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#' || line[0] == ';') continue;
        int ar, ai, br, bi, lanes;
        float dx, dy;
        RouteFix fix;
        if (sscanf_s(line.c_str(), "%d:%d %d:%d shift %f %f", &ar, &ai, &br, &bi, &dx, &dy) == 6) {
            fix.shift = true;
            fix.dx = std::clamp(dx, -8.0f, 8.0f);
            fix.dy = std::clamp(dy, -8.0f, 8.0f);
        }
        else if (sscanf_s(line.c_str(), "%d:%d %d:%d %d", &ar, &ai, &br, &bi, &lanes) == 5 && lanes >= 0 && lanes <= 7) fix.lanes = lanes;
        else continue;
        fix.a = CNodeAddress(static_cast<short>(ar), static_cast<short>(ai));
        fix.b = CNodeAddress(static_cast<short>(br), static_cast<short>(bi));
        routeFixes.push_back(fix);
    }
    lg << "Route fixes loaded: " << routeFixes.size() << "\n";
}

static void ApplyRouteFixes() {
    for (RouteFix& f : routeFixes) {
        CPathNode* nodeA = GetNode(f.a);
        if (!nodeA || !GetNode(f.b)) continue;
        CCarPathLink* link = GetLinkBetween(f.a, nodeA, f.b);
        if (!link) continue;
        if (f.shift) {
            // The game's own position the first time (and again if it was reloaded with it)
            short wantX = static_cast<short>(f.origX + std::lround(f.dx * 8.0f)), wantY = static_cast<short>(f.origY + std::lround(f.dy * 8.0f));
            if (!f.haveOrig || (link->m_vecPosn.x != wantX && link->m_vecPosn.x == f.origX)) {
                f.origX = link->m_vecPosn.x;
                f.origY = link->m_vecPosn.y;
                f.haveOrig = true;
            }
            wantX = static_cast<short>(f.origX + std::lround(f.dx * 8.0f));
            wantY = static_cast<short>(f.origY + std::lround(f.dy * 8.0f));
            if (link->m_vecPosn.x != wantX || link->m_vecPosn.y != wantY) {
                link->m_vecPosn.x = wantX;
                link->m_vecPosn.y = wantY;
                routeFixesApplied++;
            }
            continue;
        }
        if (link->m_address == f.b) {
            if (link->m_nNumLeftLanes != f.lanes) { link->m_nNumLeftLanes = f.lanes; routeFixesApplied++; }
        }
        else if (link->m_nNumRightLanes != f.lanes) { link->m_nNumRightLanes = f.lanes; routeFixesApplied++; }
    }
}

// Ambient traffic (see cfgAmbientSpawn). The road nodes of the 3x3 path areas around the player are cached every 2 s;
// a car is created on one of them in the ring around the player that the camera can't see, with nobody near it, in a
// random lane going a legal way, already moving; with a random driver and the model the game's population would pick.
// Then it's handed to the game as an ordinary random car (MARK_CAR_AS_NO_LONGER_NEEDED: counted, removed when far)
static std::vector<CNodeAddress> ambientNodes;
// The cars it created, kept as the mod's (mission) cars while they're near the player: the game removes random cars out
// of sight from 75 m (MixSets' VehDespawn settings weren't applied) and keeps only ~17 of them, so the ones created at
// 90-220 m were deleted right away. Far away, wrecked or without a driver they're handed back to the game as random cars
struct AmbientCar { int handle, driver; CVehicle* veh; };
static std::vector<AmbientCar> ambientCars;
// Where the created cars go (the first 60 that disappear are logged: age, distance, on screen, driver, speed)
struct AmbientTrack { int handle; CVehicle* veh; unsigned int born; CVector lastPos; bool lastOnScreen, lastDriver; float lastSpeed; };
static std::vector<AmbientTrack> ambientTracks;
static int ambientGoneLogged = 0;
static float cfgAmbientKeepDist = 0.0f;      // > 0: kept as the mod's cars within this (not needed with the despawn patch)
static float cfgDespawnOffScreen = 300.0f;     // m: random cars out of sight are removed beyond this (game: 45)
static float cfgDespawnOnScreen = 700.0f;      // x the camera generation multiplier (~0.46): ~320 m (game: 170 -> ~78 m); used out of sight too

static void ReleaseAmbientCar(const AmbientCar& a) {
    if (a.driver >= 0 && CPools::GetPed(a.driver)) Command<COMMAND_MARK_CHAR_AS_NO_LONGER_NEEDED>(a.driver);
    if (CPools::GetVehicle(a.handle) == a.veh) Command<COMMAND_MARK_CAR_AS_NO_LONGER_NEEDED>(a.handle);
}

static void UpdateAmbientCars(const CVector& center) {
    for (auto it = ambientCars.begin(); it != ambientCars.end();) {
        CVehicle* v = CPools::GetVehicle(it->handle);
        if (v != it->veh) { it = ambientCars.erase(it); continue; } // gone
        bool release = (v->GetPosition() - center).Magnitude2D() > cfgAmbientKeepDist || v->m_nStatus == STATUS_WRECKED ||
            !v->m_pDriver || v->m_fHealth < 250.0f;
        if (release) { ReleaseAmbientCar(*it); it = ambientCars.erase(it); }
        else ++it;
    }
}
static unsigned int ambientNodesTime = 0, ambientLastSpawn = 0;

// Nobody within 'radius', and nobody heading there that would arrive within 4 s (one created just past a junction was hit
// by a car turning out of it 2 s later)
static bool AmbientSpotFree(const CVector& pos, float radius) {
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!v) continue;
        CVector2D toSpot = (pos - v->GetPosition()).To2D();
        float dist = toSpot.Magnitude();
        if (dist < radius) return false;
        if (dist > 120.0f) continue;
        float closing = CVector2D::Dot(v->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS, toSpot * (1.0f / std::max(dist, 0.1f)));
        if (closing > 1.0f && dist / closing < 4.0f) return false;
    }
    return true;
}

// Room on that road: within 100 m of the spot along it, fewer cars than its lanes hold at one per 25 m, and no queue (3+
// cars stopped: one more car only makes it longer). One car waiting at a light kept whole roads empty
static bool AmbientRoadHasRoom(const CVector& pos, const CVector2D& dir, CCarPathLink* link) {
    const int lanes = std::max(link->m_nNumLeftLanes + link->m_nNumRightLanes, 1);
    int count = 0, stopped = 0;
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!v) continue;
        CVector2D rel = (v->GetPosition() - pos).To2D();
        float along = CVector2D::Dot(rel, dir), across = std::abs(dir.x * rel.y - dir.y * rel.x);
        if (std::abs(along) > 100.0f || across > lanes * LANE_WIDTH || std::abs(v->GetPosition().z - pos.z) > 6.0f) continue;
        if (v->m_pDriver && v->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS < 2.0f && ++stopped >= 3) return false;
        count++;
    }
    return count < lanes * 200 / 25;
}

static void UpdateAmbientSpawn() {
    if (!cfgAmbientSpawn) return;
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    CPlayerPed* player = FindPlayerPed();
    if (!player) return;
    if (CGame::currArea != 0 || CCutsceneMgr::ms_running) {
        for (const AmbientCar& a : ambientCars) ReleaseAmbientCar(a);
        ambientCars.clear();
        return;
    }
    UpdateAmbientCars(player->GetPosition());
    for (auto it = ambientTracks.begin(); it != ambientTracks.end();) {
        CVehicle* v = CPools::GetVehicle(it->handle);
        if (v == it->veh) {
            it->lastPos = v->GetPosition();
            it->lastOnScreen = v->GetIsOnScreen();
            it->lastDriver = v->m_pDriver != nullptr;
            it->lastSpeed = v->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
            if (now - it->born > 120000) { it = ambientTracks.erase(it); continue; } // lived 2 min: fine
            ++it;
            continue;
        }
        if (ambientGoneLogged < 60) {
            ambientGoneLogged++;
            lg << "Ambient car gone after " << (now - it->born) / 1000.0f << " s, " << (it->lastPos - player->GetPosition()).Magnitude()
               << " m from the player, on screen " << it->lastOnScreen << ", driver " << it->lastDriver << ", speed " << it->lastSpeed << "\n";
        }
        it = ambientTracks.erase(it);
    }
    if (now - ambientLastSpawn < static_cast<unsigned int>(cfgAmbientInterval)) return;
    // Its own target: this many cars on the roads within 250 m of the player (the game's own limit is set by its
    // population cycle, by time and zone, and stayed low whatever MixSets asked)
    {
        const CVector c = player->GetPosition();
        int nearCount = 0;
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
            CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
            if (v && v->m_pDriver && (v->GetPosition() - c).Magnitude2D() < 250.0f) nearCount++;
        }
        ambientNear = nearCount;
        if (nearCount >= cfgAmbientTarget) return;
    }
    if (CPools::ms_pVehiclePool->GetNoOfFreeSpaces() < 12) return;
    const CVector center = player->GetPosition();

    if (now - ambientNodesTime > 2000 || ambientNodes.empty()) {
        ambientNodesTime = now;
        ambientNodes.clear();
        float laneMetres = 0.0f; // lanes x length of the roads within 250 m (each link counted from both its ends: half)
        int ax = static_cast<int>((center.x + 3000.0f) / 750.0f), ay = static_cast<int>((center.y + 3000.0f) / 750.0f);
        for (int y = ay - 1; y <= ay + 1; y++) {
            for (int x = ax - 1; x <= ax + 1; x++) {
                if (x < 0 || y < 0 || x >= 8 || y >= 8) continue;
                int area = y * 8 + x;
                if (!ThePaths.m_pPathNodes[area]) continue;
                for (unsigned int i = 0; i < ThePaths.m_dwNumVehicleNodes[area]; i++) {
                    CNodeAddress addr(static_cast<short>(area), static_cast<short>(i));
                    CPathNode* node = GetNode(addr);
                    if (!node || node->SwitchedOff || node->WaterNode || node->InteriorNode) continue;
                    float dist = (GetNodePos(node) - center).Magnitude2D();
                    if (dist < 250.0f) {
                        for (int n = 0; n < node->NumberAdjNodes; n++) {
                            CNodeAddress nb = GetAdjacentNode(node, addr, n);
                            CPathNode* nbNode = GetNode(nb);
                            CCarPathLink* link = nbNode ? GetLinkBetween(addr, node, nb) : nullptr;
                            if (!link) continue;
                            int lanes = LanesToward(link, nb) + LanesToward(link, addr);
                            laneMetres += 0.5f * lanes * (GetNodePos(nbNode) - GetNodePos(node)).Magnitude2D();
                        }
                    }
                    if (node->NumberAdjNodes != 2) continue;
                    if (dist > cfgAmbientMinDist - 10.0f && dist < cfgAmbientMaxDist + 10.0f) ambientNodes.push_back(addr);
                }
            }
        }
        ambientCapacity = static_cast<int>(laneMetres / std::max(cfgAmbientLaneSpacing, 10.0f));
    }
    if (ambientNodes.empty()) return;
    // No more than the roads around hold (Bayside, a few one-lane roads, got 70 cars and locked up)
    if (ambientNear >= ambientCapacity) return;

    // A few tries at random spots this time
    for (int attempt = 0; attempt < 6; attempt++) {
        const CNodeAddress addr = ambientNodes[CGeneral::GetRandomNumberInRange(0, static_cast<int>(ambientNodes.size()))];
        CPathNode* node = GetNode(addr);
        if (!node) continue;
        CVector nodePos = GetNodePos(node);
        float dist = (nodePos - center).Magnitude2D();
        if (dist < cfgAmbientMinDist || dist > cfgAmbientMaxDist || std::abs(nodePos.z - center.z) > 40.0f) continue;
        // Which way: one of the two neighbours with lanes towards it
        int k = CGeneral::GetRandomNumberInRange(0, 2);
        CNodeAddress toAddr = GetAdjacentNode(node, addr, k);
        CPathNode* to = GetNode(toAddr);
        CCarPathLink* link = to ? GetLinkBetween(addr, node, toAddr) : nullptr;
        if (!link || LanesToward(link, toAddr) == 0) {
            toAddr = GetAdjacentNode(node, addr, 1 - k);
            to = GetNode(toAddr);
            link = to ? GetLinkBetween(addr, node, toAddr) : nullptr;
            if (!link || LanesToward(link, toAddr) == 0) continue;
        }
        int lanes = LanesToward(link, toAddr);
        CVector2D d = (GetNodePos(to) - nodePos).To2D();
        float len = d.Magnitude();
        if (len < 4.0f) continue;
        d *= 1.0f / len;
        const CVector2D r(d.y, -d.x);
        const CVector2D lp(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
        int lane = CGeneral::GetRandomNumberInRange(0, lanes);
        float along = CVector2D::Dot(nodePos.To2D() - lp, d) + std::min(4.0f, len * 0.3f);
        CVector2D p2 = lp + d * along + r * ((link->OneWayLaneOffset() + lane) * LANE_WIDTH);
        CVector pos(p2.x, p2.y, nodePos.z + 1.0f);
        // Out of sight, and nobody there
        if (TheCamera.IsSphereVisible(pos, 6.0f)) continue;
        if (!AmbientSpotFree(pos, 30.0f)) continue;
        if (!AmbientRoadHasRoom(pos, d, link)) continue;
        // Not right next to a junction (cars turn out of it onto the spot)
        if (GetNode(toAddr)->NumberAdjNodes > 2) continue;
        {
            CNodeAddress back = GetAdjacentNode(node, addr, GetAdjacentNode(node, addr, 0) == toAddr ? 1 : 0);
            CPathNode* backNode = GetNode(back);
            if (!backNode || backNode->NumberAdjNodes > 2) continue;
        }
        bool groundFound = false;
        float groundZ = FindGroundZ({ pos.x, pos.y, pos.z + 2.0f }, &groundFound);
        if (!groundFound || std::abs(groundZ - nodePos.z) > 3.0f) continue; // collision not loaded there yet
        // The model the game's population would pick here, loaded, a road vehicle
        int carClass = 0;
        int model = CCarCtrl::ChooseModel(&carClass);
        if (model <= 0 || CStreaming::ms_aInfoForModel[model].m_nLoadState != LOADSTATE_LOADED) return;
        CBaseModelInfo* mi = CModelInfo::GetModelInfo(model);
        if (!mi || mi->GetModelType() != MODEL_INFO_VEHICLE) return;
        int vtype = static_cast<CVehicleModelInfo*>(mi)->m_nVehicleType;
        if (vtype != VEHICLE_AUTOMOBILE && vtype != VEHICLE_BIKE && vtype != VEHICLE_QUAD) return;

        int handle = -1, driver = -1;
        Command<COMMAND_CREATE_CAR>(model, pos.x, pos.y, groundZ + 1.0f, &handle);
        CVehicle* veh = CPools::GetVehicle(handle);
        if (!veh) return;
        Command<COMMAND_SET_CAR_HEADING>(handle, CGeneral::GetATanOfXY(d.x, d.y) * 57.2957795f - 90.0f);
        Command<COMMAND_CREATE_RANDOM_CHAR_AS_DRIVER>(handle, &driver);
        // No driver (no ped model loaded for it): an empty car parked in the lane, cars ran into them. Not created then
        if (!veh->m_pDriver) {
            if (driver >= 0 && CPools::GetPed(driver)) Command<COMMAND_DELETE_CHAR>(driver);
            Command<COMMAND_DELETE_CAR>(handle);
            ambientLastSpawn = now;
            return;
        }
        Command<COMMAND_CAR_WANDER_RANDOMLY>(handle);
        Command<COMMAND_SET_CAR_CRUISE_SPEED>(handle, 20.0f);
        Command<COMMAND_SET_CAR_DRIVING_STYLE>(handle, 0);
        // Already moving with the traffic of that road
        float speed = node->Speed >= 2 ? 22.0f : (node->Speed == 1 ? 12.0f : 7.0f);
        veh->m_vecMoveSpeed = CVector(d.x, d.y, 0.0f) * (speed / GAME_SPEED_TO_MPS);
        if (cfgAmbientKeepDist > 0.0f) {
            vehExData.Get(veh).forceDrive = true; // a mission car while it's kept: the mod drives it anyway
            ambientCars.push_back({ handle, driver, veh });
        }
        else {
            // An ordinary random car right away (the game drives mission cars differently: U-turns, off the route)
            if (driver >= 0) Command<COMMAND_MARK_CHAR_AS_NO_LONGER_NEEDED>(driver);
            Command<COMMAND_MARK_CAR_AS_NO_LONGER_NEEDED>(handle);
        }
        ambientLastSpawn = now;
        ambientSpawnCount++;
        if (ambientTracks.size() < 200) ambientTracks.push_back({ handle, veh, now, veh->GetPosition(), false, true, speed });
        return;
    }
}

// Broken down (engine dead) and standing in the road for 20 s, out of sight: removed (a broken van on a freeway bridge
// held a queue that couldn't get round it). Random cars only
// Circles of waiting: stopped cars each waiting for the next and the last for the first (side by side squeezed at the
// Bayside junction, each side sensor on the other: both stopped for good). The one whose "leader" is the least ahead of
// it (beside or behind: it isn't really in the way) stops seeing it for 4 s and drives out
// Where a vehicle is across its road, in lanes of its own way (0 = its first lane, below -0.5: in the other way's lanes)
static float LanePosition(CVehicle* v) {
    const CAutoPilot& ap = v->m_autoPilot;
    CPathNode* curr = GetNode(ap.m_currentAddress);
    CPathNode* next = GetNode(ap.m_startingRouteNode);
    if (!curr || !next) return 0.0f;
    CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
    if (!link) return 0.0f;
    CVector2D d = (GetNodePos(next) - GetNodePos(curr)).To2D();
    float len = d.Magnitude();
    if (len < 0.5f) return 0.0f;
    d *= 1.0f / len;
    const CVector2D lp(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
    return CVector2D::Dot(v->GetPosition().To2D() - lp, CVector2D(d.y, -d.x)) / LANE_WIDTH - link->OneWayLaneOffset();
}

static void BreakDeadlocks() {
    if (frameCounter % 15 != 0) return;
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    auto waiting = [now](CVehicle* c, float minStill) -> ExtendedData* {
        if (!c || !CPools::ms_pVehiclePool->IsObjectValid(c)) return nullptr;
        ExtendedData& x = vehExData.Get(c);
        if (now - x.lastProcessed > 500 || x.stillTime < minStill || !x.leaderVeh || now < x.deadlockUntil) return nullptr;
        return &x;
    };
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!waiting(v, 4.0f)) continue;
        CVehicle* chain[8];
        int n = 0;
        bool cycle = false;
        for (CVehicle* c = v; n < 8;) {
            chain[n++] = c;
            ExtendedData* cx = waiting(c, 3.0f);
            if (!cx) break;
            c = cx->leaderVeh;
            if (c == v) { cycle = true; break; }
            if (std::find(chain, chain + n, c) != chain + n) break; // a circle without v: found from one of its members
        }
        if (!cycle || n < 2) continue;
        int victim = 0;
        float leastAhead = 1e9f;
        for (int k = 0; k < n; k++) {
            CVehicle* leader = vehExData.Get(chain[k]).leaderVeh;
            float ahead = CVector2D::Dot((leader->GetPosition() - chain[k]->GetPosition()).To2D(), chain[k]->GetForward().To2D());
            if (ahead < leastAhead) { leastAhead = ahead; victim = k; }
        }
        // Nose to nose: the one in the other way's lanes (it came the wrong way) is the one that moves, back to its own
        // lanes; the other stays where it is (going round it would take it into the oncoming lanes)
        if (n == 2 && CVector2D::Dot(chain[0]->GetForward().To2D(), chain[1]->GetForward().To2D()) < -0.5f)
            victim = LanePosition(chain[0]) < LanePosition(chain[1]) ? 0 : 1;
        ExtendedData& vx = vehExData.Get(chain[victim]);
        lg << "Deadlock: " << n << " cars waiting in a circle; veh " << CPools::GetVehicleRef(chain[victim]) << " stops waiting for veh "
            << CPools::GetVehicleRef(vx.leaderVeh) << " (" << leastAhead << " m ahead of it)\n";
        vx.deadlockIgnore = vx.leaderVeh;
        vx.deadlockUntil = now + 4000;
        // Nose to nose (one came the wrong way): the game brakes for the other car too, so it goes round it (the bypass)
        CVehicle* vv = chain[victim];
        const bool noseToNose = n == 2 && CVector2D::Dot(vv->GetForward().To2D(), vx.leaderVeh->GetForward().To2D()) < -0.5f;
        // First back out of it (2.5 s, the nose swinging away from the other one), if nobody is right behind: going round a
        // long truck turning in a junction never worked, the queue behind waited for minutes
        bool roomBehind = noseToNose && vx.noseReverses < 2;
        for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize && roomBehind; k++) {
            CVehicle* o = CPools::ms_pVehiclePool->GetAt(k);
            if (!o || o == vv || o == vx.leaderVeh) continue;
            CVector2D rel = (o->GetPosition() - vv->GetPosition()).To2D();
            float lon = CVector2D::Dot(rel, vv->GetForward().To2D());
            float lat = CVector2D::Cross(vv->GetForward().To2D(), rel);
            roomBehind = !(lon < 0.0f && lon > -12.0f && std::abs(lat) < 3.0f);
        }
        if (roomBehind) {
            CVector2D rel = (vx.leaderVeh->GetPosition() - vv->GetPosition()).To2D();
            bool otherOnLeft = CVector2D::Cross(vv->GetForward().To2D(), rel) > 0.0f;
            vv->m_autoPilot.m_nTempAction = otherOnLeft ? TEMPACT_REVERSE_LEFT : TEMPACT_REVERSE_RIGHT;
            vv->m_autoPilot.m_nTempActionTime = now + 2500;
            vx.noseReverses++;
            vx.deadlockUntil = now + 6000;
            lg << "  nose to nose: backing out\n";
        }
        else if (noseToNose && !vx.bypassUntil) {
            vx.savedStyle = static_cast<unsigned char>(vv->m_autoPilot.m_nCarDrivingStyle);
            vv->m_autoPilot.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
            vx.bypassUntil = now + 12000;
            vx.bypassBlocker = vx.leaderVeh;
            vx.deadlockUntil = now + 12000;
            lg << "  nose to nose: going round it\n";
        }
        vx.memObstacle = OBSTACLE_NONE;
        vx.memLeaderVeh = nullptr;
        deadlockBreakCount++;
    }
}

static std::unordered_map<CVehicle*, unsigned int> brokenSince;
static void RemoveBrokenDown() {
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    if (frameCounter % 30 != 0) return;
    for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
        CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
        if (!v || v->m_nCreatedBy != RANDOM_VEHICLE || (v->m_pDriver && v->m_pDriver->IsPlayer())) continue;
        bool broken = (v->m_fHealth < 250.0f || v->m_nStatus == STATUS_WRECKED) && v->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS < 0.5f;
        if (!broken) { brokenSince.erase(v); continue; }
        auto it = brokenSince.find(v);
        if (it == brokenSince.end()) { brokenSince[v] = now; continue; }
        if (now - it->second < 20000 || v->GetIsOnScreen() || (v->GetPosition() - TheCamera.GetPosition()).Magnitude() < 40.0f) continue;
        brokenSince.erase(it);
        spawnRemovals.push_back(v); // removed next frame with the bad spawns
        brokenRemovedCount++;
    }
    for (auto it = brokenSince.begin(); it != brokenSince.end();) {
        if (!CPools::ms_pVehiclePool->IsObjectValid(it->first)) it = brokenSince.erase(it);
        else ++it;
    }
}

// Cars created where they'd be hit (see MatchSpawnSpeed): removed the way the game removes distant cars
static void RemoveBadSpawns() {
    for (CVehicle* veh : spawnRemovals) {
        if (!CPools::ms_pVehiclePool->IsObjectValid(veh) || veh->m_nCreatedBy != RANDOM_VEHICLE || veh->GetIsOnScreen()) continue;
        if (veh->m_pDriver && veh->m_pDriver->IsPlayer()) continue;
        CCarCtrl::RemoveFromInterestingVehicleList(veh);
        CWorld::Remove(veh);
        delete veh;
        spawnRemovedCount++;
    }
    spawnRemovals.clear();
}

static void CountPassRefusal(ExtendedData& xdata, int why) {
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    if (now - xdata.lastPassRefusal < 3000) return;
    xdata.lastPassRefusal = now;
    passRefusals[why]++;
}

static void InitVehicle(ExtendedData& xdata, CVehicle* veh) {
    if (xdata.bInitialized || !veh->m_pDriver) return; // the driver decides the profile
    xdata.bInitialized = true;
    xdata.profile = PickProfile(veh);
    xdata.bRacer = profiles[xdata.profile].racingLogic;
    // Drivers of the same profile aren't all the same: +-6% speed, +-0.1 aggression
    float spread = std::clamp(cfgPersonalitySpread, 0.0f, 1.0f);
    xdata.speedVariation = 1.0f + CGeneral::GetRandomNumberInRange(-0.06f, 0.06f) * spread;
    xdata.aggression = std::clamp(profiles[xdata.profile].aggression + CGeneral::GetRandomNumberInRange(-0.1f, 0.1f) * spread, 0.0f, 1.0f);
    // Drivers above normal (fast, aggressive, fleeing, max performance) in ordinary cars: handbrake turns
    xdata.handbrakeCapable = cfgHandbrakeTurns && xdata.profile >= PROFILE_FAST && veh->m_nVehicleSubClass == VEHICLE_AUTOMOBILE &&
        !IsBigOrSlow(veh);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Adaptive limits: what each vehicle model really manages to do (lateral grip in g, braking in m/s2)

struct ModelLimits {
    float grip = 0.85f;
    float decel = 7.5f;
    int samples = 0;
};
static std::unordered_map<int, ModelLimits> learnedLimits;
static std::string learnedFile;
static bool learnedDirty = false;

static ModelLimits& GetModelLimits(CVehicle* veh) {
    auto it = learnedLimits.find(veh->m_nModelIndex);
    if (it != learnedLimits.end()) return it->second;
    ModelLimits limits;
    limits.grip = 0.85f * std::clamp(veh->m_pHandlingData->m_fTractionMultiplier, 0.5f, 1.5f);
    return learnedLimits[veh->m_nModelIndex] = limits;
}

static void LoadLearnedLimits() {
    std::ifstream file(learnedFile);
    int model, samples;
    float grip, decel;
    while (file >> model >> grip >> decel >> samples) {
        learnedLimits[model] = { std::clamp(grip, 0.4f, 1.6f), std::clamp(decel, 3.0f, 12.0f), samples };
    }
    lg << "Learned limits loaded for " << learnedLimits.size() << " models\n";
}

static void SaveLearnedLimits() {
    if (!learnedDirty) return;
    std::ofstream file(learnedFile, std::ios::trunc);
    for (auto& [model, limits] : learnedLimits) {
        if (limits.samples > 0) file << model << " " << limits.grip << " " << limits.decel << " " << limits.samples << "\n";
    }
    learnedDirty = false;
}

static float GetSlipAngle(CVehicle* veh) {
    CVector2D vel = veh->m_vecMoveSpeed.To2D();
    CVector2D fwd = veh->GetForward().To2D();
    float lv = vel.Magnitude(), lf = fwd.Magnitude();
    if (lv < 0.0001f || lf < 0.0001f) return 0.0f;
    return std::acos(std::clamp(CVector2D::Dot(vel, fwd) / (lv * lf), -1.0f, 1.0f));
}

static int GetWheelStates(CVehicle* veh, eWheelState* out) {
    switch (veh->m_nVehicleSubClass) {
    case VEHICLE_AUTOMOBILE: case VEHICLE_MTRUCK: case VEHICLE_QUAD:
        for (int i = 0; i < 4; i++) out[i] = static_cast<CAutomobile*>(veh)->m_wheelState[i];
        return 4;
    case VEHICLE_BIKE: case VEHICLE_BMX:
        for (int i = 0; i < 2; i++) out[i] = static_cast<CBike*>(veh)->m_wheelState[i];
        return 2;
    default:
        return 0;
    }
}

// Grip of what the tyres are on now compared to dry tarmac (game's surface.dat tyre grip), minus rain.
// outGround is the surface alone (no rain)
static float GetSurfaceGripFactor(CVehicle* veh, float& outGround) {
    outGround = 1.0f;
    SurfaceInfos_c& surfaces = *reinterpret_cast<SurfaceInfos_c*>(0xB79538); // g_surfaceInfos
    float tarmac = surfaces.GetTyreGrip(SURFACE_TARMAC);
    if (tarmac <= 0.0f) return 1.0f;

    const CColPoint* points = nullptr;
    int numWheels = 0;
    switch (veh->m_nVehicleSubClass) {
    case VEHICLE_AUTOMOBILE: case VEHICLE_MTRUCK: case VEHICLE_QUAD:
        points = static_cast<CAutomobile*>(veh)->m_wheelColPoint; numWheels = 4; break;
    case VEHICLE_BIKE: case VEHICLE_BMX:
        points = static_cast<CBike*>(veh)->m_anWheelColPoint; numWheels = 2; break;
    default: return 1.0f;
    }

    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < numWheels; i++) {
        unsigned char surface = points[i].m_nSurfaceTypeB;
        if (surface == SURFACE_DEFAULT) continue; // wheel in the air
        sum += std::clamp(surfaces.GetTyreGrip(surface) / tarmac, 0.3f, 1.2f);
        count++;
    }
    float factor = count ? sum / count : 1.0f;
    outGround = factor;
    return factor * (1.0f - cfgRainGripLoss * std::clamp(CWeather::WetRoads, 0.0f, 1.0f));
}

// Speed factor for a damaged vehicle (and grip factor for burst tyres): it limps along until it's removed
static float GetDamageFactor(CVehicle* veh, float& outGripFactor) {
    outGripFactor = 1.0f;
    bool burst = false;
    switch (veh->m_nVehicleSubClass) {
    case VEHICLE_AUTOMOBILE: case VEHICLE_MTRUCK: case VEHICLE_QUAD:
        for (int i = 0; i < 4; i++) burst |= static_cast<CAutomobile*>(veh)->m_damageManager.GetWheelStatus(i) != 0; // burst/missing
        break;
    case VEHICLE_BIKE: case VEHICLE_BMX:
        for (int i = 0; i < 2; i++) burst |= static_cast<CBike*>(veh)->m_nWheelStatus[i] != 0;
        break;
    }
    float factor = 1.0f;
    if (veh->m_fHealth < 250.0f) factor = 0.45f;      // burning
    else if (veh->m_fHealth < 390.0f) factor = 0.6f;  // heavy smoke
    else if (veh->m_fHealth < 650.0f) factor = 0.85f; // smoke
    if (burst) {
        factor = std::min(factor, 0.45f);
        outGripFactor = 0.6f;
    }
    return factor;
}

// Learns from every frame the vehicle is driven: sliding while cornering = grip limit found,
// cornering harder than the estimate without sliding = the estimate was too low. Same for braking
static void UpdateLearnedLimits(CVehicle* veh, ExtendedData& xdata, float speed, float dt) {
    // Real acceleration of the vehicle = change of its velocity vector (not yaw rate * speed: a spinning car
    // rotates without turning). Split into along the travel direction (braking) and across it (cornering)
    CVector2D vel = veh->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS;
    CVector2D prevVel = xdata.prevVel;
    bool consecutive = xdata.prevSampleFrame + 1 == frameCounter;
    xdata.prevVel = vel;
    xdata.prevSpeed = speed;
    xdata.prevSampleFrame = frameCounter;
    if (!cfgAdaptiveLimits || !consecutive || dt < 0.001f) { xdata.filtLat = xdata.filtLong = 0.0f; return; }

    float v = vel.Magnitude();
    if (v < 8.0f) return;
    CVector2D dir = vel * (1.0f / v);
    CVector2D acc = (vel - prevVel) * (1.0f / dt);
    if (acc.Magnitude() > 25.0f) { xdata.filtLat = xdata.filtLong = 0.0f; return; } // impact, not driving
    // Smooth out kerbs, bumps and physics jitter (~0.1s)
    xdata.filtLat += (std::abs(CVector2D::Cross(dir, acc)) - xdata.filtLat) * 0.25f;
    xdata.filtLong += (CVector2D::Dot(acc, dir) - xdata.filtLong) * 0.25f;

    bool isBike = veh->m_nVehicleSubClass == VEHICLE_BIKE || veh->m_nVehicleSubClass == VEHICLE_BMX;
    if (veh->GetNumContactWheels() < (isBike ? 2 : 4)) return;
    if (xdata.damageFactor < 1.0f) return; // damaged handling teaches nothing about the model

    // Stored as "dry tarmac, undamaged" values; what was measured on dirt/rain is scaled back up
    float normalize = 1.0f / std::max(xdata.surfaceFactor, 0.3f);
    ModelLimits& limits = GetModelLimits(veh);
    float latG = xdata.filtLat / GRAVITY * normalize;
    float decel = -xdata.filtLong * normalize;
    float slip = GetSlipAngle(veh);
    bool learned = false;

    if (latG > 0.3f) {
        if (slip > 0.15f) { limits.grip += (latG - limits.grip) * 0.05f; learned = true; }          // sliding: that's the limit
        else if (latG > limits.grip) { limits.grip += (latG - limits.grip) * 0.02f; learned = true; } // more grip than we thought
    }

    eWheelState wheels[4];
    int numWheels = GetWheelStates(veh, wheels);
    bool locked = false;
    for (int i = 0; i < numWheels; i++) locked |= wheels[i] == WHEEL_STATE_FIXED || wheels[i] == WHEEL_STATE_SKIDDING;
    if (veh->m_fBreakPedal > 0.5f && decel > 2.0f) {
        if (locked) { limits.decel += (decel - limits.decel) * 0.05f; learned = true; }
        else if (decel > limits.decel) { limits.decel += (decel - limits.decel) * 0.02f; learned = true; }
    }

    if (learned) {
        limits.grip = std::clamp(limits.grip, 0.4f, 1.6f);
        limits.decel = std::clamp(limits.decel, 3.0f, 12.0f);
        limits.samples++;
        learnedDirty = true;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Path nodes

static bool IsAddressEmpty(const CNodeAddress& addr) {
    return addr.Region == 0xFFFF || addr.Index == 0xFFFF;
}

static CPathNode* GetNode(const CNodeAddress& addr) {
    if (IsAddressEmpty(addr) || addr.Region >= NUM_PATH_MAP_AREAS + NUM_PATH_INTERIOR_AREAS) return nullptr;
    CPathNode* nodes = ThePaths.m_pPathNodes[addr.Region];
    if (!nodes || addr.Index >= ThePaths.m_dwNumNodes[addr.Region]) return nullptr; // area not streamed in
    return &nodes[addr.Index];
}

static CVector GetNodePos(const CPathNode* node) {
    return { node->CoorsX / 8.0f, node->CoorsY / 8.0f, node->CoorsZ / 8.0f };
}

static CNodeAddress GetAdjacentNode(const CPathNode* node, const CNodeAddress& nodeAddr, int i) {
    return ThePaths.m_pNodeLinks[nodeAddr.Region][static_cast<UInt16>(node->IndexAdjacentNodes) + i];
}

// Max speed (m/s) allowed now so the vehicle can still brake to a safe speed for every curve ahead.
// Walks the AI route: previous node -> current node -> next node and then keeps following the road
// while it has no junctions (with junctions we don't know which way the AI will go).
// Car path link (navi node) of the road segment a -> b. Null if none or not streamed in
static CCarPathLink* GetLinkBetween(const CNodeAddress& a, const CPathNode* nodeA, const CNodeAddress& b) {
    if (a.Region >= NUM_PATH_MAP_AREAS || !ThePaths.m_pNaviLinks[a.Region]) return nullptr;
    for (int k = 0; k < nodeA->NumberAdjNodes; k++) {
        unsigned int idx = static_cast<UInt16>(nodeA->IndexAdjacentNodes) + k;
        if (ThePaths.m_pNodeLinks[a.Region][idx] != b) continue;
        UInt16 raw = *reinterpret_cast<const UInt16*>(&ThePaths.m_pNaviLinks[a.Region][idx]);
        if (raw == 0xFFFF) return nullptr;
        unsigned int area = raw >> 10, id = raw & 0x3FF;
        if (!ThePaths.m_pNaviNodes[area] || id >= ThePaths.m_dwNumCarPathLinks[area]) return nullptr;
        return &ThePaths.m_pNaviNodes[area][id];
    }
    return nullptr;
}

static CCarPathLink* LinkFromAddress(const CCarPathLinkAddress& address) {
    UInt16 raw = *reinterpret_cast<const UInt16*>(&address);
    if (raw == 0xFFFF) return nullptr;
    unsigned int area = raw >> 10, id = raw & 0x3FF;
    if (area >= NUM_PATH_MAP_AREAS || !ThePaths.m_pNaviNodes[area] || id >= ThePaths.m_dwNumCarPathLinks[area]) return nullptr;
    return &ThePaths.m_pNaviNodes[area][id];
}

// Traffic light that a vehicle going towards node 'towards' sees on this link (same test as
// CTrafficLights::ShouldCarStopForLight @ 0x49D610)
static eLightColor GetLinkLightColor(const CCarPathLink* link, const CNodeAddress& towards) {
    if (!link->m_nTrafficLightState) return LIGHT_NONE;
    bool appliesToUs = link->m_bTrafficLightDirection ? (link->m_address == towards) : (link->m_address != towards);
    if (!appliesToUs) return LIGHT_NONE;
    unsigned char color = link->m_nTrafficLightState == 1 ?
        plugin::CallAndReturn<unsigned char, 0x49D2D0>() :  // CTrafficLights::LightForCars1
        plugin::CallAndReturn<unsigned char, 0x49D310>();   // CTrafficLights::LightForCars2
    return color <= LIGHT_RED ? static_cast<eLightColor>(color) : LIGHT_RED;
}

static bool DrivingStyleObeysLights(eCarDrivingStyle style) {
    return style == DRIVINGSTYLE_STOP_FOR_CARS || style == DRIVINGSTYLE_SLOW_DOWN_FOR_CARS || style == 5 || style == 6;
}

// Fast curves (long, wide, usually banked): taken with more of the grip, up to FastCurveGripBoost x for curves good for
// 110 km/h and more; curves under 60 km/h as before. Fast curves were braked far below what the car could do
static float cfgFastCurveGripBoost = 1.7f;
static float cfgHighwayGapFactor = 0.3f;  // following time gap on freeways/fast roads, x the profile's
static float CurveSpeedFor(float grip, float curvature) {
    float v = std::sqrt(grip * GRAVITY / curvature);
    float t = std::clamp((v - 60.0f / 3.6f) / ((110.0f - 60.0f) / 3.6f), 0.0f, 1.0f);
    return v * std::sqrt(1.0f + (cfgFastCurveGripBoost - 1.0f) * t);
}

// Max speed (m/s) through a corner of 'angle' radians whose road segments around it are lenIn and lenOut long
static float CornerSpeed(float angle, float lenIn, float lenOut, float grip, float& outHalfLen) {
    // Arc tangent to both segments at their midpoints (where the AI's car path links are)
    outHalfLen = 0.5f * std::min(lenIn, lenOut);
    if (angle < 0.0175f) return 1000.0f; // < 1 degree, straight
    float radius = outHalfLen / std::tan(angle * 0.5f);
    // No fast-curve boost here: sharp corners between two nodes are ramps and junction turns (a car took an off-ramp at
    // 115 km/h with the boost and went over the edge)
    return std::sqrt(grip * GRAVITY * radius);
}

static float TurnAngle(const CVector& a, const CVector& b, const CVector& c, float& lenIn, float& lenOut) {
    CVector2D in = (b - a).To2D();
    CVector2D out = (c - b).To2D();
    lenIn = in.Magnitude();
    lenOut = out.Magnitude();
    if (lenIn < 0.5f || lenOut < 0.5f) return 0.0f;
    return std::acos(std::clamp(CVector2D::Dot(in, out) / (lenIn * lenOut), -1.0f, 1.0f));
}

// Lanes a vehicle going towards node 'towards' has on this link (same rule as CCarCtrl::PickNextNodeRandomly)
static int LanesToward(const CCarPathLink* link, const CNodeAddress& towards) {
    return link->m_address == towards ? link->m_nNumLeftLanes : link->m_nNumRightLanes;
}

// Distance (m) along the road from the vehicle to the next junction ahead, following nodes with 2 links. Up to 'maxDist'
static float DistToJunctionAhead(CVehicle* veh, float maxDist) {
    const CAutoPilot& ap = veh->m_autoPilot;
    CNodeAddress prevAddr = ap.m_currentAddress, addr = ap.m_startingRouteNode;
    CPathNode* node = GetNode(addr);
    if (!node) return maxDist;
    float dist = (GetNodePos(node) - veh->GetPosition()).Magnitude2D();
    for (int n = 0; n < 40 && dist < maxDist; n++) {
        if (node->NumberAdjNodes != 2) return dist;
        CNodeAddress next = GetAdjacentNode(node, addr, 0);
        if (next == prevAddr) next = GetAdjacentNode(node, addr, 1);
        CPathNode* nextNode = GetNode(next);
        if (!nextNode) return maxDist; // not streamed in: far
        dist += (GetNodePos(nextNode) - GetNodePos(node)).Magnitude2D();
        prevAddr = addr; addr = next; node = nextNode;
    }
    return std::min(dist, maxDist);
}

// Type of the road segment current node -> next node of the AI route. The speed class of the path nodes (0 = slow road,
// 1 = normal, 2 = freeway; the game makes traffic 0.65x / 1x / 2.3x faster with it, CCarCtrl::FindSpeedMultiplierWithSpeedFromNodes
// @ 0x424130) tells freeways; then the number of lanes tells fast roads, avenues and streets, and the distance between
// junctions tells a rural road from a city street
static eRoadType GetRoadType(CVehicle* veh, float groundGrip, float& outDistToJunction) {
    const CAutoPilot& ap = veh->m_autoPilot;
    outDistToJunction = 1000.0f;
    CPathNode* curr = GetNode(ap.m_currentAddress);
    CPathNode* next = GetNode(ap.m_startingRouteNode);
    if (!curr || !next) return ROAD_UNKNOWN;
    if (groundGrip < 0.85f) return ROAD_DIRT; // tyres on dirt/grass/sand
    if (curr->InteriorNode || next->InteriorNode) return ROAD_NARROW;

    // Lower class of both ends, so we're already slower when the segment leads to a slower road
    int nodeClass = std::min(curr->Speed, next->Speed);
    CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
    int lanes = link ? LanesToward(link, ap.m_startingRouteNode) : 1;
    int oncoming = link ? (link->m_address == ap.m_startingRouteNode ? link->m_nNumRightLanes : link->m_nNumLeftLanes) : 1;

    if (nodeClass >= 2) return ROAD_HIGHWAY;
    if (lanes + oncoming <= 1) return ROAD_NARROW; // single lane: alleys, ramps, service roads
    if (nodeClass == 0) return lanes >= 2 ? ROAD_STREET : ROAD_NARROW;
    if (lanes >= 3) return ROAD_FAST;
    if (lanes >= 2) return ROAD_AVENUE;
    outDistToJunction = DistToJunctionAhead(veh, cfgRuralJunctionDist);
    return outDistToJunction >= cfgRuralJunctionDist ? ROAD_RURAL : ROAD_STREET;
}

// How much harder than on a street drivers accelerate on this type of road
static float RoadAccelFactor(eRoadType type) {
    switch (type) {
    case ROAD_HIGHWAY: return cfgAccelHighway;
    case ROAD_FAST: return cfgAccelFastRoad;
    case ROAD_RURAL: return cfgAccelRural;
    case ROAD_AVENUE: return cfgAccelAvenue;
    case ROAD_NARROW: return cfgAccelNarrow;
    case ROAD_DIRT: return cfgAccelDirt;
    default: return 1.0f;
    }
}

// Speed (m/s) a normal driver keeps on this type of road
static float GetRoadSpeed(eRoadType type) {
    switch (type) {
    case ROAD_DIRT: return cfgRoadSpeedDirt / MPS_TO_KMH;
    case ROAD_NARROW: return cfgRoadSpeedNarrow / MPS_TO_KMH;
    case ROAD_STREET: return cfgRoadSpeedStreet / MPS_TO_KMH;
    case ROAD_RURAL: return cfgRoadSpeedRural / MPS_TO_KMH;
    case ROAD_AVENUE: return cfgRoadSpeedAvenue / MPS_TO_KMH;
    case ROAD_FAST: return cfgRoadSpeedFastRoad / MPS_TO_KMH;
    case ROAD_HIGHWAY: return cfgRoadSpeedHighway / MPS_TO_KMH;
    default: return 1000.0f;
    }
}

static int routesThisFrame = 0;     // route searches are spread over frames
static int routeSearchCount = 0, routeFailCount = 0, offRouteCount = 0;
// Why vehicles leave their route (AutoTest): no exit planned for that junction, the game took another exit than the
// planned one, the node wasn't a junction (the route didn't match the road)
static int offRouteNoPlan = 0, offRouteRejected = 0, offRouteNotJunction = 0, seqPlanApplied = 0, seqNoPlan = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Lane change board: how the drivers talk to each other. Before signalling, a driver asks for the piece of road its lane
// change will use (from a bit behind it to where it'll be when it's done, across its lane and the target one). It's
// granted only if no other granted request overlaps it: two cars never go for the same space, and when one has already
// started passing a truck the one ahead of it waits until it has gone by. Held while signalling and changing

struct LaneReservation {
    CVehicle* veh;
    CVector2D center, axis;    // oriented rectangle on the road
    float halfLen, halfWid, z;
    unsigned int expires;
    float priority = 0.0f;     // who needs that space more (see LanePriority)
    bool committed = false;    // already moving over: nobody takes it away
};
static std::vector<LaneReservation> laneBoard;
static int boardGranted = 0, boardDenied = 0, boardDeniedAttempts = 0; // denied: every request; attempts: once per wait
static int boardPreempted = 0;  // a signalling driver gave its space to one that needed it more
static std::vector<CVehicle*> boardEvicted; // told to give up their signalled lane change (read by UpdateLanes)

static bool ReservationsOverlap(const LaneReservation& a, const LaneReservation& b) {
    if (std::abs(a.z - b.z) > 4.0f) return false; // another level
    const CVector2D axes[4] = { a.axis, CVector2D(-a.axis.y, a.axis.x), b.axis, CVector2D(-b.axis.y, b.axis.x) };
    const CVector2D d = b.center - a.center;
    for (const CVector2D& ax : axes) {
        float ra = a.halfLen * std::abs(CVector2D::Dot(a.axis, ax)) + a.halfWid * std::abs(CVector2D::Dot(CVector2D(-a.axis.y, a.axis.x), ax));
        float rb = b.halfLen * std::abs(CVector2D::Dot(b.axis, ax)) + b.halfWid * std::abs(CVector2D::Dot(CVector2D(-b.axis.y, b.axis.x), ax));
        if (std::abs(CVector2D::Dot(d, ax)) > ra + rb) return false;
    }
    return true;
}

static void PruneLaneBoard() {
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    laneBoard.erase(std::remove_if(laneBoard.begin(), laneBoard.end(), [&](const LaneReservation& r) {
        return now >= r.expires || !CPools::ms_pVehiclePool->IsObjectValid(r.veh);
    }), laneBoard.end());
}

// Two drivers want the same space: the one that needs it more gets it. A driver only signalling (not moving over yet)
// with a clearly lower priority gives it up; one already moving over keeps it
static bool RequestLaneSpace(const LaneReservation& request) {
    PruneLaneBoard();
    for (const LaneReservation& r : laneBoard) {
        if (r.veh == request.veh || !ReservationsOverlap(r, request)) continue;
        if (r.committed || request.priority < r.priority + 1.0f) { boardDenied++; return false; }
    }
    // Ours: the lower-priority ones in the way give up
    for (auto it = laneBoard.begin(); it != laneBoard.end();) {
        if (it->veh != request.veh && ReservationsOverlap(*it, request)) {
            boardEvicted.push_back(it->veh);
            boardPreempted++;
            it = laneBoard.erase(it);
        }
        else ++it;
    }
    laneBoard.erase(std::remove_if(laneBoard.begin(), laneBoard.end(), [&](const LaneReservation& r) { return r.veh == request.veh; }),
        laneBoard.end());
    laneBoard.push_back(request);
    boardGranted++;
    return true;
}

static void HoldLaneSpace(CVehicle* veh, unsigned int until) {
    for (LaneReservation& r : laneBoard) if (r.veh == veh) r.expires = until;
}

static void CommitLaneSpace(CVehicle* veh) {
    for (LaneReservation& r : laneBoard) if (r.veh == veh) r.committed = true;
}

static bool TakeBoardEviction(CVehicle* veh) {
    auto it = std::find(boardEvicted.begin(), boardEvicted.end(), veh);
    if (it == boardEvicted.end()) return false;
    boardEvicted.erase(it);
    return true;
}

static void ReleaseLaneSpace(CVehicle* veh) {
    laneBoard.erase(std::remove_if(laneBoard.begin(), laneBoard.end(), [&](const LaneReservation& r) { return r.veh == veh; }),
        laneBoard.end());
}

// Roads the game itself leaves unused (switched off in the map data: dirt roads, country tracks) count as roads too;
// roads a mission/the story switched off (bridges to locked islands, roadblocks) don't
static bool cfgUseUnusedRoads = true;
static bool IsUsableCarNode(const CNodeAddress& addr, CPathNode* node) {
    return node && addr.Region < NUM_PATH_MAP_AREAS && addr.Index < ThePaths.m_dwNumVehicleNodes[addr.Region] &&
        (!node->SwitchedOff || (cfgUseUnusedRoads && node->SwitchedOffOriginal)) && !node->WaterNode && !node->InteriorNode;
}

// Speed (m/s) a route "thinks" a road segment has. Narrow roads are made look slower than they are (single lane each
// way x2, slow roads x3), so routes stay on the roads with 2+ lanes and leave them only when they must
static float RouteEdgeSpeed(const CPathNode* a, const CPathNode* b, int lanes) {
    int cls = std::min(a->Speed, b->Speed);
    if (cls >= 2) return 110.0f / MPS_TO_KMH;
    if (cls == 0) return 10.0f / MPS_TO_KMH;
    return (lanes >= 3 ? 90.0f : (lanes == 2 ? 70.0f : 25.0f)) / MPS_TO_KMH;
}

// A node on a road with 2+ lanes in one direction (freeways, avenues)
static bool IsWideRoadNode(const CNodeAddress& addr, CPathNode* node) {
    for (int k = 0; k < node->NumberAdjNodes; k++) {
        CNodeAddress other = GetAdjacentNode(node, addr, k);
        CCarPathLink* link = GetLinkBetween(addr, node, other);
        if (link && (link->m_nNumLeftLanes >= 2 || link->m_nNumRightLanes >= 2)) return true;
    }
    return false;
}

// A random destination among the road nodes loaded (the game only has the roads around the player): most drivers
// (DestinationWideRoads %) go somewhere on a road with 2+ lanes 1.2-2.5 km away, preferably a freeway; the others
// anywhere 0.8-2 km away (so there's traffic leaving the big roads too)
static void PickDestination(ExtendedData& xdata, CVehicle* veh) {
    const CVector pos = veh->GetPosition();
    CNodeAddress best;
    float bestScore = -1.0f;
    int loaded[NUM_PATH_MAP_AREAS], numLoaded = 0;
    for (int a = 0; a < NUM_PATH_MAP_AREAS; a++) if (ThePaths.m_pPathNodes[a] && ThePaths.m_dwNumVehicleNodes[a] > 0) loaded[numLoaded++] = a;
    const bool wide = static_cast<int>(CGeneral::GetRandomNumberInRange(0, 100)) < cfgDestinationWideRoads;
    const float minDist = wide ? 1200.0f : 800.0f, maxDist = wide ? 2500.0f : 2000.0f;
    for (int t = 0; t < 120 && numLoaded > 0; t++) {
        int area = loaded[CGeneral::GetRandomNumberInRange(0, numLoaded)];
        CNodeAddress addr(static_cast<short>(area), static_cast<short>(CGeneral::GetRandomNumberInRange(0, static_cast<int>(ThePaths.m_dwNumVehicleNodes[area]))));
        CPathNode* node = GetNode(addr);
        if (!IsUsableCarNode(addr, node) || node->NumberAdjNodes < 2) continue;
        bool isWide = node->Speed >= 2 || IsWideRoadNode(addr, node);
        float dist = (GetNodePos(node) - pos).Magnitude2D();
        float score = std::min(dist, maxDist) / maxDist;
        if (dist >= minDist && dist <= maxDist) score += 2.0f;
        if (wide && isWide) score += node->Speed >= 2 ? 4.0f : 3.0f;
        // Ahead of it: on one-way roads (freeways) a destination behind is far or unreachable
        CVector2D dir = (GetNodePos(node) - pos).To2D();
        if (dist > 1.0f && CVector2D::Dot(dir * (1.0f / dist), veh->GetForward().To2D()) > 0.3f) score += 1.5f;
        score += CGeneral::GetRandomNumberInRange(0.0f, 0.5f);
        if (score > bestScore) { bestScore = score; best = addr; }
    }
    xdata.hasDestination = !best.IsEmpty();
    xdata.destNode = best;
    if (CPathNode* node = GetNode(best)) xdata.destination = GetNodePos(node).To2D();
    xdata.destinationTime = CTimer::m_snTimeInMilliseconds;
    xdata.route.clear();
    xdata.routeComplete = false;
    xdata.planCount = 0; // plans were made for the old destination
}

// A* from the node the vehicle is heading to, to its destination. Cost = travel time (by road type) + turns
// (left 8 s, right 3 s, no U-turns), only legal directions. When the destination isn't reachable through the loaded
// roads, the route goes to the explored node closest to it, and is calculated again when it gets there
static bool CalculateRoute(CVehicle* veh, ExtendedData& xdata) {
    const CAutoPilot& ap = veh->m_autoPilot;
    xdata.route.clear();
    xdata.routePos = 0;
    xdata.routeComplete = false;
    xdata.routeTime = CTimer::m_snTimeInMilliseconds;
    // Off the route (the game took another exit): the roads of the old route are cheaper, so the new one goes back to
    // it unless another way is clearly better (it used to plan from scratch: a fast driver ended up crossing Las Venturas)
    std::unordered_set<uint64_t> oldEdges;
    for (size_t i = 0; i + 1 < xdata.oldRoute.size(); i++) {
        const CNodeAddress& a = xdata.oldRoute[i];
        const CNodeAddress& b = xdata.oldRoute[i + 1];
        oldEdges.insert((static_cast<uint64_t>((a.Region << 16) | a.Index) << 32) | static_cast<uint32_t>((b.Region << 16) | b.Index));
    }
    xdata.oldRoute.clear();
    CPathNode* fromNode = GetNode(ap.m_currentAddress);
    CPathNode* startNode = GetNode(ap.m_startingRouteNode);
    CPathNode* goalNode = GetNode(xdata.destNode);
    if (!fromNode || !startNode || (!goalNode && !xdata.fixedDestination)) return false;
    routeSearchCount++;
    // A far destination (roads not loaded there yet): the route goes as close to it as the loaded roads allow
    const CVector goalPos = goalNode ? GetNodePos(goalNode) : CVector(xdata.destination.x, xdata.destination.y, 0.0f);
    auto key = [](const CNodeAddress& a) { return (static_cast<uint32_t>(a.Region) << 16) | a.Index; };
    auto addrOf = [](uint32_t k) { return CNodeAddress(static_cast<short>(k >> 16), static_cast<short>(k & 0xFFFF)); };
    struct Rec { float g; uint32_t parent; bool closed; };
    std::unordered_map<uint32_t, Rec> recs;
    recs.reserve(16384);
    using Item = std::pair<float, uint32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    const float heurSpeed = 110.0f / MPS_TO_KMH;

    const uint32_t fromKey = key(ap.m_currentAddress), startKey = key(ap.m_startingRouteNode);
    const uint32_t goalKey = goalNode ? key(xdata.destNode) : 0xFFFFFFFE;
    recs[fromKey] = { 0.0f, 0xFFFFFFFF, true };
    recs[startKey] = { 0.0f, fromKey, false };
    open.push({ (GetNodePos(startNode) - goalPos).Magnitude2D() / heurSpeed, startKey });
    uint32_t bestKey = startKey;
    float bestH = 1e9f;
    int expansions = 0;
    bool found = false;
    uint32_t goalHit = goalKey;
    while (!open.empty() && expansions < 80000) {
        uint32_t k = open.top().second;
        open.pop();
        Rec& rec = recs[k];
        if (rec.closed) continue;
        rec.closed = true;
        expansions++;
        CNodeAddress addr = addrOf(k);
        CPathNode* node = GetNode(addr);
        if (!node) continue;
        CVector nodePos = GetNodePos(node);
        float h = (nodePos - goalPos).Magnitude2D();
        if (h < bestH) { bestH = h; bestKey = k; }
        if (k == goalKey || (xdata.goalRadius > 0.0f && h < xdata.goalRadius && (!xdata.goalFreeway || node->Speed >= 2))) {
            found = true;
            goalHit = k;
            break;
        }
        CPathNode* parent = rec.parent != 0xFFFFFFFF ? GetNode(addrOf(rec.parent)) : nullptr;
        float g = rec.g;
        uint32_t parentKey = rec.parent;
        for (int n = 0; n < node->NumberAdjNodes; n++) {
            CNodeAddress nb = GetAdjacentNode(node, addr, n);
            uint32_t nk = key(nb);
            if (nk == parentKey) continue; // no U-turns
            CPathNode* nbNode = GetNode(nb);
            if (!IsUsableCarNode(nb, nbNode) || (nbNode->NumberAdjNodes <= 1 && nk != goalKey)) continue;
            // "Don't wander here" roads too (the game's random wandering avoids them, the route's exit is forced: see
            // ForceRouteExit). Not into nodes with a special function (parking, valet, nightclub, deliveries, drive-through)
            if (!cfgUseUnusedRoads && nbNode->DontWanderHere && !node->DontWanderHere && nk != goalKey) continue;
            if (nbNode->SpecialFunction != 0 && nk != goalKey) continue;
            CCarPathLink* link = GetLinkBetween(addr, node, nb);
            int lanes = link ? LanesToward(link, nb) : 0;
            if (lanes <= 0) continue; // wrong way
            CVector nbPos = GetNodePos(nbNode);
            float len = (nbPos - nodePos).Magnitude2D();
            float cost = len / RouteEdgeSpeed(node, nbNode, lanes);
            if (!oldEdges.empty() && oldEdges.count((static_cast<uint64_t>(k) << 32) | nk)) cost *= cfgRejoinRouteFactor;
            if (parent && node->NumberAdjNodes > 2) {
                CVector2D in = (nodePos - GetNodePos(parent)).To2D(), out = (nbPos - nodePos).To2D();
                float turn = WrapAngle(CGeneral::GetATanOfXY(out.x, out.y) - CGeneral::GetATanOfXY(in.x, in.y));
                if (std::abs(turn) > 2.6f) continue;
                if (turn > 0.5f) cost += 8.0f;       // left turn: crosses the oncoming traffic
                else if (turn < -0.5f) cost += 3.0f; // right turn
            }
            auto it = recs.find(nk);
            if (it != recs.end() && (it->second.closed || it->second.g <= g + cost)) continue;
            recs[nk] = { g + cost, k, false };
            open.push({ g + cost + (nbPos - goalPos).Magnitude2D() / heurSpeed, nk });
        }
    }
    uint32_t end = found ? goalHit : bestKey;
    if (end == startKey && !found) { routeFailCount++; return false; }
    for (uint32_t k = end; k != 0xFFFFFFFF; k = recs[k].parent) {
        xdata.route.push_back(addrOf(k));
        if (xdata.route.size() > 6000) { xdata.route.clear(); routeFailCount++; return false; } // broken chain
    }
    std::reverse(xdata.route.begin(), xdata.route.end());
    xdata.routeComplete = found;
    xdata.planCount = 0;
    return true;
}

// Keeps track of where the vehicle is on its route; a new route when it left it, or when it got to the end of it
// A destination given from outside (AutoTest): its road node is found once the roads there are loaded
static void ResolveFixedDestination(ExtendedData& xdata) {
    if (!xdata.fixedDestination || GetNode(xdata.destNode)) return;
    float best = 400.0f; // the nearest road within 400 m
    for (int a = 0; a < NUM_PATH_MAP_AREAS; a++) {
        if (!ThePaths.m_pPathNodes[a]) continue;
        for (unsigned int i = 0; i < ThePaths.m_dwNumVehicleNodes[a]; i++) {
            CNodeAddress addr(static_cast<short>(a), static_cast<short>(i));
            CPathNode* node = GetNode(addr);
            if (!IsUsableCarNode(addr, node) || node->NumberAdjNodes < 2) continue;
            float d = (GetNodePos(node).To2D() - xdata.destination).Magnitude();
            if (d < best) { best = d; xdata.destNode = addr; }
        }
    }
}

// radius > 0: a waypoint, passed through anywhere within radius (its nearest node can be on the other carriageway: the
// route went round the city to reach it from the other side); freeway: through a freeway node there
void SetFixedDestination(CVehicle* veh, const CVector2D& destination, float radius = 0.0f, bool freeway = false) {
    ExtendedData& xdata = vehExData.Get(veh);
    xdata.goalRadius = radius;
    xdata.goalFreeway = freeway;
    xdata.destination = destination;
    xdata.destNode.Clear();
    xdata.hasDestination = true;
    xdata.fixedDestination = true;
    xdata.destinationTime = CTimer::m_snTimeInMilliseconds;
    xdata.route.clear();
    xdata.routeComplete = false;
    xdata.routeTime = 0;
    xdata.planCount = 0;
}

static void UpdateDestination(ExtendedData& xdata, CVehicle* veh) {
    if (!cfgDestinations) return;
    const CAutoPilot& ap = veh->m_autoPilot;
    const unsigned int now = CTimer::m_snTimeInMilliseconds;
    bool arrived = xdata.hasDestination && (xdata.destination - veh->GetPosition().To2D()).Magnitude() < 60.0f;
    if (xdata.fixedDestination) arrived = false; // whoever set it decides what happens there
    if (!xdata.hasDestination || arrived || (!xdata.fixedDestination && now - xdata.destinationTime > 600000)) PickDestination(xdata, veh);
    if (!xdata.hasDestination) return;

    bool needRoute = xdata.route.empty();
    if (!needRoute) {
        // Where are we: current node and next node of the vehicle, a few steps from the last known place
        int found = -1;
        int last = static_cast<int>(xdata.route.size()) - 1;
        for (int i = std::max(xdata.routePos - 1, 0); i < last && i <= xdata.routePos + 6; i++) {
            if (xdata.route[i] == ap.m_currentAddress && xdata.route[i + 1] == ap.m_startingRouteNode) { found = i; break; }
        }
        // Fast cars pass more than 6 nodes between two looks: on the route further on is still on the route (it was taken
        // for leaving it: the route was dropped and the car drove without one until the next was planned)
        for (int i = xdata.routePos + 7; found < 0 && i < last; i++) {
            if (xdata.route[i] == ap.m_currentAddress && xdata.route[i + 1] == ap.m_startingRouteNode) found = i;
        }
        if (found >= 0) xdata.routePos = found;
        else if (!(xdata.route[xdata.routePos] == ap.m_currentAddress)) { // left the route
            needRoute = true;
            CPathNode* junction = GetNode(ap.m_currentAddress);
            int planIdx = -1;
            for (int p = 0; p < xdata.planCount; p++) {
                if (xdata.plans[p].junction == ap.m_currentAddress && xdata.plans[p].from == ap.field_8) planIdx = p;
            }
            const char* why;
            offRouteCount++;
            if (!junction || junction->NumberAdjNodes <= 2) { offRouteNotJunction++; why = "not a junction"; }
            else if (planIdx < 0) { offRouteNoPlan++; why = "no plan"; }
            else {
                offRouteRejected++;
                why = "game took another exit";
            }
            if (offRouteCount <= 25) {
                bool onRoute = false;
                for (auto& a : xdata.route) onRoute |= a == ap.m_currentAddress;
                lg << "Off route (" << why << "): veh " << CPools::GetVehicleRef(veh) << " routePos " << xdata.routePos << "/"
                    << xdata.route.size() << " current " << ap.m_currentAddress.Region << ":" << ap.m_currentAddress.Index
                    << " expected " << xdata.route[xdata.routePos].Region << ":" << xdata.route[xdata.routePos].Index
                    << " adj " << (junction ? (int)junction->NumberAdjNodes : -1) << " currentOnRoute " << onRoute
                    << " | prev " << ap.field_8.Region << ":" << ap.field_8.Index << " next " << ap.m_startingRouteNode.Region << ":"
                    << ap.m_startingRouteNode.Index << " | route";
                for (int i = std::max(xdata.routePos - 3, 0); i < static_cast<int>(xdata.route.size()) && i <= xdata.routePos + 3; i++)
                    lg << (i == xdata.routePos ? " [" : " ") << xdata.route[i].Region << ":" << xdata.route[i].Index << (i == xdata.routePos ? "]" : "");
                lg << std::endl;
            }
            xdata.oldRoute.assign(xdata.route.begin() + std::min(xdata.routePos, static_cast<int>(xdata.route.size())), xdata.route.end());
            xdata.route.clear();
            xdata.routePos = 0;
            rejoinRouteCount++;
        }
        if (!xdata.route.empty() && xdata.routePos >= static_cast<int>(xdata.route.size()) - 3) {
            if (xdata.routeComplete && !xdata.fixedDestination) { PickDestination(xdata, veh); }
            needRoute = true;
        }
    }
    if (needRoute && now - xdata.routeTime > 300 && routesThisFrame < 4) {
        routesThisFrame++;
        ResolveFixedDestination(xdata);
        if (!CalculateRoute(veh, xdata) && !xdata.fixedDestination) PickDestination(xdata, veh); // somewhere it can get to, next time
    }
}

// Drivers decide at which exit they'll leave each junction ahead, long before the game asks (see InitSequenceHook).
// Among the legal exits (no dead ends, no wrong way, no U-turns) they take the one that heads to their destination,
// preferring freeways when it's far. Racing profiles only consider the exits that can be taken (nearly) at full speed
static bool GetPlannedExit(ExtendedData& xdata, const CNodeAddress& junctionAddr, CPathNode* junction, const CNodeAddress& from,
    const CVector& fromPos, const CVector& junctionPos, float grip, float desired, CNodeAddress& outExit)
{
    for (int p = 0; p < xdata.planCount; p++) {
        if (xdata.plans[p].junction == junctionAddr && xdata.plans[p].from == from) {
            outExit = xdata.plans[p].exit;
            return true;
        }
    }
    auto addPlan = [&](const CNodeAddress& exit) {
        if (xdata.planCount == MAX_ROUTE_PLANS) {
            for (int p = 1; p < MAX_ROUTE_PLANS; p++) xdata.plans[p - 1] = xdata.plans[p];
            xdata.planCount--;
        }
        xdata.plans[xdata.planCount++] = { junctionAddr, from, exit };
        outExit = exit;
    };
    // On the route: its next node
    if (cfgDestinations) {
        int last = static_cast<int>(xdata.route.size()) - 1;
        for (int i = std::max(xdata.routePos, 1); i < last; i++) {
            if (xdata.route[i] == junctionAddr && xdata.route[i - 1] == from) { addPlan(xdata.route[i + 1]); return true; }
        }
    }

    CNodeAddress options[16];
    float optionSpeed[16];
    CVector optionPos[16];
    int optionClass[16];
    int numOptions = 0;
    float best = 0.0f;
    for (int k = 0; k < junction->NumberAdjNodes && numOptions < 16; k++) {
        CNodeAddress exitAddr = GetAdjacentNode(junction, junctionAddr, k);
        if (exitAddr == from) continue;
        CPathNode* exitNode = GetNode(exitAddr);
        if (!exitNode || exitNode->SwitchedOff || exitNode->NumberAdjNodes <= 1) continue;
        if (exitNode->DontWanderHere && !junction->DontWanderHere) continue;
        if (exitNode->SpecialFunction != 0) continue; // parking, drive-through...: the game won't take it
        CCarPathLink* link = GetLinkBetween(junctionAddr, junction, exitAddr);
        if (!link || LanesToward(link, exitAddr) == 0) continue; // wrong way on a one-way road

        float lenIn, lenOut, halfLen;
        float angle = TurnAngle(fromPos, junctionPos, GetNodePos(exitNode), lenIn, lenOut);
        if (angle > 2.6f) continue;
        float speed = std::min(CornerSpeed(angle, lenIn, lenOut, grip, halfLen), desired);
        options[numOptions] = exitAddr;
        optionSpeed[numOptions] = speed;
        optionPos[numOptions] = GetNodePos(exitNode);
        optionClass[numOptions] = exitNode->Speed;
        best = std::max(best, speed);
        numOptions++;
    }
    if (numOptions == 0) return false;

    CVector2D toDest = xdata.destination - junctionPos.To2D();
    float destDist = toDest.Magnitude();
    if (destDist > 0.1f) toDest *= 1.0f / destDist;
    int chosen = 0;
    float bestScore = -1e9f;
    for (int i = 0; i < numOptions; i++) {
        if (xdata.bRacer && optionSpeed[i] < best * 0.9f) continue;
        CVector2D dir = (optionPos[i] - junctionPos).To2D();
        float len = dir.Magnitude();
        if (len > 0.1f) dir *= 1.0f / len;
        float score = cfgDestinations && xdata.hasDestination ? CVector2D::Dot(dir, toDest) : 0.0f;
        if (cfgDestinations && destDist > 600.0f) score += optionClass[i] >= 2 ? 0.4f : (optionClass[i] == 0 ? -0.3f : 0.0f);
        score += CGeneral::GetRandomNumberInRange(-0.15f, 0.15f); // not all drivers think alike
        if (score > bestScore) { bestScore = score; chosen = i; }
    }

    addPlan(options[chosen]);
    return true;
}

// Height of the bottom of the vehicle (path nodes are at road level)
static float GetVehicleBaseZ(CVehicle* veh) {
    return veh->GetPosition().z + veh->GetColModel()->m_boundBox.m_vecMin.z;
}

// Speed allowed now to still reach 'pointSpeed' at a point 'dist' ahead braking at 'decel'.
// Going downhill gravity fights the brakes; uphill helps (energy: v^2 = vp^2 + 2*b*d + 2*g*dh)
static float BrakingSpeed(float pointSpeed, float dist, float decel, float heightDiff) {
    float v2 = pointSpeed * pointSpeed + 2.0f * decel * std::max(dist, 0.0f);
    if (cfgSlopeAware) v2 += 2.0f * GRAVITY * heightDiff;
    return std::sqrt(std::max(v2, pointSpeed * pointSpeed * 0.5f));
}

static CVector2D IntersectLines(const CVector2D& p, const CVector2D& u, const CVector2D& q, const CVector2D& v, bool& ok) {
    float den = CVector2D::Cross(u, v);
    ok = std::abs(den) > 0.0001f;
    if (!ok) return p;
    float t = CVector2D::Cross(q - p, v) / den;
    return p + u * t;
}

// Racing line: the shortest path (a taut string) from the vehicle through the corridor made of the lanes of
// OUR direction only. It never crosses the road center, so it never goes into oncoming traffic.
// Lane geometry is the same the AI uses: lanes to the right of the link position, (OneWayLaneOffset + lane) * 5.4m
static void BuildRacingLine(CVehicle* veh, ExtendedData& xdata, const CNodeAddress* addrs, CPathNode* const* nodes, int count,
    float halfWidth, float decel, float grip)
{
    xdata.lineCount = 0;
    xdata.inCorridor = false;
    xdata.lineCurveSpeed = 1000.0f;
    xdata.lineCurveDist = 0.0f;

    struct Segment { CVector2D c, d, r; float lo, hi, z; };
    Segment segs[MAX_LOOKAHEAD_NODES];
    int numSegs = 0;
    const float margin = halfWidth + cfgRacingLineMargin;
    for (int i = 0; i < count - 1; i++) {
        CCarPathLink* link = GetLinkBetween(addrs[i], nodes[i], addrs[i + 1]);
        if (!link) break;
        int lanes = LanesToward(link, addrs[i + 1]);
        CVector2D d = (xdata.path[i + 1] - xdata.path[i]).To2D();
        float len = d.Magnitude();
        if (lanes == 0 || len < 0.5f) break;
        d *= 1.0f / len;

        Segment& s = segs[numSegs++];
        s.c = CVector2D(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
        s.d = d;
        s.r = CVector2D(d.y, -d.x); // right side of the travel direction
        float ow = link->OneWayLaneOffset();
        s.lo = (ow - 0.5f) * LANE_WIDTH + margin;
        s.hi = (ow + lanes - 0.5f) * LANE_WIDTH - margin;
        if (s.lo > s.hi) s.lo = s.hi = (ow + (lanes - 1) * 0.5f) * LANE_WIDTH;
        s.z = (xdata.path[i].z + xdata.path[i + 1].z) * 0.5f;
    }
    if (numSegs < 2) return;

    const CVector2D pos = veh->GetPosition().To2D();
    const CVector2D fwd = veh->GetForward().To2D();

    // Is the vehicle inside its own lanes now? (otherwise it's recovering/avoiding: leave it to the game)
    int nearest = std::abs(CVector2D::Dot(pos - segs[0].c, segs[0].d)) < std::abs(CVector2D::Dot(pos - segs[1].c, segs[1].d)) ? 0 : 1;
    float lateral = CVector2D::Dot(pos - segs[nearest].c, segs[nearest].r);
    xdata.inCorridor = lateral > segs[nearest].lo - cfgRacingLineMargin - 0.3f && lateral < segs[nearest].hi + cfgRacingLineMargin + 0.3f;

    // Portals (cross sections of the corridor): one per link, plus a mitered one at each corner
    CVector2D portalL[MAX_LINE_POINTS], portalR[MAX_LINE_POINTS];
    float portalZ[MAX_LINE_POINTS];
    int numPortals = 0;
    bool started = false;
    auto addPortal = [&](const CVector2D& l, const CVector2D& r, float z) {
        if (numPortals >= MAX_LINE_POINTS - 1) return;
        if (!started && CVector2D::Dot((l + r) * 0.5f - pos, fwd) < 2.0f) return; // behind us
        started = true;
        portalL[numPortals] = l;
        portalR[numPortals] = r;
        portalZ[numPortals] = z;
        numPortals++;
    };
    for (int s = 0; s < numSegs; s++) {
        addPortal(segs[s].c + segs[s].r * segs[s].lo, segs[s].c + segs[s].r * segs[s].hi, segs[s].z);
        if (s + 1 < numSegs && CVector2D::Dot(segs[s].d, segs[s + 1].d) < 0.99f) { // corner > 8 degrees
            bool okL, okR;
            CVector2D l = IntersectLines(segs[s].c + segs[s].r * segs[s].lo, segs[s].d, segs[s + 1].c + segs[s + 1].r * segs[s + 1].lo, segs[s + 1].d, okL);
            CVector2D r = IntersectLines(segs[s].c + segs[s].r * segs[s].hi, segs[s].d, segs[s + 1].c + segs[s + 1].r * segs[s + 1].hi, segs[s + 1].d, okR);
            CVector2D node = xdata.path[s + 1].To2D();
            if (okL && okR && (l - node).Magnitude() < 40.0f && (r - node).Magnitude() < 40.0f) {
                addPortal(l, r, xdata.path[s + 1].z);
            }
        }
    }
    if (numPortals < 2) return;

    // Pull the string tight: each point moves to where the straight line between its neighbours crosses its portal
    CVector2D pts[MAX_LINE_POINTS];
    float t[MAX_LINE_POINTS];
    pts[0] = pos;
    for (int j = 0; j < numPortals; j++) {
        t[j] = 0.5f;
        pts[j + 1] = portalL[j] + (portalR[j] - portalL[j]) * 0.5f;
    }
    for (int iter = 0; iter < 24; iter++) {
        for (int j = 0; j < numPortals - 1; j++) { // last portal stays in the middle of our lanes
            const CVector2D& a = pts[j];
            const CVector2D& b = pts[j + 2];
            CVector2D ab = b - a;
            CVector2D lr = portalR[j] - portalL[j];
            float den = CVector2D::Cross(ab, lr);
            if (std::abs(den) > 0.0001f) t[j] = std::clamp(-CVector2D::Cross(ab, portalL[j] - a) / den, 0.0f, 1.0f);
            pts[j + 1] = portalL[j] + lr * t[j];
        }
    }

    xdata.lineCount = numPortals + 1;
    xdata.line[0] = veh->GetPosition();
    for (int j = 0; j < numPortals; j++) xdata.line[j + 1] = CVector(pts[j + 1].x, pts[j + 1].y, portalZ[j] + 1.0f);

    // Speed limits along the racing line (bigger radius than the lane center)
    CVector2D simple[MAX_LINE_POINTS];
    float simpleZ[MAX_LINE_POINTS];
    int numSimple = 0;
    simpleZ[numSimple] = GetVehicleBaseZ(veh);
    simple[numSimple++] = pts[0];
    for (int j = 1; j <= numPortals; j++) {
        if (j < numPortals && numSimple >= 1) {
            CVector2D in = pts[j] - simple[numSimple - 1], out = pts[j + 1] - pts[j];
            float li = in.Magnitude(), lo = out.Magnitude();
            if (li < 0.3f) continue;
            if (lo > 0.3f && CVector2D::Dot(in, out) / (li * lo) > 0.99996f) continue; // collinear (< 0.5 degree)
        }
        simpleZ[numSimple] = portalZ[j - 1];
        simple[numSimple++] = pts[j];
    }
    float dist = 0.0f;
    for (int j = 1; j < numSimple - 1; j++) {
        CVector2D in = simple[j] - simple[j - 1], out = simple[j + 1] - simple[j];
        float lenIn = in.Magnitude(), lenOut = out.Magnitude();
        dist += lenIn;
        if (lenIn < 0.3f || lenOut < 0.3f) continue;
        float angle = std::acos(std::clamp(CVector2D::Dot(in, out) / (lenIn * lenOut), -1.0f, 1.0f));
        float halfLen;
        float cornerSpeed = CornerSpeed(angle, lenIn, lenOut, grip, halfLen);
        if (cornerSpeed >= 1000.0f) continue;
        float d = std::max(dist - halfLen, 0.0f);
        float allowed = BrakingSpeed(cornerSpeed, d, decel, simpleZ[j] - simpleZ[0]);
        if (allowed < xdata.lineCurveSpeed) {
            xdata.lineCurveSpeed = allowed;
            xdata.lineCurveDist = d;
        }
    }
}

static void UpdateRouteLimits(CVehicle* veh, ExtendedData& xdata, float speed, float decel, float grip, float junctionSpeed,
    float lightApproachSpeed, float desired, float halfWidth)
{
    xdata.curveSpeed = 1000.0f;
    xdata.curveDist = 0.0f;
    xdata.curveIsJunction = false;
    xdata.lightSpeed = 1000.0f;
    xdata.lightDist = 0.0f;
    xdata.lightColor = LIGHT_NONE;
    xdata.pathCount = 0;
    xdata.limitingNode = -1;
    xdata.lineCount = 0;
    xdata.lineCurveSpeed = 1000.0f;
    xdata.crossSpeed = 1000.0f;

    const bool planRoute = cfgRoutePlanning;
    const CAutoPilot& ap = veh->m_autoPilot;
    CNodeAddress addrs[MAX_LOOKAHEAD_NODES + 1];
    addrs[0] = ap.field_8;              // previous route node
    addrs[1] = ap.m_currentAddress;     // current route node
    addrs[2] = ap.m_startingRouteNode;  // next route node

    CPathNode* nodes[MAX_LOOKAHEAD_NODES + 1];
    int count = 0;
    for (int i = 0; i < 3; i++) {
        nodes[i] = GetNode(addrs[i]);
        if (!nodes[i]) break;
        xdata.path[i] = GetNodePos(nodes[i]);
        count++;
    }
    if (count < 3) {
        xdata.pathCount = count;
        return;
    }

    // Follow the road beyond the next node, at least as far as needed to brake from the current speed.
    // Normal drivers stop at the first junction (we don't know where the AI will go); racers go on
    // through the exits they planned
    float lookahead = std::min(std::max({ cfgMaxLookaheadDist, speed * speed / (2.0f * decel) + speed * 1.5f,
        std::max(speed * cfgLanePrepareTime, 80.0f) + 20.0f }), MAX_LOOKAHEAD_DIST);
    bool endsInJunction = false;
    float lengthSoFar = (xdata.path[2] - xdata.path[1]).Magnitude2D();
    while (count < MAX_LOOKAHEAD_NODES + 1 && lengthSoFar < lookahead) {
        CPathNode* last = nodes[count - 1];
        CNodeAddress next;
        bool found = false;
        if (last->NumberAdjNodes == 2) {
            next = GetAdjacentNode(last, addrs[count - 1], 0);
            if (next == addrs[count - 2]) next = GetAdjacentNode(last, addrs[count - 1], 1);
            found = next != addrs[count - 2];
        }
        else if (last->NumberAdjNodes > 2 && planRoute) {
            found = GetPlannedExit(xdata, addrs[count - 1], last, addrs[count - 2], xdata.path[count - 2], xdata.path[count - 1],
                grip, desired, next);
        }
        if (!found) {
            endsInJunction = last->NumberAdjNodes > 2;
            break;
        }
        CPathNode* node = GetNode(next);
        if (!node) break;
        addrs[count] = next;
        nodes[count] = node;
        xdata.path[count] = GetNodePos(node);
        lengthSoFar += (xdata.path[count] - xdata.path[count - 1]).Magnitude2D();
        count++;
    }
    xdata.pathCount = count;
    for (int i = 0; i + 1 < count; i++) {
        CCarPathLink* link = GetLinkBetween(addrs[i], nodes[i], addrs[i + 1]);
        xdata.pathLinkOk[i] = link != nullptr;
        xdata.pathLink[i] = link;
        if (link) {
            xdata.pathLinkPos[i] = CVector2D(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
            xdata.pathLaneOffset[i] = link->OneWayLaneOffset();
        }
    }

    // Forget plans of junctions we're no longer heading to (the current node is kept: the game may be deciding there now)
    for (int p = 0; p < xdata.planCount;) {
        bool stillAhead = false;
        for (int i = 1; i < count && !stillAhead; i++) stillAhead = xdata.plans[p].junction == addrs[i];
        if (stillAhead) { p++; continue; }
        for (int q = p + 1; q < xdata.planCount; q++) xdata.plans[q - 1] = xdata.plans[q];
        xdata.planCount--;
    }

    const CVector pos = veh->GetPosition();
    const CVector fwd = veh->GetForward();

    // Distance along the route from the vehicle to node i
    float distToNode[MAX_LOOKAHEAD_NODES + 1];
    distToNode[1] = (xdata.path[1] - pos).Magnitude2D();
    bool currentBehind = CVector2D::Dot((xdata.path[1] - pos).To2D(), fwd.To2D()) < 0.0f;
    if (currentBehind) distToNode[1] = -distToNode[1];
    for (int i = 2; i < count; i++) {
        distToNode[i] = distToNode[i - 1] + (xdata.path[i] - xdata.path[i - 1]).Magnitude2D();
    }
    distToNode[0] = distToNode[1] - (xdata.path[1] - xdata.path[0]).Magnitude2D();

    const float baseZ = GetVehicleBaseZ(veh);
    xdata.curveIsCrest = false;
    auto applyLimit = [&](float pointSpeed, float dist, int node, bool junction, bool crest = false) {
        float allowed = BrakingSpeed(pointSpeed, dist, decel, xdata.path[node].z - baseZ);
        if (allowed < xdata.curveSpeed) {
            xdata.curveSpeed = allowed;
            xdata.curveDist = std::max(dist, 0.0f);
            xdata.limitingNode = node;
            xdata.curveIsJunction = junction;
            xdata.curveIsCrest = crest;
        }
    };

    // Curves. The route's polyline resampled every CURVE_STEP m and, at each sample, the radius of the circle through
    // the points a window before and after it. Node by node (the angle at each node) the jagged node positions read
    // sharp bends in smooth arcs: cars braked 100 -> 60 for one node and floored it again after it
    {
        constexpr float CURVE_STEP = 3.0f;
        constexpr int MAX_SAMPLES = 200;
        static CVector2D sp[MAX_SAMPLES];
        static float sd[MAX_SAMPLES];
        static int sn[MAX_SAMPLES];
        static bool sj[MAX_SAMPLES];
        int n = 0;
        for (int i = 0; i + 1 < count && n < MAX_SAMPLES; i++) {
            CVector2D a = xdata.path[i].To2D(), ab = xdata.path[i + 1].To2D() - a;
            float len = ab.Magnitude();
            if (len < 0.1f) continue;
            // From the first sample on, every CURVE_STEP m of route
            float start = n == 0 ? 0.0f : std::fmod(CURVE_STEP - std::fmod(distToNode[i] - sd[0], CURVE_STEP), CURVE_STEP);
            for (float t = start; t < len && n < MAX_SAMPLES; t += CURVE_STEP) {
                sp[n] = a + ab * (t / len);
                sd[n] = distToNode[i] + t;
                sn[n] = t < len * 0.5f ? i : i + 1;
                sj[n] = nodes[i]->NumberAdjNodes > 2 || nodes[i + 1]->NumberAdjNodes > 2;
                n++;
            }
        }
        // Window: through junctions ~12 m each side (longer ones cut the tight turns). Along the road (no junction in it)
        // much longer, many nodes: a wide curve is an arc over its whole length, a short window only sees the jagged
        // node positions (curves that can be taken at 120 were braked to 60)
        const int kTurn = 4;
        const int kRoad = std::clamp(static_cast<int>(std::clamp(speed * 1.0f, 15.0f, 36.0f) / CURVE_STEP + 0.5f), kTurn, 12);
        for (int j = kTurn; j + kTurn < n; j++) {
            if (sd[j] < -CURVE_STEP) continue; // behind us
            int k = kRoad;
            for (int q = std::max(j - kRoad, 0); q <= std::min(j + kRoad, n - 1) && k == kRoad; q++) if (sj[q]) k = kTurn;
            if (j - k < 0 || j + k >= n) k = kTurn;
            CVector2D p0 = sp[j - k], p1 = sp[j], p2 = sp[j + k];
            float a = (p1 - p0).Magnitude(), b = (p2 - p1).Magnitude(), c = (p2 - p0).Magnitude();
            float cross = std::abs(CVector2D::Cross(p1 - p0, p2 - p0));
            if (a < 0.5f || b < 0.5f || c < 0.5f) continue;
            float curvature = 2.0f * cross / (a * b * c); // 1 / circumradius
            if (curvature < 0.002f) continue;            // radius > 500 m
            // Junction turns (the short window): less of the grip. Into them at the open-road share, cars swung wide across
            // the oncoming lanes
            // The fast-curve boost only for curves along the road (the long window), not junction/ramp turns
            float cornerSpeed = k == kTurn ? std::sqrt(grip * cfgJunctionGripFactor * GRAVITY / curvature) : CurveSpeedFor(grip, curvature);
            // Tight junction turns: a handbrake turn rotates the car, it goes in faster (and doesn't brake to a crawl)
            if (k == kTurn && xdata.handbrakeCapable && curvature > 1.0f / 30.0f) cornerSpeed *= cfgHandbrakeEntryBoost;
            applyLimit(cornerSpeed, sd[j], sn[j], false);
        }
    }

    for (int i = 1; i < count - 1; i++) {
        float lenIn, lenOut;
        TurnAngle(xdata.path[i - 1], xdata.path[i], xdata.path[i + 1], lenIn, lenOut);

        // Crest: going up then down. Faster than sqrt(g * R) the car takes off (vertical radius like a curve's)
        if (cfgSlopeAware && cfgCrestFactor > 0.0f && lenIn > 0.5f && lenOut > 0.5f && distToNode[i] > -lenOut * 0.5f) {
            float pitchIn = std::atan((xdata.path[i].z - xdata.path[i - 1].z) / lenIn);
            float pitchOut = std::atan((xdata.path[i + 1].z - xdata.path[i].z) / lenOut);
            float bend = pitchIn - pitchOut;
            if (bend > 0.03f) {
                float halfLenV = 0.5f * std::min(lenIn, lenOut);
                float radiusV = halfLenV / std::tan(bend * 0.5f);
                applyLimit(std::sqrt(GRAVITY * radiusV * cfgCrestFactor), distToNode[i] - halfLenV, i, false, true);
            }
        }
    }

    if (planRoute && cfgRacingLine && xdata.bRacer) {
        BuildRacingLine(veh, xdata, addrs, nodes, count, halfWidth, decel, grip);
    }

    // The lane the route needs at the next junction, and the next turn (for the indicators). The planned exit is
    // compared with the other exits of the junction: the rightmost one needs the right lane (right turns and exits
    // to the right), a real left turn that's the leftmost one needs the left lane
    xdata.requiredSide = 0;
    xdata.requiredDist = 1000.0f;
    xdata.turnSide = 0;
    xdata.turnDist = 1000.0f;
    xdata.turnAngle = 0.0f;
    for (int j = 1; j + 1 < count; j++) {
        if (nodes[j]->NumberAdjNodes <= 2 || distToNode[j] < 0.0f) continue;
        CVector2D in = (xdata.path[j] - xdata.path[j - 1]).To2D();
        float inAngle = CGeneral::GetATanOfXY(in.x, in.y);
        auto turnOf = [&](const CVector& to) {
            CVector2D out = (to - xdata.path[j]).To2D();
            return WrapAngle(CGeneral::GetATanOfXY(out.x, out.y) - inAngle); // > 0 = left
        };
        float ours = turnOf(xdata.path[j + 1]);
        bool anyRight = false, anyLeft = false;
        for (int k = 0; k < nodes[j]->NumberAdjNodes; k++) {
            CNodeAddress other = GetAdjacentNode(nodes[j], addrs[j], k);
            if (other == addrs[j - 1] || other == addrs[j + 1]) continue;
            CPathNode* otherNode = GetNode(other);
            CCarPathLink* link = otherNode ? GetLinkBetween(addrs[j], nodes[j], other) : nullptr;
            if (!link || LanesToward(link, other) == 0) continue; // can't be taken anyway
            float t = turnOf(GetNodePos(otherNode));
            if (std::abs(t) > 2.6f) continue;
            if (t < ours) anyRight = true;
            else anyLeft = true;
        }
        if (!anyRight && anyLeft) xdata.requiredSide = 1;                        // ours is the rightmost exit
        else if (!anyLeft && anyRight && ours > 0.5f) xdata.requiredSide = -1;  // a left turn, the leftmost exit
        xdata.requiredDist = distToNode[j];
        if (std::abs(ours) > 0.5f) {
            xdata.turnSide = ours > 0.0f ? -1 : 1;
            xdata.turnDist = distToNode[j];
            xdata.turnAngle = std::abs(ours);
            CVector2D exitDir = (xdata.path[j + 1] - xdata.path[j]).To2D();
            float exitLen = exitDir.Magnitude();
            xdata.turnExitDir = exitLen > 0.1f ? exitDir * (1.0f / exitLen) : CVector2D(0.0f, 0.0f);
        }
        break; // the next junction only
    }

    // The road loses lanes ahead (before the next junction): the game squeezes the lanes that end into the last one left
    // (lane index clamped, 0x42E99B), so cars there must merge before, signalling
    xdata.dropLanes = 0;
    xdata.dropDist = 1000.0f;
    {
        CCarPathLink* first = GetLinkBetween(addrs[1], nodes[1], addrs[2]);
        int lanesNow = first ? LanesToward(first, addrs[2]) : 0;
        for (int i = 2; i + 1 < count && lanesNow > 1; i++) {
            // A junction: lanes are chosen there (OnNewSegment), unless the route goes straight on through it into fewer
            // lanes (most lanes in this map end at a junction node: the zipper never started)
            if (nodes[i]->NumberAdjNodes > 2) {
                float lenIn, lenOut;
                if (TurnAngle(xdata.path[i - 1], xdata.path[i], xdata.path[i + 1], lenIn, lenOut) > 0.35f) break;
            }
            CCarPathLink* link = GetLinkBetween(addrs[i], nodes[i], addrs[i + 1]);
            if (!link) break;
            int lanesThere = LanesToward(link, addrs[i + 1]);
            if (lanesThere > 0 && lanesThere < lanesNow) { xdata.dropLanes = lanesThere; xdata.dropDist = distToNode[i]; break; }
        }
    }

    // Junction we couldn't plan (normal drivers, or no legal exit known): assume the sharpest exit,
    // but never slower than junctionSpeed (so gentle forks/merges don't slow racers at all)
    if (endsInJunction && junctionSpeed > 0.0f) {
        const int j = count - 1;
        CPathNode* junction = nodes[j];
        float worstExit = 1000.0f;
        float worstHalfLen = 0.0f;
        for (int k = 0; k < junction->NumberAdjNodes; k++) {
            CNodeAddress exitAddr = GetAdjacentNode(junction, addrs[j], k);
            if (exitAddr == addrs[j - 1]) continue;
            CPathNode* exitNode = GetNode(exitAddr);
            if (!exitNode) { worstExit = std::min(worstExit, junctionSpeed); continue; }
            float lenIn, lenOut, halfLen;
            float angle = TurnAngle(xdata.path[j - 1], xdata.path[j], GetNodePos(exitNode), lenIn, lenOut);
            if (angle > 2.6f) continue; // U-turn back down the other carriageway, AI doesn't do that at speed
            float exitSpeed = CornerSpeed(angle, lenIn, lenOut, grip, halfLen);
            if (exitSpeed < worstExit) { worstExit = exitSpeed; worstHalfLen = halfLen; }
        }
        float allowedAtJunction = std::max(worstExit, junctionSpeed);
        if (allowedAtJunction < 1000.0f) {
            float d = distToNode[j] - std::max(worstHalfLen, 8.0f);
            applyLimit(allowedAtJunction, d, j, true);
            float allowed = BrakingSpeed(allowedAtJunction, d, decel, xdata.path[j].z - baseZ);
            if (allowed < xdata.lineCurveSpeed) { xdata.lineCurveSpeed = allowed; xdata.lineCurveDist = std::max(d, 0.0f); }
        }
    }

    // Traffic lights: the first one ahead that applies to our direction, on any segment of the route
    if (lightApproachSpeed > 0.0f) {
        for (int i = 0; i < count - 1; i++) {
            CCarPathLink* link = GetLinkBetween(addrs[i], nodes[i], addrs[i + 1]);
            if (!link) continue;
            eLightColor color = GetLinkLightColor(link, addrs[i + 1]);
            if (color == LIGHT_NONE) continue;

            // Stop line: the link position projected on the segment, minus a margin
            CVector linkPos(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f, xdata.path[i + 1].z);
            CVector2D seg = (xdata.path[i + 1] - xdata.path[i]).To2D();
            float segLen = seg.Magnitude();
            float along = segLen > 0.1f ? std::clamp(CVector2D::Dot((linkPos - xdata.path[i]).To2D(), seg) / segLen, 0.0f, segLen) : 0.0f;
            float dist = distToNode[i] + along - cfgLightStopMargin;
            if (dist < -cfgLightStopMargin) continue; // already crossed it

            // Yellow: stop if it can be done comfortably (aggressive drivers accept a harder stop before going through)
            float yellowDecel = decel * Lerp(0.8f, 1.25f, xdata.aggression);
            if (xdata.runsRedLights && color != LIGHT_GREEN) lightApproachSpeed = std::min(lightApproachSpeed, RED_LIGHT_RUN_SPEED);
            bool mustStop = !xdata.runsRedLights && (color == LIGHT_RED ||
                (color == LIGHT_YELLOW && speed * speed / (2.0f * yellowDecel) < std::max(dist, 0.0f)));
            float dz = linkPos.z - baseZ;
            float allowed = mustStop ? BrakingSpeed(0.0f, dist, decel, dz) : BrakingSpeed(lightApproachSpeed, dist, decel, dz);
            // Green: through it at the road's speed (if it turns yellow, the yellow rule above decides in time)
            if (color == LIGHT_GREEN && cfgGreenLightFullSpeed) allowed = 1000.0f;

            xdata.lightSpeed = allowed;
            xdata.lightDist = std::max(dist, 0.0f);
            xdata.lightColor = color;
            xdata.lightPos = linkPos;
            break;
        }
    }

    // Cross traffic: before going into the next junction, make sure nobody is crossing it or about to
    xdata.crossSpeed = 1000.0f;
    if (cfgJunctionProtection) {
        for (int j = 1; j < count; j++) {
            if (nodes[j]->NumberAdjNodes <= 2) continue;
            const float stopBefore = 12.0f;
            float distToJunction = distToNode[j];
            if (distToJunction > std::max(60.0f, speed * 4.0f)) break;

            // Junctions without lights in town: first come, first served (a 4-way stop). Everyone within 30 m of it says
            // when it got there; a car goes in only if nobody coming from another direction got there before it (the
            // ones stuck for 6 s don't count: they're held by something else). With nobody else around there's no wait.
            // At the way into Bayside cars from all four sides went in together and locked the junction for good
            if (cfgJunctionFirstCome && nodes[j]->Speed < 2 && distToJunction < 30.0f && distToJunction > -12.0f && j >= 1) {
                bool lights = xdata.pathLinkOk[j - 1] && GetLinkLightColor(xdata.pathLink[j - 1], addrs[j]) != LIGHT_NONE;
                CVector2D dirIn = (xdata.path[j] - xdata.path[j - 1]).To2D();
                float dirLen = dirIn.Magnitude();
                if (!lights && dirLen > 0.1f) {
                    dirIn *= 1.0f / dirLen;
                    const unsigned int now = CTimer::m_snTimeInMilliseconds;
                    auto& claims = junctionClaims;
                    claims.erase(std::remove_if(claims.begin(), claims.end(), [&](const JunctionClaim& c) {
                        return now - c.lastSeen > 1500 || !CPools::ms_pVehiclePool->IsObjectValid(c.veh);
                    }), claims.end());
                    JunctionClaim* mine = nullptr;
                    for (JunctionClaim& c : claims) if (c.veh == veh) mine = &c;
                    // A new junction (not the one it claimed before): a new arrival
                    if (mine && (mine->pos - xdata.path[j].To2D()).Magnitude() > 30.0f) { mine->arrival = now; mine->pos = xdata.path[j].To2D(); }
                    if (!mine) { claims.push_back({ veh, now, now, 0, dirIn, xdata.path[j].To2D() }); mine = &claims.back(); }
                    mine->lastSeen = now;
                    mine->dir = dirIn;
                    if (speed < 1.0f) { if (!mine->stillSince) mine->stillSince = now; }
                    else mine->stillSince = 0;
                    // Already in the intersection (the node before this one is part of it): never stop in there, get out
                    const bool insideAlready = nodes[j - 1]->NumberAdjNodes > 2 && (xdata.path[j] - xdata.path[j - 1]).Magnitude2D() < 30.0f &&
                        distToNode[j - 1] < 2.0f;
                    if (distToJunction > stopBefore * 0.5f && !insideAlready) {
                        const unsigned int myArrival = mine->arrival;
                        for (const JunctionClaim& c : claims) {
                            if (c.veh == veh || CVector2D::Dot(c.dir, dirIn) > 0.7f) continue; // same way in: a queue
                            if ((c.pos - xdata.path[j].To2D()).Magnitude() > 30.0f) continue;              // another junction
                            if (c.stillSince && now - c.stillSince > 6000) continue;
                            bool earlier = c.arrival < myArrival || (c.arrival == myArrival && c.veh < veh);
                            if (!earlier) continue;
                            float allowed = BrakingSpeed(0.0f, distToJunction - stopBefore * 0.5f, decel, 0.0f);
                            if (allowed < xdata.crossSpeed) {
                                xdata.crossSpeed = allowed;
                                xdata.crossDist = std::max(distToJunction - stopBefore * 0.5f, 0.0f);
                                xdata.crossPos = c.veh->GetPosition();
                                if (now - xdata.firstComeWaitTime > 3000) firstComeWaitCount++;
                                xdata.firstComeWaitTime = now;
                            }
                            break;
                        }
                    }
                }
            }
            if (distToJunction < stopBefore) break; // committed: too close to stop, the game's avoidance takes over

            CVector2D junctionPos = xdata.path[j].To2D();
            CVector2D approach = (xdata.path[j] - xdata.path[j - 1]).To2D();
            float approachLen = approach.Magnitude();
            if (approachLen < 0.1f) break;
            approach *= 1.0f / approachLen;

            // Don't block the box: the road we leave the junction by has a stopped vehicle just past it (a queue reaching
            // back to the junction): wait before the junction, not in it. Cars stopped in the middle of a junction behind
            // such a queue held the cross street on a green light, with twice the traffic the whole area locked up
            const bool inIntersection = nodes[j - 1]->NumberAdjNodes > 2 && (xdata.path[j] - xdata.path[j - 1]).Magnitude2D() < 30.0f &&
                distToNode[j - 1] < 2.0f; // already in it: never stop in there
            if (j + 1 < count && xdata.pathLinkOk[j] && !inIntersection) {
                CVector2D exitDir = (xdata.path[j + 1] - xdata.path[j]).To2D();
                float exitLen = exitDir.Magnitude();
                if (exitLen > 1.0f) {
                    exitDir *= 1.0f / exitLen;
                    const CVector2D er(exitDir.y, -exitDir.x);
                    const int exitLanes = LanesToward(xdata.pathLink[j], addrs[j + 1]);
                    // Room for the whole car past the far side of the junction: from ~6 m past its centre (beyond its middle),
                    // our length + 8 m. It looked 4-10 m past the centre only (the car's width was used as its length):
                    // in big junctions that's still inside, and the queue beyond went unseen. Crawling queues count too
                    const float carLen = std::max(veh->GetColModel() ? veh->GetColModel()->m_boundBox.m_vecMax.y -
                        veh->GetColModel()->m_boundBox.m_vecMin.y : 5.0f, 4.0f);
                    const float exitStart = std::max(6.0f, exitLanes * LANE_WIDTH * 0.5f + 3.0f);
                    bool boxBlocked = false;
                    for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize && !boxBlocked; k++) {
                        CVehicle* o = CPools::ms_pVehiclePool->GetAt(k);
                        if (!o || o == veh || std::abs(o->GetPosition().z - xdata.path[j].z) > 4.0f) continue;
                        if (o->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS > 3.0f) continue;
                        CVector2D op = o->GetPosition().To2D();
                        float lon = CVector2D::Dot(op - junctionPos, exitDir);
                        if (lon < exitStart || lon > exitStart + carLen + 8.0f) continue;
                        float laneF = CVector2D::Dot(op - xdata.pathLinkPos[j], er) / LANE_WIDTH - xdata.pathLaneOffset[j];
                        boxBlocked = laneF > -0.6f && laneF < exitLanes - 0.4f;
                    }
                    if (boxBlocked) {
                        float allowed = BrakingSpeed(0.0f, distToJunction - stopBefore * 0.6f, decel, 0.0f);
                        if (allowed < xdata.crossSpeed) {
                            xdata.crossSpeed = allowed;
                            xdata.crossDist = distToJunction - stopBefore * 0.6f;
                            xdata.crossPos = xdata.path[j + 1];
                            if (CTimer::m_snTimeInMilliseconds - xdata.boxWaitTime > 3000) boxWaitCount++;
                            xdata.boxWaitTime = CTimer::m_snTimeInMilliseconds;
                        }
                    }
                }
            }

            for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize; k++) {
                CVehicle* other = CPools::ms_pVehiclePool->GetAt(k);
                if (!other || other == veh || other->m_nStatus == STATUS_WRECKED || other == veh->m_pTrailer || other == veh->m_pTractor) continue;
                CVector2D rel = other->GetPosition().To2D() - junctionPos;
                float distToCenter = rel.Magnitude();
                if (distToCenter > 25.0f || std::abs(other->GetPosition().z - xdata.path[j].z) > 5.0f) continue;

                CVector2D vel = other->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS;
                float otherSpeed = vel.Magnitude();
                CVector2D dir = otherSpeed > 1.5f ? vel * (1.0f / otherSpeed) : other->GetForward().To2D();
                float alignment = std::abs(CVector2D::Dot(dir, approach));
                if (alignment > 0.75f) continue; // same road as us (ahead of us, or oncoming going straight)

                // Stopped cars only count when really in the middle (not waiting at the cross street's stop line)
                bool insideCore = distToCenter < (otherSpeed > 1.5f ? 12.0f : 7.0f);
                bool arriving = otherSpeed > 1.5f && CVector2D::Dot(-rel, vel) > 0.0f && distToCenter / otherSpeed < 3.0f;
                if (!insideCore && !arriving) continue;

                float allowed = BrakingSpeed(0.0f, distToJunction - stopBefore, decel, xdata.path[j].z - baseZ);
                if (allowed < xdata.crossSpeed) {
                    xdata.crossSpeed = allowed;
                    xdata.crossDist = distToJunction - stopBefore;
                    xdata.crossPos = other->GetPosition();
                }
            }
            break; // only the first junction ahead
        }
    }
}

// Distance between the front and rear axles: from the wheels' suspension lines of the collision model (the 0.6 x body
// length guess was metres off on trucks and buses, long overhangs: the turn radius from the steering came out too large,
// the cement truck went into junction turns too fast and couldn't make them), per model
static std::unordered_map<int, float> wheelBases;
static float GetWheelBase(CVehicle* veh, const CVector& modelMin, const CVector& modelMax) {
    auto it = wheelBases.find(veh->m_nModelIndex);
    if (it != wheelBases.end()) return it->second;
    float wb = std::max((modelMax.y - modelMin.y) * 0.6f, 1.0f);
    CColModel* col = veh->GetColModel();
    if (col && col->m_pColData && col->m_pColData->m_nNumLines >= 4 && col->m_pColData->m_pLines) {
        float front = -1000.0f, rear = 1000.0f;
        for (int i = 0; i < 4; i++) {
            float y = col->m_pColData->m_pLines[i].m_vecStart.y;
            front = std::max(front, y);
            rear = std::min(rear, y);
        }
        if (front - rear > 1.0f && front - rear < 20.0f) wb = front - rear;
    }
    lg << "Wheelbase model " << veh->m_nModelIndex << ": " << wb << " m (body length guess " << (modelMax.y - modelMin.y) * 0.6f << " m)\n";
    wheelBases[veh->m_nModelIndex] = wb;
    return wb;
}

// Max speed (m/s) for the turn the vehicle is making right now. Curvature is the largest of what the AI
// asks with the steering wheel (bicycle model) and what the car is really doing (yaw rate / speed)
static float GetHeadingLimit(CVehicle* veh, ExtendedData& xdata, const CVector& modelMin, const CVector& modelMax, float speed, float grip) {
    float wheelBase = GetWheelBase(veh, modelMin, modelMax);
    // Small steering corrections (lane keeping, lane changes) aren't a turn: at speed 3 degrees of steering read as a
    // 30 m radius and held traffic at ~45 km/h on freeways. The steering only counts from ~8 degrees (a real turn starting)
    float steerAngle = std::max(std::abs(veh->m_fSteerAngle) - 0.14f, 0.0f);
    float steerCurvature = std::tan(std::min(steerAngle, 1.2f)) / wheelBase;
    // The real turn, smoothed (~0.4 s): the AI's steering wobbles on freeways (15-20 degrees/s back and forth), which
    // read as a tight curve for an instant and braked the car
    float yawRate = veh->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS; // rad/s
    xdata.filtYawRate += (yawRate - xdata.filtYawRate) * std::min(0.05f * CTimer::ms_fTimeStep, 1.0f);
    float yawCurvature = speed > 1.0f ? std::abs(xdata.filtYawRate) / speed : 0.0f;
    float curvature = std::max(steerCurvature, yawCurvature);
    if (curvature < 0.002f) return 1000.0f; // radius > 500 m
    return std::max(CurveSpeedFor(grip, curvature), 5.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Game AI curve braking
// CCarCtrl::SteerAICarWithPhysicsFollowPath (0x434900) wants cruise * min(curve multipliers, traffic multiplier).
// The traffic multiplier (cars ahead, red lights, train crossings) stays. The two curve multipliers
// (FindSpeedMultiplier calls @ 0x435461 / 0x4354A8) become 1.0 for the vehicles this mod drives, since
// their cruise speed already has the curve lookahead, heading limiter and traffic lights in it.

static CVehicle* followPathVehicle = nullptr;

// Pure pursuit: steering so the vehicle's arc passes through 'target'
static float SteerTowards(CVehicle* veh, const CVector2D& target) {
    const CVector2D pos = veh->GetPosition().To2D();
    CVector2D toTarget = target - pos;
    float dist = std::max(toTarget.Magnitude(), 1.0f);
    CVector2D fwd = veh->GetForward().To2D();
    float alpha = WrapAngle(CGeneral::GetATanOfXY(toTarget.x, toTarget.y) - CGeneral::GetATanOfXY(fwd.x, fwd.y));

    CVector modelMin, modelMax;
    GetEntityDimensions(veh, modelMin, modelMax);
    float wheelBase = GetWheelBase(veh, modelMin, modelMax);
    float steer = std::atan(2.0f * wheelBase * std::sin(alpha) / dist);
    float maxSteer = plugin::CallAndReturn<float, 0x427FE0, CVehicle*>(veh); // CCarCtrl::FindMaxSteerAngle
    return std::clamp(steer, -maxSteer, maxSteer);
}

// Pure pursuit on the racing line: steer so the vehicle's arc passes through the point lookDist ahead on the line
static float GetRacingLineSteer(CVehicle* veh, const ExtendedData& xdata, float speed) {
    const CVector2D pos = veh->GetPosition().To2D();
    float lookDist = std::clamp(speed * 0.55f, 6.0f, 30.0f);

    // Closest point of the line to the vehicle (the line was built a few frames ago)
    int seg = 0;
    float bestDist = 1e9f, bestT = 0.0f;
    for (int i = 0; i + 1 < xdata.lineCount && i < 8; i++) {
        CVector2D a = xdata.line[i].To2D(), ab = xdata.line[i + 1].To2D() - a;
        float len2 = ab.MagnitudeSqr();
        float tt = len2 > 0.0001f ? std::clamp(CVector2D::Dot(pos - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
        float d = (a + ab * tt - pos).MagnitudeSqr();
        if (d < bestDist) { bestDist = d; seg = i; bestT = tt; }
    }

    // Walk lookDist along the line
    CVector2D a = xdata.line[seg].To2D();
    CVector2D target = a + (xdata.line[seg + 1].To2D() - a) * bestT;
    float remaining = lookDist;
    for (int i = seg; i + 1 < xdata.lineCount && remaining > 0.0f; i++) {
        CVector2D from = (i == seg) ? target : xdata.line[i].To2D();
        CVector2D to = xdata.line[i + 1].To2D();
        float len = (to - from).Magnitude();
        if (len >= remaining) { target = from + (to - from) * (remaining / len); remaining = 0.0f; }
        else { target = to; remaining -= len; }
    }

    return SteerTowards(veh, target);
}

// Lane change: the lane position goes from where it was to the target lane along a smooth S (smoothstep) in
// changeDuration s, and the vehicle is steered to the point of that path lookDist ahead. The point follows the road
// ahead (the route's segments, each with its own lanes), so in a curve it stays on the road. The game's own steering
// would reach the new lane by the end of the current road segment: on freeways (short segments) a sharp swerve
static float GetLaneChangeSteer(CVehicle* veh, const ExtendedData& xdata, float speed, float progress) {
    if (xdata.pathCount < 3) return NAN;
    const CVector2D pos = veh->GetPosition().To2D();
    // Far enough ahead that the car follows the S instead of chasing a point beside it (sharp steering)
    float lookDist = std::clamp(speed * 1.2f, 10.0f, 40.0f);
    float t = std::clamp(progress + lookDist / std::max(speed, 1.0f) / xdata.changeDuration, 0.0f, 1.0f);
    float f = t * t * (3.0f - 2.0f * t);
    float laneF = xdata.changeFromF + (xdata.changeTarget - xdata.changeFromF) * f;

    // Walk lookDist along the route from the vehicle (segment 1 -> 2 is current node -> next node)
    int seg = 1;
    CVector2D a = xdata.path[1].To2D(), dir = xdata.path[2].To2D() - a;
    float len = dir.Magnitude();
    if (len < 0.5f) return NAN;
    dir *= 1.0f / len;
    float along = std::clamp(CVector2D::Dot(pos - a, dir), 0.0f, len) + lookDist;
    while (along > len && seg + 2 < xdata.pathCount) {
        along -= len;
        seg++;
        a = xdata.path[seg].To2D();
        dir = xdata.path[seg + 1].To2D() - a;
        len = dir.Magnitude();
        if (len < 0.5f) return NAN;
        dir *= 1.0f / len;
    }
    if (!xdata.pathLinkOk[seg]) return NAN;
    CVector2D point = a + dir * std::min(along, len);
    const CVector2D r(dir.y, -dir.x);
    const CVector2D& linkPos = xdata.pathLinkPos[seg];
    float lat = (xdata.pathLaneOffset[seg] + laneF) * LANE_WIDTH;
    CVector2D target = linkPos + dir * CVector2D::Dot(point - linkPos, dir) + r * lat;
    return SteerTowards(veh, target);
}

// Creeping up to a stopped vehicle in a queue: our own pedals (the game's car-ahead logic stops ~5 m behind it)
static bool ApplyCloseUp(CVehicle* veh, ExtendedData& xdata, float* gas, float* brake) {
    // Not while getting around a blocked lane: the close-up held the brake on behind the very car it was going around
    if (xdata.closeUpTarget < 0.0f || xdata.bypassUntil || CTimer::m_snTimeInMilliseconds - xdata.lastProcessed > 200) return false;
    float forwardSpeed = CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS;
    if (forwardSpeed < -0.3f) return false; // backing off: the game's
    float error = xdata.closeUpTarget - forwardSpeed;
    if (xdata.closeUpTarget <= 0.0f) { *gas = 0.0f; *brake = 1.0f; }                         // there: hold it
    else if (error > 0.2f) { *gas = std::clamp(0.2f + error * 0.25f, 0.0f, 0.5f); *brake = 0.0f; }
    else if (error < -0.3f) { *gas = 0.0f; *brake = std::clamp(-error * 0.5f, 0.2f, 1.0f); }
    else { *gas = 0.12f; *brake = 0.0f; }
    return true;
}

// Gas to keep the speed (drag + rolling resistance), roughly
static float CruiseGas(CVehicle* veh, float speed) {
    float topSpeed = std::max(GetVehicleTopSpeed(veh), 10.0f);
    return std::clamp((speed / topSpeed) * (speed / topSpeed) * 1.1f + 0.08f, 0.0f, 0.9f);
}

// Gas<->brake switches of the speed control, and its driving time (AutoTest: switches per minute)
static int pedalSwitchCount = 0;
static float pedalSeconds = 0.0f;

// Our own pedals: follow the target speed precisely instead of the game's rough gas dosing.
// Whenever the game itself wants to stop (car ahead, red light, train crossing, parking) its decision stays
static void ApplyPedalControl(CVehicle* veh, ExtendedData& xdata, float speed, float* gas, float* brake, bool* handbrake) {
    if (!cfgPedalControl) return;
    float forwardSpeed = CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS;
    float target = xdata.targetSpeed;
    if (forwardSpeed < -0.5f || *handbrake) return;         // rolling backwards / maneuvering
    // Braking while not too fast = car ahead, red light, train... But a light brake for a car we see too is the game's own
    // (longer) following distance: our gap rules there, or cars kept far behind on freeways
    if (*brake > 0.05f && speed < target + 0.5f &&
        (*brake > 0.5f || xdata.obstacle != OBSTACLE_VEHICLE || xdata.gap < 6.0f || speed < 3.0f)) return;
    if (*gas <= 0.01f && speed < 2.0f) return;              // the game is holding the car still
    // Queued behind a stopped/slow vehicle (traffic light, jam): the game creeps up to it, we don't push
    bool queued = (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED) && xdata.leaderSpeed < 2.0f &&
        xdata.gap < std::max(8.0f, speed * 2.0f);
    if (queued && speed < 6.0f) return;
    const float gameBrake = *brake;

    // Like a driver holding a speed: gas from the estimate of what holds it, corrected slowly (integral) and quickly
    // (proportional); the brake only when clearly too fast (from ~5 km/h over, until ~1.5 km/h over), coasting in
    // between. It used to switch gas/brake every few frames around the target (P only, braking from 3.6 km/h over)
    const float dt = std::min(CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS, 0.1f);
    float error = target - speed;
    float gain = Lerp(0.8f, 1.25f, xdata.aggression);
    float cruiseGas = CruiseGas(veh, speed);
    if (xdata.pedalBraking ? error > -0.4f : error < -1.4f) xdata.pedalBraking = !xdata.pedalBraking;

    if (xdata.pedalBraking) {
        xdata.pedalGas = 0.0f;
        xdata.speedTrim = std::max(xdata.speedTrim - 0.05f * dt, -0.3f); // it was too fast: the estimate was high
        *gas = 0.0f;
        *brake = std::max(std::clamp((-error - 0.4f) * 0.2f * gain, 0.05f, 1.0f), gameBrake); // never less than the game wanted
    }
    else {
        // Integral only near the target (no wind-up while speeding up or when the gas is at a stop)
        if (std::abs(error) < 3.0f && xdata.pedalGas > 0.02f && xdata.pedalGas < 0.98f)
            xdata.speedTrim = std::clamp(xdata.speedTrim + error * 0.06f * dt, -0.3f, 0.3f);
        float kp = error > 0.0f ? (error > 3.0f ? 0.25f : 0.15f) * gain : 0.3f;
        float want = std::clamp(cruiseGas + xdata.speedTrim + error * kp, 0.0f, 1.0f);
        // The foot moves: up to ~2.5/s pressing, ~5/s letting go
        float delta = std::clamp(want - xdata.pedalGas, -5.0f * dt, 2.5f * dt);
        xdata.pedalGas = std::clamp(xdata.pedalGas + delta, 0.0f, 1.0f);
        *gas = xdata.pedalGas;
        *brake = 0.0f;
    }
    // A little over the target the game brakes lightly for that alone (we coast instead); hard, it has a reason
    if (gameBrake > 0.5f && !xdata.pedalBraking) { *brake = gameBrake; *gas = 0.0f; xdata.pedalGas = 0.0f; }
    signed char side = *brake > 0.1f ? -1 : (*gas > 0.1f ? 1 : 0);
    if (side != 0) {
        if (xdata.pedalSide != 0 && side != xdata.pedalSide) pedalSwitchCount++;
        xdata.pedalSide = side;
    }
    pedalSeconds += dt;
    xdata.aidFlags |= AID_PEDALS;
}

// The profile's acceleration: the gas is capped so the vehicle doesn't speed up harder than its driver would (pulling
// away from a light too, where the game's own full throttle is kept). The cap follows the measured acceleration
static void ApplyAccelLimit(CVehicle* veh, ExtendedData& xdata, float speed, float* gas, float* brake) {
    float forwardSpeed = CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS;
    if (!cfgPedalControl || xdata.accelLimit <= 0.0f || *gas <= 0.0f || *brake > 0.05f || forwardSpeed < -0.5f) {
        xdata.accelLimiting = false;
        return;
    }
    const float dt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
    const float cruiseGas = CruiseGas(veh, speed);
    if (!xdata.accelLimiting) xdata.gasTrim = 0.1f + 0.06f * xdata.accelLimit; // first guess, corrected below
    xdata.accelLimiting = true;
    float error = xdata.accelLimit - xdata.measAccel;
    xdata.gasTrim = std::clamp(xdata.gasTrim + error * 0.3f * dt, -cruiseGas, 1.0f);
    float cap = std::clamp(cruiseGas + xdata.gasTrim + error * 0.05f, 0.05f, 1.0f);
    if (*gas > cap) {
        *gas = cap;
        xdata.aidFlags |= AID_ACCEL;
    }
    else xdata.gasTrim = std::min(xdata.gasTrim, *gas - cruiseGas + 0.1f); // not limiting now: no wind-up
}

// Counter-steer: when the car slides, steer into the direction it's really going, like a driver catching a slide
static void ApplyCounterSteer(CVehicle* veh, ExtendedData& xdata, float speed, float* steer) {
    if (!cfgCounterSteer || speed < 6.0f) return;
    CVector2D vel = veh->m_vecMoveSpeed.To2D();
    CVector2D fwd = veh->GetForward().To2D();
    if (CVector2D::Dot(vel, fwd) <= 0.0f) return;
    float slip = WrapAngle(CGeneral::GetATanOfXY(vel.x, vel.y) - CGeneral::GetATanOfXY(fwd.x, fwd.y));
    float amount = std::clamp((std::abs(slip) - 0.12f) / 0.2f, 0.0f, 1.0f); // starts at ~7 degrees, full at ~18
    if (amount <= 0.0f) return;
    float maxSteer = plugin::CallAndReturn<float, 0x427FE0, CVehicle*>(veh); // CCarCtrl::FindMaxSteerAngle
    float counter = std::clamp(slip * 1.2f, -maxSteer, maxSteer);
    *steer += (counter - *steer) * amount;
    xdata.aidFlags |= AID_COUNTERSTEER;
}

// ABS, traction control and stability control on the pedals
static void ApplyDriverAids(CVehicle* veh, ExtendedData& xdata, float speed, float* gas, float* brake) {
    eWheelState wheels[4];
    int numWheels = GetWheelStates(veh, wheels);
    bool locked = false, spinning = false;
    for (int i = 0; i < numWheels; i++) {
        locked |= wheels[i] == WHEEL_STATE_FIXED || wheels[i] == WHEEL_STATE_SKIDDING;
        spinning |= wheels[i] == WHEEL_STATE_SPINNING;
    }

    if (cfgABS && *brake > 0.2f && speed > 3.0f && locked) {
        *brake *= (frameCounter & 1) ? 0.35f : 0.8f; // pulse
        xdata.aidFlags |= AID_ABS;
    }
    // Progressive: the cut grows while the wheels spin and gives the gas back gradually once they grip (halving the gas
    // each frame they spun and giving it all back the next made the car surge and spin again, losing acceleration). More
    // spinning wheels, more cut. Not pulling away on a hill: some wheelspin there is normal, and cutting the gas made weak
    // cars roll back down the hills of Las Colinas
    if (cfgTractionControl) {
        int spinCount = 0;
        for (int i = 0; i < numWheels; i++) spinCount += wheels[i] == WHEEL_STATE_SPINNING;
        const float step = std::min(CTimer::ms_fTimeStep, 3.0f);
        const bool hillStart = speed < 5.0f && veh->GetForward().z > 0.08f;
        if (spinCount > 0 && *gas > 0.2f && !hillStart) xdata.tcCut = std::min(xdata.tcCut + (0.06f + 0.04f * spinCount) * step, 0.8f);
        else xdata.tcCut = std::max(xdata.tcCut - 0.04f * step, 0.0f);
        if (xdata.tcCut > 0.0f && *gas > 0.0f) {
            *gas *= 1.0f - xdata.tcCut;
            xdata.aidFlags |= AID_TC;
        }
    }
    if (cfgStabilityControl && speed > 10.0f && *gas > 0.3f && GetSlipAngle(veh) > 0.25f) { // sliding > 14 degrees
        *gas = 0.3f;
        xdata.aidFlags |= AID_ESC;
    }
}

static void FollowPathHook(CVehicle* veh, float* steer, float* gas, float* brake, bool* handbrake) {
    CVehicle* prev = followPathVehicle;
    followPathVehicle = veh;
    plugin::Call<0x434900, CVehicle*, float*, float*, float*, bool*>(veh, steer, gas, brake, handbrake);
    followPathVehicle = prev;

    // Cruise speed 0 (what this mod sets to hold a car behind a stopped one): FollowPath divides the traffic speed by
    // the cruise speed (FIDIV @ 0x435315) and returns NaN pedals. Cruise 0 means stand still
    if (!std::isfinite(*gas) || !std::isfinite(*brake)) {
        *gas = 0.0f;
        *brake = 1.0f;
    }
    if (!std::isfinite(*steer)) *steer = 0.0f;

    ExtendedData& xdata = vehExData.Get(veh);
    xdata.dbgGameGas = xdata.dbgGas = *gas;
    xdata.dbgGameBrake = xdata.dbgBrake = *brake;
    xdata.dbgPedalTime = CTimer::m_snTimeInMilliseconds;
    // Acceleration along the forward, smoothed (~0.15 s)
    float fwdSpeed = CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS;
    float frameDt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
    if (xdata.prevFwdFrame + 1 == frameCounter && frameDt > 0.001f) {
        float a = std::clamp((fwdSpeed - xdata.prevFwdSpeed) / frameDt, -30.0f, 30.0f);
        xdata.measAccel += (a - xdata.measAccel) * std::min(0.15f * CTimer::ms_fTimeStep, 1.0f);
    }
    else if (xdata.prevFwdFrame != frameCounter) xdata.measAccel = 0.0f;
    xdata.prevFwdSpeed = fwdSpeed;
    xdata.prevFwdFrame = frameCounter;

    if (cfgCloseUpInQueues && ApplyCloseUp(veh, xdata, gas, brake)) {
        xdata.dbgGas = *gas;
        xdata.dbgBrake = *brake;
        xdata.accelLimiting = false;
        return;
    }
    if (!xdata.valid) {
        xdata.lineBlend = 0.0f;
        // Pulling away from a stop it isn't 'valid' yet (not moving), but its driver's acceleration still applies
        if (xdata.accelLimit > 0.0f && CTimer::m_snTimeInMilliseconds - xdata.lastProcessed < 200) {
            ApplyAccelLimit(veh, xdata, veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS, gas, brake);
            xdata.dbgGas = *gas;
        }
        else xdata.accelLimiting = false;
        return;
    }
    float speed = veh->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;

    // Racing line steering only on a clear road, inside our own lanes; traffic avoidance stays with the game
    bool trafficClose = (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED || xdata.obstacle == OBSTACLE_OBJECT) &&
        xdata.gap < std::max(25.0f, speed * 2.0f);
    // Also off with anyone beside us (e.g. the car we just overtook): the line could cut across its lane
    bool useLine = cfgRacingLine && xdata.bRacer && xdata.lineCount >= 2 && xdata.inCorridor && speed > 8.0f && !trafficClose &&
        !xdata.trafficAround;
    float step = CTimer::ms_fTimeStep;
    xdata.lineBlend = useLine ? std::min(xdata.lineBlend + 0.05f * step, 1.0f) : std::max(xdata.lineBlend - 0.25f * step, 0.0f);
    if (xdata.lineBlend > 0.0f) {
        float lineSteer = GetRacingLineSteer(veh, xdata, speed);
        *steer += (lineSteer - *steer) * xdata.lineBlend;
    }

    if (xdata.laneState == LANE_CHANGING) {
        float progress = (CTimer::m_snTimeInMilliseconds - xdata.changeStart) / (xdata.changeDuration * 1000.0f);
        float laneSteer = progress < 1.0f ? GetLaneChangeSteer(veh, xdata, speed, progress) : NAN;
        if (progress >= 1.0f || !std::isfinite(laneSteer)) {
            // Done (or the road ended): now it's the game's lane, and the space on the board is free again
            veh->m_autoPilot.m_nCurrentLane = veh->m_autoPilot.m_nNextLane = static_cast<char>(xdata.changeTarget);
            xdata.laneState = xdata.changeAfter;
            ReleaseLaneSpace(veh);
        }
        else {
            *steer = laneSteer;
            xdata.lineBlend = 0.0f;
        }
    }

    // Going round a blocked lane (the game's bike logic): never off the edge of the road. It steered the car off a bridge.
    // Right: not past the rightmost lane. Left: on freeways and one-way roads not past the leftmost lane (elsewhere the
    // oncoming lanes are how it gets round)
    if (xdata.bypassUntil) {
        const CAutoPilot& ap = veh->m_autoPilot;
        CPathNode* curr = GetNode(ap.m_currentAddress);
        CPathNode* next = GetNode(ap.m_startingRouteNode);
        CCarPathLink* link = curr && next ? GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode) : nullptr;
        if (link) {
            int lanes = LanesToward(link, ap.m_startingRouteNode);
            int oncoming = LanesToward(link, ap.m_currentAddress);
            CVector2D d = (GetNodePos(next) - GetNodePos(curr)).To2D();
            float len = d.Magnitude();
            if (lanes > 0 && len > 0.5f) {
                d *= 1.0f / len;
                const CVector2D r(d.y, -d.x);
                const CVector2D linkPos(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
                const float ow = link->OneWayLaneOffset();
                const CVector2D pos = veh->GetPosition().To2D();
                float laneF = CVector2D::Dot(pos - linkPos, r) / LANE_WIDTH - ow;
                float headingSide = CVector2D::Dot(veh->GetForward().To2D(), r); // > 0: heading right
                bool leftLimited = oncoming == 0 || next->Speed >= 2;
                float clampTo = laneF;
                if (laneF > lanes - 0.5f && headingSide > -0.05f) clampTo = static_cast<float>(lanes - 1);
                else if (leftLimited && laneF < -0.5f && headingSide < 0.05f) clampTo = 0.0f;
                if (clampTo != laneF) {
                    CVector2D target = linkPos + d * (CVector2D::Dot(pos - linkPos, d) + 8.0f) + r * ((ow + clampTo) * LANE_WIDTH);
                    *steer = SteerTowards(veh, target);
                }
            }
        }
    }

    // Bike weaving into someone beside it (the game's bike logic doesn't look sideways): straight along the road instead
    if (CTimer::m_snTimeInMilliseconds < xdata.bikeHoldUntil) {
        *steer = SteerTowards(veh, veh->GetPosition().To2D() + xdata.bikeRoadDir * std::clamp(speed * 1.2f, 10.0f, 30.0f));
    }

    // Close behind a vehicle and closing in at low speed: full brakes. The AI's own braking near standstill is soft (it
    // aims at the cruise speed), and in queues at lights cars still rolling at 5-12 km/h with "stop" asked bumped the one
    // ahead: most of the contacts in dense traffic
    bool lowSpeedBrake = false; // applied last (pedal control would undo it)
    if (xdata.obstacle == OBSTACLE_VEHICLE && xdata.leaderVeh && speed < 8.0f && !xdata.bypassUntil &&
        CTimer::m_snTimeInMilliseconds - xdata.lastProcessed < 300) {
        float closing = speed - std::max(xdata.leaderSpeed, 0.0f);
        if (closing > 0.3f && xdata.gap < 1.2f + speed * 0.6f + closing * closing / 6.0f) {
            lowSpeedBrake = true;
            if (CTimer::m_snTimeInMilliseconds - xdata.lowBrakeTime > 2000) lowSpeedBrakeCount++;
            xdata.lowBrakeTime = CTimer::m_snTimeInMilliseconds;
        }
    }

    // Turning: someone on the outside of the turn (beside us, or coming the other way on that side): turn tighter and ease
    // off. Cars swung wide in turns into the next lane and the oncoming one
    if (cfgTurnSideGuard && speed > 3.0f && std::abs(xdata.filtYawRate) > 0.15f && veh->m_nVehicleSubClass != VEHICLE_BIKE) {
        const int outside = xdata.filtYawRate > 0.0f ? 1 : -1; // turning left: the right side is the outside
        CVector mn, mx;
        GetEntityDimensions(veh, mn, mx);
        const CVector2D pos = veh->GetPosition().To2D(), fwd = veh->GetForward().To2D(), right = veh->GetRight().To2D();
        const float halfWidth = std::max(mx.x, -mn.x);
        bool threat = false;
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize && !threat; i++) {
            CVehicle* o = CPools::ms_pVehiclePool->GetAt(i);
            if (!o || o == veh || std::abs(o->GetPosition().z - veh->GetPosition().z) > 3.0f) continue;
            CVector2D d = o->GetPosition().To2D() - pos;
            if (d.MagnitudeSqr() > 30.0f * 30.0f) continue;
            float lat = CVector2D::Dot(d, right) * outside, lon = CVector2D::Dot(d, fwd);
            CVector omn, omx;
            GetEntityDimensions(o, omn, omx);
            float oHalf = std::max(omx.x, -omn.x);
            // Beside us on the outside, within about a lane
            bool beside = lat > 0.0f && lat < halfWidth + oHalf + 2.5f && lon > mn.y - omx.y - 1.0f && lon < mx.y + omx.y + 2.0f;
            // Ahead on the outside, coming towards us (the oncoming lanes we'd swing into)
            float closing = -CVector2D::Dot((o->m_vecMoveSpeed.To2D() - veh->m_vecMoveSpeed.To2D()) * GAME_SPEED_TO_MPS, d * (1.0f / std::max(d.Magnitude(), 0.1f)));
            bool oncoming = lat > 0.0f && lat < halfWidth + oHalf + 4.0f && lon > 0.0f && lon < 25.0f && closing > 3.0f &&
                CVector2D::Dot(o->GetForward().To2D(), fwd) < -0.3f;
            threat = beside || oncoming;
        }
        if (threat) {
            float maxSteer = plugin::CallAndReturn<float, 0x427FE0, CVehicle*>(veh);
            *steer = std::clamp(*steer - outside * 0.12f, -maxSteer, maxSteer); // towards the inside
            *gas = std::min(*gas, 0.15f);
            if (speed > 6.0f) *brake = std::max(*brake, 0.35f);
            if (CTimer::m_snTimeInMilliseconds - xdata.turnGuardTime > 2000) turnGuardCount++;
            xdata.turnGuardTime = CTimer::m_snTimeInMilliseconds;
        }
    }

    // Handbrake turn: into a tight turn (> ~55 degrees) at the junction, still pointing well away from the exit road and
    // too fast to just steer round: a short pull of the handbrake swings the tail, then back on the gas out of it
    const unsigned int nowMs = CTimer::m_snTimeInMilliseconds;
    bool handbrakeTurn = false;
    // (from ~25 km/h, within 15 m of the junction: tighter than that it fired once in two trips)
    if (xdata.handbrakeCapable && xdata.turnAngle > 0.95f && xdata.turnDist < 15.0f && xdata.turnDist > -15.0f &&
        speed > 7.0f && speed < 30.0f && veh->GetNumContactWheels() == 4 && !(xdata.obstacle != OBSTACLE_NONE && xdata.gap < 12.0f))
    {
        CVector2D fwd = veh->GetForward().To2D();
        float err = std::atan2(CVector2D::Cross(fwd, xdata.turnExitDir), CVector2D::Dot(fwd, xdata.turnExitDir)); // > 0: exit to the left
        bool steeringIn = (err > 0.0f) == (*steer > 0.0f) && std::abs(*steer) > 0.15f;
        if (nowMs >= xdata.handbrakeUntil && nowMs - xdata.handbrakeLast > 4000 && std::abs(err) > 0.75f && steeringIn) {
            xdata.handbrakeUntil = nowMs + static_cast<unsigned int>(Lerp(300.0f, 600.0f, std::clamp((speed - 7.0f) / 14.0f, 0.0f, 1.0f)));
            xdata.handbrakeLast = nowMs;
            handbrakeTurnCount++;
        }
        // Rotated enough: let go
        if (nowMs < xdata.handbrakeUntil && std::abs(err) < 0.45f) xdata.handbrakeUntil = nowMs;
    }
    if (nowMs < xdata.handbrakeUntil) handbrakeTurn = true;

    xdata.aidFlags = 0;
    if (!handbrakeTurn) ApplyCounterSteer(veh, xdata, speed, steer);
    ApplyPedalControl(veh, xdata, speed, gas, brake, handbrake);
    ApplyAccelLimit(veh, xdata, speed, gas, brake);
    ApplyDriverAids(veh, xdata, speed, gas, brake);
    if (lowSpeedBrake) { *gas = 0.0f; *brake = 1.0f; }
    if (handbrakeTurn) {
        *handbrake = true;
        *brake = 0.0f;
        *gas = std::max(*gas, 0.3f);
        float maxSteer = plugin::CallAndReturn<float, 0x427FE0, CVehicle*>(veh); // full lock into the turn
        *steer = std::copysign(maxSteer, *steer);
    }
    xdata.dbgGas = *gas;
    xdata.dbgBrake = *brake;
}

// Route planning: CCarCtrl::PickNextNodeRandomly tries the junction exits in the order given by
// CCarCtrl::InitSequence, starting at SequenceRandomOffset, and takes the first one that passes its checks
// (lanes, one-way, etc). We make it try our planned exit first; if it's not legal the game goes on as usual
// The exit the route (or the junction plan) takes at the junction the vehicle has just reached
static CNodeAddress PlannedExit(CVehicle* veh, ExtendedData& xdata) {
    const CAutoPilot& ap = veh->m_autoPilot;
    if (cfgDestinations) {
        int last = static_cast<int>(xdata.route.size()) - 1;
        for (int i = std::max(xdata.routePos, 1); i < last && i <= xdata.routePos + 8; i++)
            if (xdata.route[i] == ap.m_currentAddress && xdata.route[i - 1] == ap.field_8) return xdata.route[i + 1];
    }
    for (int p = 0; p < xdata.planCount; p++)
        if (xdata.plans[p].junction == ap.m_currentAddress && xdata.plans[p].from == ap.field_8) return xdata.plans[p].exit;
    return CNodeAddress();
}

static void __cdecl InitSequenceForVehicle(int numElements, CVehicle* veh) {
    plugin::Call<0x421740, int>(numElements); // CCarCtrl::InitSequence
    if (!cfgRoutePlanning || !veh) return;
    ExtendedData& xdata = vehExData.Get(veh);
    xdata.forcedExit = CNodeAddress();
    if (CTimer::m_snTimeInMilliseconds - xdata.lastProcessed > 500) return; // not driven by the mod

    const CAutoPilot& ap = veh->m_autoPilot; // already shifted: current = the junction, field_8 = where we came from
    CPathNode* junctionNode = GetNode(ap.m_currentAddress);
    if (!junctionNode || junctionNode->NumberAdjNodes != numElements) return;
    xdata.forcedExit = PlannedExit(veh, xdata);
    auto force = [&](const CNodeAddress& exit) {
        for (int k = 0; k < numElements; k++) {
            if (GetAdjacentNode(junctionNode, ap.m_currentAddress, k) == exit) {
                CCarCtrl::SequenceRandomOffset = k; // first index tried is SequenceRandomOffset % numElements
                seqPlanApplied++;
                return true;
            }
        }
        return false;
    };
    // The route: its node after this junction (when we came from its node before it)
    if (cfgDestinations) {
        int last = static_cast<int>(xdata.route.size()) - 1;
        for (int i = std::max(xdata.routePos, 1); i < last && i <= xdata.routePos + 8; i++) {
            if (xdata.route[i] == ap.m_currentAddress && xdata.route[i - 1] == ap.field_8) { if (force(xdata.route[i + 1])) return; break; }
        }
    }
    for (int p = 0; p < xdata.planCount; p++) {
        if (xdata.plans[p].junction != ap.m_currentAddress || xdata.plans[p].from != ap.field_8) continue;
        CPathNode* junction = GetNode(ap.m_currentAddress);
        if (!junction || junction->NumberAdjNodes != numElements) return;
        for (int k = 0; k < numElements; k++) {
            if (GetAdjacentNode(junction, ap.m_currentAddress, k) == xdata.plans[p].exit) {
                CCarCtrl::SequenceRandomOffset = k; // first index tried is SequenceRandomOffset % numElements
                seqPlanApplied++;
                return;
            }
        }
        return;
    }
    if (numElements > 2) seqNoPlan++;
}

// The game's checks for each exit (0x42DAB0 "appropriate node": "don't wander here", unused roads, special nodes,
// vehicle type...) refused the route's exit and sent the car another way, lost. The route's exit is taken whenever it's
// a legal way to drive: lanes towards it, not a U-turn, not into water. Called instead of 0x42DAB0 at 0x42E18B
static int forcedExitCount = 0;
static bool __cdecl ForceRouteExit(CVehicle* veh, uint32_t candidateRaw) {
    if (!cfgRoutePlanning || !veh) return false;
    ExtendedData& xdata = vehExData.Get(veh);
    if (CTimer::m_snTimeInMilliseconds - xdata.lastProcessed > 500 || xdata.forcedExit.IsEmpty()) return false;
    CNodeAddress candidate;
    memcpy(&candidate, &candidateRaw, sizeof(candidate));
    if (!(candidate == xdata.forcedExit)) return false;
    const CAutoPilot& ap = veh->m_autoPilot;
    CPathNode* junction = GetNode(ap.m_currentAddress);
    CPathNode* exitNode = GetNode(candidate);
    if (!junction || !exitNode || candidate == ap.field_8 || exitNode->WaterNode != junction->WaterNode) return false;
    CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, junction, candidate);
    if (!link || LanesToward(link, candidate) <= 0) return false;
    forcedExitCount++;
    return true;
}
static uintptr_t gameExitCheck = 0x42DAB0, gameExitTaken = 0x42E468;
static void __declspec(naked) ExitCheckHook() {
    __asm {
        push dword ptr [esp + 10h] // the exit tried (4th argument)
        push dword ptr [esp + 8]   // the vehicle (1st argument)
        call ForceRouteExit
        add esp, 8
        test al, al
        jnz taken
        jmp dword ptr [gameExitCheck] // the game's own check, same arguments and return address
    taken:
        add esp, 1Ch                  // return address + the 6 arguments
        jmp dword ptr [gameExitTaken] // where PickNextNodeRandomly goes with the exit it chose
    }
}

// Call @ 0x42E039 inside PickNextNodeRandomly; the vehicle is in esi there
static void __declspec(naked) InitSequenceHook() {
    __asm {
        push esi
        push dword ptr [esp + 8]
        call InitSequenceForVehicle
        add esp, 8
        ret
    }
}

static float CurveSpeedMultiplierHook(float angle, float minAngle, float maxAngle, float minMult) {
    if (followPathVehicle && vehExData.Get(followPathVehicle).valid) return 1.0f;
    return plugin::CallAndReturn<float, 0x4224E0, float, float, float, float>(angle, minAngle, maxAngle, minMult);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Horns

static void __fastcall CustomPlayHornIfNecessary(CAutomobile* _this) {
    if (CGeneral::GetRandomNumberInRange(0, cfgHornsThreshold) == 0) {
        if (_this->m_autoPilot.m_nCarCtrlFlags & 3) {
            if (cfgNoBicycleHorns && _this->m_nVehicleSubClass == VEHICLE_BMX) return;
            if (!_this->HasCarStoppedBecauseOfLight()) _this->PlayCarHorn();
        }
    }
}

struct MoreCustomPlayHornIfNecessary {
    void operator()(injector::reg_pack& regs) {
        if (CGeneral::GetRandomNumberInRange(0, cfgHornsThreshold) == 0) {
            auto bike = reinterpret_cast<CVehicle*>(regs.esi);
            if (cfgNoBicycleHorns && bike->m_nVehicleSubClass == VEHICLE_BMX) return;
            bike->PlayCarHorn();
        }
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// Missing keys keep the default value

static void ReadIniFloat(CIniReader& ini, const char* key, float& value) {
    value = ini.ReadFloat("Settings", key, value);
    lg << key << " = " << std::fixed << value << "\n";
}

static void ReadIniInt(CIniReader& ini, const char* key, int& value) {
    value = ini.ReadInteger("Settings", key, value);
    lg << key << " = " << value << "\n";
}

static void ReadIniBool(CIniReader& ini, const char* key, bool& value) {
    value = ini.ReadInteger("Settings", key, value ? 1 : 0) == 1;
    lg << key << " = " << (value ? "true" : "false") << "\n";
}

// One section per profile, [Profile.Calm] ... [Profile.MaxPerformance]
static void ReadProfiles(CIniReader& ini) {
    for (int i = 0; i < NUM_PROFILES; i++) {
        DriverProfile& p = profiles[i];
        const char* sec = profileSections[i];
        p.chance = std::max(ini.ReadInteger(sec, "Chance", p.chance), 0);
        p.speedMult = std::max(ini.ReadFloat(sec, "SpeedMult", p.speedMult), 0.2f);
        p.accel = std::max(ini.ReadFloat(sec, "Acceleration", p.accel), 0.3f);
        p.decel = std::max(ini.ReadFloat(sec, "Deceleration", p.decel), 0.5f);
        p.timeGap = std::max(ini.ReadFloat(sec, "TimeGap", p.timeGap), 0.2f);
        p.minGap = std::max(ini.ReadFloat(sec, "MinGap", p.minGap), 0.3f);
        p.cornerGrip = std::clamp(ini.ReadFloat(sec, "CornerGrip", p.cornerGrip), 0.1f, 1.5f);
        p.topSpeedUse = std::clamp(ini.ReadFloat(sec, "TopSpeedUse", p.topSpeedUse), 0.2f, 1.0f);
        p.overtakeRatio = std::clamp(ini.ReadFloat(sec, "OvertakeSpeedRatio", p.overtakeRatio), 0.0f, 1.0f);
        p.laneChangeInterval = std::max(ini.ReadFloat(sec, "LaneChangeInterval", p.laneChangeInterval), 1.0f);
        p.aggression = std::clamp(ini.ReadFloat(sec, "Aggression", p.aggression), 0.0f, 1.0f);
        p.keepRight = std::clamp(ini.ReadInteger(sec, "KeepRight", p.keepRight), -1, 2);
        p.lightApproach = ini.ReadFloat(sec, "LightApproachSpeed", p.lightApproach);
        p.racingLogic = ini.ReadInteger(sec, "RacingLogic", p.racingLogic ? 1 : 0) == 1;
        p.ignoreRoadLimit = ini.ReadInteger(sec, "IgnoreRoadSpeed", p.ignoreRoadLimit ? 1 : 0) == 1;
        p.runRedLights = ini.ReadInteger(sec, "RunRedLights", p.runRedLights ? 1 : 0) == 1;
        lg << Format("[%s] chance %d speed x%.2f accel %.1f decel %.1f gap %.1fs/%.1fm grip %.2f top %.2f overtake %.2f "
            "interval %.0fs aggr %.2f keepRight %d light %.0f racing %d ignoreRoad %d redLights %d\n", sec, p.chance, p.speedMult,
            p.accel, p.decel, p.timeGap, p.minGap, p.cornerGrip, p.topSpeedUse, p.overtakeRatio, p.laneChangeInterval, p.aggression,
            p.keepRight, p.lightApproach, p.racingLogic, p.ignoreRoadLimit, p.runRedLights);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

static std::string DriverTypeName(CVehicle* veh) { return profileNames[vehExData.Get(veh).profile]; }

#include "AutoTest.h"

class RealTrafficFix {
public:
    RealTrafficFix() {
        std::string dir = GetModuleDir();
        lg.open(dir + "RealTrafficFix.log", std::fstream::out | std::fstream::trunc);
        lg << MOD_VERSION << std::endl;

        unsigned int gameVersion = GetGameVersion();
        if (gameVersion != GAME_10US_COMPACT && gameVersion != GAME_10US_HOODLUM) {
            lg << "Unsupported game version (" << gameVersion << "). Only GTA SA 1.0 US is supported. Mod disabled.\n";
            lg.flush();
            return;
        }

        CIniReader ini(dir + "RealTrafficFix.ini");
        ReadIniBool(ini, "TestMode", cfgTestMode);
        ReadIniInt(ini, "DebugLevel", cfgDebugLevel);
        cfgDebugLevel = std::clamp(cfgDebugLevel, 1, 2);
        AutoTest::Init(dir);
        ReadIniInt(ini, "DebugToggleKey", cfgDebugToggleKey);
        ReadIniFloat(ini, "DebugDistance", cfgDebugDistance);
        ReadIniBool(ini, "OnlyInNormal", cfgOnlyInNormal);
        ReadIniBool(ini, "FixGroundStuck", cfgFixGroundStuck);
        ReadIniInt(ini, "GroundStuckAttempts", cfgGroundStuckAttempts);
        ReadIniBool(ini, "UseBikeLogicOnCars", cfgUseBikeLogicOnCars);
        ReadIniBool(ini, "UseBikeLogicOnBicycles", cfgUseBikeLogicOnBicycles);
        ReadIniBool(ini, "BicyclesDontStopForRed", cfgBicyclesDontStopForRed);
        ReadIniBool(ini, "NoBicycleHorns", cfgNoBicycleHorns);
        ReadIniBool(ini, "CheckObjects", cfgCheckObjects);
        ReadIniBool(ini, "SlowVehiclesSlowLane", cfgSlowVehiclesSlowLane);
        ReadIniBool(ini, "AvoidVehiclesChangingLane", cfgAvoidVehiclesChangingLane);

        ReadIniFloat(ini, "TurningSpeedDecrease", cfgTurningSpeedDecrease);
        ReadIniFloat(ini, "CruiseMinSpeed", cfgCruiseMinSpeed);
        ReadIniFloat(ini, "CruiseMaxSpeed", cfgCruiseMaxSpeed);
        ReadIniFloat(ini, "FrontMultDist", cfgFrontMultDist);
        ReadIniFloat(ini, "SidesSpeedOffsetDiv", cfgSidesSpeedOffsetDiv);
        ReadIniFloat(ini, "CheckGroundHeight", cfgCheckGroundHeight);
        ReadIniFloat(ini, "FinalGroundHeight", cfgFinalGroundHeight);
        ReadIniFloat(ini, "HeightDiffLimit", cfgHeightDiffLimit);
        ReadIniInt(ini, "HornsThreshold", cfgHornsThreshold);
        ReadIniFloat(ini, "BicycleSpeedMult", cfgBicycleSpeedMult);
        ReadIniFloat(ini, "BicycleCruiseMaxSpeed", cfgBicycleCruiseMaxSpeed);
        ReadIniFloat(ini, "BumpSearchRadius", cfgBumpSearchRadius);
        ReadIniFloat(ini, "BumpSearchDistMult", cfgBumpSearchDistMult);
        ReadIniFloat(ini, "BumpSpeedDiv", cfgBumpSpeedDiv);
        ReadIniInt(ini, "BumpModelStart", cfgBumpModelStart);
        ReadIniInt(ini, "BumpModelEnd", cfgBumpModelEnd);
        ReadIniInt(ini, "SearchBumpFramesDelay", cfgSearchBumpFramesDelay);

        ReadIniInt(ini, "SensorInterval", cfgSensorInterval);
        ReadIniInt(ini, "SensorIntervalFar", cfgSensorIntervalFar);
        ReadIniFloat(ini, "SensorFarDistance", cfgSensorFarDistance);


        ReadIniBool(ini, "ReplaceGameCurveBraking", cfgReplaceGameCurveBraking);
        ReadIniBool(ini, "HeadingLimiter", cfgHeadingLimiter);
        ReadIniBool(ini, "TrafficLightDetection", cfgTrafficLightDetection);
        ReadIniFloat(ini, "LightStopMargin", cfgLightStopMargin);
        ReadIniFloat(ini, "BikeGapExtra", cfgBikeGapExtra);
        ReadIniFloat(ini, "HeadingGripMargin", cfgHeadingGripMargin);
        ReadIniFloat(ini, "PassRequestMinDiff", cfgPassRequestMinDiff);
        ReadIniBool(ini, "HandbrakeTurns", cfgHandbrakeTurns);
        ReadIniFloat(ini, "RejoinRouteFactor", cfgRejoinRouteFactor);
        ReadIniBool(ini, "RouteFixes", cfgRouteFixes);
        ReadIniBool(ini, "AmbientSpawn", cfgAmbientSpawn);
        ReadIniFloat(ini, "AmbientSpawnMinDist", cfgAmbientMinDist);
        ReadIniFloat(ini, "AmbientSpawnMaxDist", cfgAmbientMaxDist);
        ReadIniFloat(ini, "AmbientSpawnFill", cfgAmbientFill);
        cfgAmbientTarget = ini.ReadInteger("Settings", "AmbientSpawnTarget", cfgAmbientTarget);
        ReadIniFloat(ini, "AmbientLaneSpacing", cfgAmbientLaneSpacing);
        ReadIniFloat(ini, "AmbientKeepDist", cfgAmbientKeepDist);
        ReadIniFloat(ini, "DespawnOffScreen", cfgDespawnOffScreen);
        ReadIniFloat(ini, "DespawnOnScreen", cfgDespawnOnScreen);
        cfgAmbientInterval = ini.ReadInteger("Settings", "AmbientSpawnInterval", cfgAmbientInterval);
        ReadIniFloat(ini, "HandbrakeEntryBoost", cfgHandbrakeEntryBoost);
        ReadIniFloat(ini, "JunctionGripFactor", cfgJunctionGripFactor);
        ReadIniBool(ini, "TurnSideGuard", cfgTurnSideGuard);
        ReadIniBool(ini, "V2VOvertakes", cfgV2VOvertakes);
        ReadIniBool(ini, "JunctionFirstCome", cfgJunctionFirstCome);
        ReadIniFloat(ini, "UnplannedJunctionSpeed", cfgUnplannedJunctionSpeed);
        ReadIniFloat(ini, "CurveGripUseMin", cfgCurveUseMin);
        ReadIniFloat(ini, "CurveGripUseMax", cfgCurveUseMax);
        ReadIniFloat(ini, "CurveBrakeUseMin", cfgCurveBrakeUseMin);
        ReadIniFloat(ini, "CurveBrakeUseMax", cfgCurveBrakeUseMax);

        ReadIniFloat(ini, "SensorHeight", cfgSensorHeight);
        ReadIniFloat(ini, "WallNormalZ", cfgWallNormalZ);
        ReadIniBool(ini, "RoutePlanning", cfgRoutePlanning);
        ReadIniBool(ini, "RacingLine", cfgRacingLine);
        ReadIniFloat(ini, "RacingLineMargin", cfgRacingLineMargin);
        ReadIniBool(ini, "AdaptiveLimits", cfgAdaptiveLimits);
        ReadIniFloat(ini, "LearnedDecelUse", cfgLearnedDecelUse);
        ReadIniBool(ini, "ABS", cfgABS);
        ReadIniBool(ini, "TractionControl", cfgTractionControl);
        ReadIniBool(ini, "UseUnusedRoads", cfgUseUnusedRoads);
        ReadIniFloat(ini, "FastCurveGripBoost", cfgFastCurveGripBoost);
        ReadIniFloat(ini, "HighwayGapFactor", cfgHighwayGapFactor);
        ReadIniBool(ini, "StabilityControl", cfgStabilityControl);
        ReadIniBool(ini, "CounterSteer", cfgCounterSteer);
        ReadIniBool(ini, "PedalControl", cfgPedalControl);
        ReadIniBool(ini, "FrictionCircle", cfgFrictionCircle);
        ReadIniBool(ini, "SlopeAware", cfgSlopeAware);
        ReadIniFloat(ini, "CrestFactor", cfgCrestFactor);
        ReadIniBool(ini, "SurfaceGrip", cfgSurfaceGrip);
        ReadIniFloat(ini, "RainGripLoss", cfgRainGripLoss);
        ReadIniBool(ini, "DamageAware", cfgDamageAware);
        ReadIniBool(ini, "Profiles", cfgProfiles);
        ReadIniFloat(ini, "PersonalitySpread", cfgPersonalitySpread);
        ReadIniBool(ini, "JunctionProtection", cfgJunctionProtection);
        ReadIniBool(ini, "CrashRecovery", cfgCrashRecovery);
        ReadIniFloat(ini, "StuckTime", cfgStuckTime);
        ReadIniFloat(ini, "FlipResetTime", cfgFlipResetTime);

        ReadIniBool(ini, "CurveLookahead", cfgCurveLookahead);
        ReadIniFloat(ini, "MaxLookaheadDist", cfgMaxLookaheadDist);

        ReadIniFloat(ini, "RacerMaxSpeed", cfgRacerMaxSpeed);
        ReadIniFloat(ini, "RacerJunctionSpeed", cfgRacerJunctionSpeed);
        ReadIniFloat(ini, "RacerSideRayLength", cfgRacerSideRayLength);
        ReadIniInt(ini, "RacerDrivingStyle", cfgRacerDrivingStyle);
        ReadIniInt(ini, "RacerBikeDrivingStyle", cfgRacerBikeDrivingStyle);
        ReadIniBool(ini, "Overtaking", cfgOvertaking);
        ReadIniFloat(ini, "OvertakeRearTime", cfgOvertakeRearTime);
        ReadIniBool(ini, "KeepRight", cfgKeepRight);
        ReadIniBool(ini, "LaneDiscipline", cfgLaneDiscipline);
        ReadIniBool(ini, "LaneChangeCoordination", cfgLaneBoard);
        ReadIniBool(ini, "Destinations", cfgDestinations);
        ReadIniBool(ini, "TurnSignals", cfgTurnSignals);
        ReadIniBool(ini, "YieldToSignals", cfgYieldToSignals);
        ReadIniFloat(ini, "LanePrepareTime", cfgLanePrepareTime);
        ReadIniInt(ini, "DestinationWideRoads", cfgDestinationWideRoads);
        ReadIniBool(ini, "LoadAllPathNodes", cfgLoadAllPathNodes);
        ReadIniBool(ini, "BypassBlocked", cfgBypassBlocked);
        ReadIniFloat(ini, "BypassWait", cfgBypassWait);
        ReadIniFloat(ini, "BypassSpeed", cfgBypassSpeed);
        ReadIniBool(ini, "RoadSpeedLimits", cfgRoadSpeedLimits);
        ReadIniFloat(ini, "RoadSpeedHighway", cfgRoadSpeedHighway);
        ReadIniFloat(ini, "RoadSpeedFastRoad", cfgRoadSpeedFastRoad);
        ReadIniFloat(ini, "RoadSpeedAvenue", cfgRoadSpeedAvenue);
        ReadIniFloat(ini, "RoadSpeedRural", cfgRoadSpeedRural);
        ReadIniFloat(ini, "RuralJunctionDist", cfgRuralJunctionDist);
        ReadIniFloat(ini, "RoadSpeedStreet", cfgRoadSpeedStreet);
        ReadIniFloat(ini, "RoadSpeedNarrow", cfgRoadSpeedNarrow);
        ReadIniFloat(ini, "RoadSpeedDirt", cfgRoadSpeedDirt);
        ReadIniFloat(ini, "RoadSpeedBigMult", cfgRoadSpeedBigMult);
        ReadIniFloat(ini, "AccelHighway", cfgAccelHighway);
        ReadIniFloat(ini, "AccelFastRoad", cfgAccelFastRoad);
        ReadIniFloat(ini, "AccelRural", cfgAccelRural);
        ReadIniFloat(ini, "AccelAvenue", cfgAccelAvenue);
        ReadIniFloat(ini, "AccelNarrow", cfgAccelNarrow);
        ReadIniFloat(ini, "AccelDirt", cfgAccelDirt);
        ReadIniBool(ini, "OvertakeBoost", cfgOvertakeBoost);
        ReadIniFloat(ini, "FastRoadCornerGrip", cfgFastRoadCornerGrip);
        ReadIniBool(ini, "GreenLightFullSpeed", cfgGreenLightFullSpeed);
        ReadIniFloat(ini, "OvertakeSpeedBoost", cfgOvertakeSpeedBoost);
        ReadIniBool(ini, "OvertakeFullThrottle", cfgOvertakeFullThrottle);
        ReadIniBool(ini, "CloseUpInQueues", cfgCloseUpInQueues);
        float fullGapKmh = cfgFullGapSpeed * MPS_TO_KMH;
        ReadIniFloat(ini, "FullGapSpeed", fullGapKmh);
        cfgFullGapSpeed = std::max(fullGapKmh / MPS_TO_KMH, 1.0f);
        ReadIniFloat(ini, "LowSpeedGapFactor", cfgMinGapFactor);
        ReadIniFloat(ini, "OvertakeAccelMult", cfgOvertakeAccelMult);
        ReadIniFloat(ini, "OvertakeMinAccel", cfgOvertakeMinAccel);
        ReadProfiles(ini);
        lg.flush();

        cfgHornsThreshold = std::max(cfgHornsThreshold, 1);
        cfgSensorInterval = std::max(cfgSensorInterval, 1);
        cfgSensorIntervalFar = std::max(cfgSensorIntervalFar, 1);

        // Horns
        patch::RedirectCall(0x6B5275, CustomPlayHornIfNecessary, true);
        patch::RedirectCall(0x6B52B8, CustomPlayHornIfNecessary, true);
        patch::RedirectCall(0x6C1C22, CustomPlayHornIfNecessary, true);
        patch::RedirectCall(0x6C1C6B, CustomPlayHornIfNecessary, true);
        injector::MakeInline<MoreCustomPlayHornIfNecessary>(0x6BCD3A, 0x6BCD3A + 6);
        injector::MakeInline<MoreCustomPlayHornIfNecessary>(0x6BCD97, 0x6BCD97 + 6);
        injector::MakeInline<MoreCustomPlayHornIfNecessary>(0x41FD30, 0x41FD30 + 6);

        // CCarCtrl::PickNextNodeRandomly changes lane at random every 4-7 nodes (countdown at CAutoPilot +0x50, 0x42E95C),
        // without looking at the lane it goes to. Vehicles driven by the mod only change lanes when the mod decides
        if (cfgAvoidVehiclesChangingLane || cfgLaneDiscipline) {
            injector::MakeInline<0x0042E98B, 0x0042E98B + 6>([](injector::reg_pack& regs) {
                auto veh = reinterpret_cast<CVehicle*>(regs.esi);
                if (cfgLaneDiscipline && CTimer::m_snTimeInMilliseconds - vehExData.Get(veh).lastProcessed < 500) return;
                if (cfgAvoidVehiclesChangingLane && playerPed && playerPed->m_pVehicle) {
                    float playerCarSpeed = playerPed->m_pVehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH;
                    if (playerCarSpeed > 80.0f || veh->GetPosition().Distance(playerPed->m_pVehicle->GetPosition()) <= 20.0f) {
                        return;
                    }
                }
                veh->m_autoPilot.m_nNextLane = static_cast<char>(regs.eax); // mov [esi+3B8h], al
            });
        }

        // Lane of a vehicle being created (CCarCtrl::GenerateOneRandomCar: random lane = edx, number of lanes = ecx)
        if (cfgSlowVehiclesSlowLane) {
            injector::MakeInline<0x004308C9, 0x004308C9 + 12>([](injector::reg_pack& regs) {
                auto veh = reinterpret_cast<CVehicle*>(regs.esi);
                if (IsBigOrSlow(veh) && !IsEmergency(veh)) {
                    // Rightmost lane; flag 0x10 keeps it there at every node (0x42E9CB)
                    veh->m_autoPilot.m_nCurrentLane = static_cast<char>(std::max(static_cast<int>(regs.ecx) - 1, 0));
                    veh->m_autoPilot.m_nNextLane = veh->m_autoPilot.m_nCurrentLane;
                    veh->m_autoPilot.m_nCarCtrlFlags |= 0x10;
                }
                else {
                    veh->m_autoPilot.m_nCurrentLane = static_cast<char>(regs.edx);
                    veh->m_autoPilot.m_nNextLane = static_cast<char>(regs.edx);
                }
            });
        }

        if (cfgReplaceGameCurveBraking || cfgRacingLine || cfgABS || cfgTractionControl || cfgStabilityControl) {
            patch::RedirectCall(0x4374D8, FollowPathHook, true);
        }
        if (cfgReplaceGameCurveBraking) {
            patch::RedirectCall(0x435461, CurveSpeedMultiplierHook, true);
            patch::RedirectCall(0x4354A8, CurveSpeedMultiplierHook, true);
        }
        if (cfgRoutePlanning) {
            patch::RedirectCall(0x42E039, InitSequenceHook, true);
            // The exit checks: only patched where the game's code is what we expect (call 0x42DAB0)
            if (*reinterpret_cast<unsigned char*>(0x42E18B) == 0xE8 && 0x42E190 + *reinterpret_cast<int*>(0x42E18C) == 0x42DAB0)
                patch::RedirectCall(0x42E18B, ExitCheckHook, true);
            else lg << "PickNextNodeRandomly: unexpected code at 0x42E18B, the route's exits are not forced\n";
        }

        // The game only keeps the road nodes around the player loaded (CPathFind::UpdateStreaming @ 0x450A60: clears
        // ToBeStreamed[8][8] @ 0x96EFD0, marks the areas near the player, then loads/unloads by reading it at 0x450D00).
        // Making that read see "all areas" keeps the whole map's nodes loaded, so a route can go anywhere (the same
        // idea as GTA GPS Redux by juicermv). Only if the instruction is the expected one: mov al, [esi+ebp*8+96EFD0h]
        // How far random cars are kept (CCarCtrl::PossiblyRemoveVehicle @ 0x424F80): out of sight 45 m, an immediate in
        // "mov [esp+34h], 45.0" @ 0x4250DE; in sight 170 m x the generation multiplier, "fmul [858F98h]" @ 0x4250EE
        // (that constant is shared: the instruction is pointed at our own). Cars created around the player out of sight
        // were removed as soon as they were made
        if (cfgDespawnOffScreen > 0.0f) {
            const unsigned char offExpected[] = { 0xC7, 0x44, 0x24, 0x34, 0x00, 0x00, 0x34, 0x42 };
            const unsigned char onExpected[] = { 0xD8, 0x0D, 0x98, 0x8F, 0x85, 0x00 };
            if (memcmp(reinterpret_cast<void*>(0x4250DE), offExpected, sizeof(offExpected)) == 0 &&
                memcmp(reinterpret_cast<void*>(0x4250EE), onExpected, sizeof(onExpected)) == 0) {
                patch::SetFloat(0x4250E2, cfgDespawnOffScreen);
                patch::SetPointer(0x4250F0, &cfgDespawnOnScreen);
                lg << "Vehicle despawn distances: " << cfgDespawnOffScreen << " m out of sight, " << cfgDespawnOnScreen << " m in sight\n";
            }
            else lg << "Despawn distances: unexpected code at 0x4250DE/0x4250EE (another mod?), not patched\n";
        }

        if (cfgLoadAllPathNodes) {
            const unsigned char expected[] = { 0x8A, 0x84, 0xEE, 0xD0, 0xEF, 0x96, 0x00 };
            if (memcmp(reinterpret_cast<void*>(0x450D00), expected, sizeof(expected)) == 0) {
                memset(allPathAreas, 1, sizeof(allPathAreas));
                patch::SetPointer(0x450D03, allPathAreas);
                lg << "All path node areas kept loaded" << std::endl;
            }
            else lg << "LoadAllPathNodes: unexpected code at 0x450D00, not patched\n";
        }

        if (cfgRouteFixes) LoadRouteFixes(dir + "RealTrafficFix.routefixes.txt");
        learnedFile = dir + "RealTrafficFix.limits.txt"; // v2 measurement (the old .learned.txt values were inflated);
        if (cfgAdaptiveLimits) LoadLearnedLimits();

        // Runs once per game frame, before CWorld::Process (where the AI reads the cruise speed),
        // for every vehicle, not only the ones being rendered
        Events::processScriptsEvent.after += [] {
            playerPed = FindPlayerPed();
            frameCounter++;
            BreakDeadlocks();
            RemoveBrokenDown();
            RemoveBadSpawns();
            // MixSets writes its own out-of-sight despawn distance (0.6 x VehDespawnOffScr) after us: set ours again once
            // every mod has loaded. Only the opcode is checked ("mov dword ptr [esp+34h], imm32")
            if (frameCounter == 60 && cfgDespawnOffScreen > 0.0f) {
                const unsigned char op[] = { 0xC7, 0x44, 0x24, 0x34 };
                if (memcmp(reinterpret_cast<void*>(0x4250DE), op, sizeof(op)) == 0) {
                    float was = *reinterpret_cast<float*>(0x4250E2);
                    patch::SetFloat(0x4250E2, cfgDespawnOffScreen);
                    lg << "Out-of-sight despawn distance " << was << " m (set by another mod) -> " << cfgDespawnOffScreen << " m\n";
                }
            }
            // Who removes the random cars: the code as it is once every mod has loaded (another one may hook it)
            if (frameCounter == 300) {
                auto dump = [](const char* what, unsigned int addr, int n) {
                    lg << what << " @ 0x" << std::hex << addr << ":";
                    for (int i = 0; i < n; i++) lg << " " << std::setw(2) << std::setfill('0') << (int)*reinterpret_cast<unsigned char*>(addr + i);
                    lg << std::dec << std::setfill(' ') << "\n";
                };
                dump("PossiblyRemoveVehicle", 0x424F80, 8);
                dump("off-screen despawn", 0x4250DE, 8);
                dump("on-screen despawn", 0x4250EE, 6);
                dump("RemoveDistantCars", 0x42CD10, 8);
                dump("RemoveDistantCars call site scan", 0x42CD10, 64);
            }
            // The game's random car limit comes from its population cycle (~17 here, MixSets' VehLimit didn't apply): over
            // it, it thins the random cars out. Room for the ambient traffic
            if (cfgAmbientSpawn && CCarCtrl::MaxNumberOfCarsInUse < static_cast<unsigned int>(cfgAmbientTarget + 30))
                CCarCtrl::MaxNumberOfCarsInUse = cfgAmbientTarget + 30;
            UpdateAmbientSpawn();
            if (cfgRouteFixes && frameCounter % 120 == 1) ApplyRouteFixes(); // ~2 s, and right after the road data loads
            UpdatePassMeasures();
            if (frameCounter % 3000 == 0) SaveLearnedLimits(); // ~1 min

            if (checkBumps == -1) {
                checkBumps = 0;
                if (cfgBumpModelStart > 0 && cfgBumpModelEnd > cfgBumpModelStart && cfgBumpSearchRadius > 0.0f &&
                    cfgBumpSearchDistMult > 0.0f && cfgBumpSpeedDiv > 0.0f)
                {
                    if (CStreaming::IsObjectInCdImage(cfgBumpModelStart) && CStreaming::IsObjectInCdImage(cfgBumpModelEnd)) {
                        checkBumps = 1;
                        lg << "Bumps is installed and will be checked\n";
                        lg.flush();
                    }
                }
            }

            // Debug toggle (edge triggered)
            static bool keyWasDown = false;
            bool keyDown = cfgDebugToggleKey > 0 && KeyPressed(cfgDebugToggleKey);
            if (keyDown && !keyWasDown) {
                // off -> compact -> full -> off
                if (!cfgTestMode) { cfgTestMode = true; cfgDebugLevel = 1; }
                else if (cfgDebugLevel == 1) cfgDebugLevel = 2;
                else cfgTestMode = false;
                lg << "Debug " << (cfgTestMode ? (cfgDebugLevel == 1 ? "compact" : "full") : "off") << "\n";
                lg.flush();
            }
            keyWasDown = keyDown;

            debugVehiclesProcessed = 0;
            debugRacers = 0;
            routesThisFrame = 0;
            for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
                if (CVehicle* vehicle = CPools::ms_pVehiclePool->GetAt(i)) {
                    ProcessVehicle(vehicle, i);
                }
            }
            DrawIndicators();
            AutoTest::Update();
        };

        Events::drawingEvent += DrawDebug;
        Events::drawingEvent += AutoTest::Draw;

        Events::onPauseAllSounds += [] {
            lg.flush();
            SaveLearnedLimits();
        };
    }

    static void ProcessVehicle(CVehicle* vehicle, int poolIndex) {
        ExtendedData& xdata = vehExData.Get(vehicle);
        xdata.valid = false;
        xdata.accelLimit = 0.0f; // set again below if the mod drives it this frame

        if (!vehicle->m_pDriver || (vehicle->m_nCreatedBy == MISSION_VEHICLE && !xdata.forceDrive) || vehicle->m_nStatus == STATUS_WRECKED) return;
        bool isPlayerDriving = vehicle->m_pDriver->IsPlayer();
        if (isPlayerDriving && !playerAutoDrive) return;
        if (AutoTest::vanilla) return; // A/B test: the game drives

        int subClass = vehicle->m_nVehicleSubClass;
        if (subClass != VEHICLE_AUTOMOBILE && subClass != VEHICLE_BIKE && subClass != VEHICLE_BMX &&
            subClass != VEHICLE_MTRUCK && subClass != VEHICLE_QUAD)
        {
            return;
        }

        InitVehicle(xdata, vehicle);
        if (!xdata.spawnChecked && xdata.bInitialized) { xdata.spawnChecked = true; MatchSpawnSpeed(vehicle); }
        UpdateDestination(xdata, vehicle);
        {
            UInt16 nextLink = *reinterpret_cast<const UInt16*>(&vehicle->m_autoPilot.m_nNextPathNodeInfo);
            if (nextLink != xdata.lastNextLink) {
                bool first = xdata.lastNextLink == 0xFFFF;
                xdata.lastNextLink = nextLink;
                xdata.newLinkTime = CTimer::m_snTimeInMilliseconds;
                if (!first && cfgLaneDiscipline && subClass != VEHICLE_BIKE && subClass != VEHICLE_BMX && !(vehicle->m_autoPilot.m_nCarCtrlFlags & 0x18)) OnNewSegment(vehicle, xdata);
            }
        }
        xdata.lastProcessed = CTimer::m_snTimeInMilliseconds;
        const bool racer = xdata.bRacer;
        const bool isBike = (subClass == VEHICLE_BIKE || subClass == VEHICLE_BMX);
        CAutoPilot& ap = vehicle->m_autoPilot;
        const float aggr = xdata.aggression;

        // Use real physics, once the ground under it is there: a car created where the collision isn't loaded yet fell
        // through it (from the stacked freeways of Mulholland to the street below)
        if (!xdata.bPhysics && !isPlayerDriving &&
            (subClass == VEHICLE_AUTOMOBILE || subClass == VEHICLE_BMX || subClass == VEHICLE_MTRUCK))
        {
            const CVector pos = vehicle->GetPosition();
            bool groundFound = false;
            float groundZ = FindGroundZ({ pos.x, pos.y, pos.z + 2.5f }, &groundFound);
            if (groundFound && pos.z - groundZ < 3.0f && pos.z - groundZ > -2.5f) {
                // Sunk into the road (created at the path node's height, or the ground streamed in after it): put it on its
                // wheels first. Switched to real physics as it was, it stayed stuck in the tarmac
                if (subClass == VEHICLE_AUTOMOBILE || subClass == VEHICLE_MTRUCK) {
                    float base = GetVehicleBaseZ(vehicle);
                    if (base < groundZ - 0.15f) {
                        reinterpret_cast<CAutomobile*>(vehicle)->PlaceOnRoadProperly();
                        sunkSpawnFixCount++;
                    }
                }
                CCarCtrl::SwitchVehicleToRealPhysics(vehicle);
                xdata.bPhysics = true;
            }
        }

        float distanceToCam = TheCamera.GetPosition().Distance(vehicle->GetPosition());

        CVector modelMin, modelMax;
        GetEntityDimensions(vehicle, modelMin, modelMax);

        if (xdata.bRunGroundStuckFix && cfgFixGroundStuck && !isPlayerDriving) {
            if ((vehicle->m_fMovingSpeed * (CTimer::ms_fTimeStep * MAGIC)) < 0.5f) {
                FixGroundStuck(vehicle, modelMin, modelMax);
            }
        }

        const float dt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
        if (cfgCrashRecovery && !isPlayerDriving && !ap.m_pTargetCar) UpdateCrashRecovery(vehicle, xdata, isBike, dt);

        // What the tyres are on, and how damaged the vehicle is
        float ground = 1.0f;
        xdata.surfaceFactor = cfgSurfaceGrip ? GetSurfaceGripFactor(vehicle, ground) : 1.0f;
        xdata.groundGrip += (ground - xdata.groundGrip) * std::min(0.05f * CTimer::ms_fTimeStep, 1.0f);
        float damageGrip = 1.0f;
        xdata.damageFactor = cfgDamageAware ? GetDamageFactor(vehicle, damageGrip) : 1.0f;

        int interval = distanceToCam > cfgSensorFarDistance ? cfgSensorIntervalFar : cfgSensorInterval;
        bool sensorTick = (frameCounter + poolIndex) % interval == 0;

        const DriverProfile& prof = profiles[xdata.profile];

        // Road type and what this vehicle can do: the max speed is the lower of both
        if (sensorTick || !xdata.bRoadChecked) {
            xdata.bRoadChecked = true;
            xdata.roadType = cfgRoadSpeedLimits ? GetRoadType(vehicle, xdata.groundGrip, xdata.distToJunctionAhead) : ROAD_UNKNOWN;
            float roadSpeed = cfgRoadSpeedLimits ? GetRoadSpeed(xdata.roadType) : cfgCruiseMaxSpeed / MPS_TO_CRUISE;
            if (roadSpeed < 1000.0f && IsBigOrSlow(vehicle)) roadSpeed *= cfgRoadSpeedBigMult;
            xdata.roadLimit = roadSpeed;
            // Gearbox top speed (the real one may be a bit lower because of drag); calm drivers don't push the engine
            xdata.carTopSpeed = GetVehicleTopSpeed(vehicle) * prof.topSpeedUse;
        }

        // Driver behaviour: its profile, with its own small variation. It wants the road's speed times the profile's
        // multiplier (calm drivers below it, aggressive ones above), never more than the car can do
        float desired; // m/s
        eSpeedLimiter desiredLimiter = LIMIT_ROAD;
        if (prof.ignoreRoadLimit || xdata.roadLimit >= 1000.0f) {
            desired = cfgRacerMaxSpeed / MPS_TO_KMH;
            desiredLimiter = LIMIT_DESIRED;
        }
        else desired = xdata.roadLimit * prof.speedMult * xdata.speedVariation;
        if (xdata.carTopSpeed < desired) { desired = xdata.carTopSpeed; desiredLimiter = LIMIT_CAR_TOP; }
        // Drivers accelerate harder on fast roads (merging onto a freeway, out of a curve) than in streets and alleys
        const float roadAccel = RoadAccelFactor(xdata.roadType);
        float accel = std::min(prof.accel * roadAccel, FULL_THROTTLE_ACCEL); // IDM
        float decel = prof.decel;
        // The gap to the vehicle ahead shrinks at low speed (stop-and-go: cars close up), full above ~55 km/h
        const float speedNow = vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
        const float timeGap = prof.timeGap * std::clamp(speedNow / cfgFullGapSpeed, cfgMinGapFactor, 1.0f);
        // Behind a motorbike or bicycle more room: they wobble at low speed, stop short and fall over, and cars
        // closing up to the car gap touched them at lights
        float minGap = prof.minGap;
        if (xdata.obstacle == OBSTACLE_VEHICLE && xdata.leaderVeh && CPools::ms_pVehiclePool->IsObjectValid(xdata.leaderVeh) &&
            (xdata.leaderVeh->m_nVehicleSubClass == VEHICLE_BIKE || xdata.leaderVeh->m_nVehicleSubClass == VEHICLE_BMX))
            minGap += cfgBikeGapExtra;
        // Pedal control keeps the acceleration at the profile's (0 = no limit, full throttle)
        xdata.accelLimit = prof.accel < FULL_THROTTLE_ACCEL ? prof.accel * roadAccel * xdata.speedVariation : 0.0f;
        // Overtaking: get it done quickly. Faster than it wanted (the road's limit too) and harder on the gas, as long
        // as the car can
        const bool overtaking = xdata.laneState == LANE_SIGNAL_OVERTAKE || xdata.laneState == LANE_OVERTAKING ||
            (xdata.laneState == LANE_CHANGING && xdata.changeAfter == LANE_OVERTAKING);
        if (overtaking && cfgOvertakeBoost) {
            // Passing: no road limit, no profile share of the top speed: all the car has. Curves, the one ahead and the
            // rest still limit it further on
            float carLimit = GetVehicleTopSpeed(vehicle);
            if (carLimit > desired) { desired = carLimit; desiredLimiter = LIMIT_OVERTAKE; }
            // Flooring it to pass: everything the car has (no profile limit) until the overtake is done
            if (cfgOvertakeFullThrottle) {
                accel = FULL_THROTTLE_ACCEL;
                xdata.accelLimit = 0.0f;
            }
            else {
                accel = std::min(std::max(accel * cfgOvertakeAccelMult, cfgOvertakeMinAccel), FULL_THROTTLE_ACCEL);
                if (xdata.accelLimit > 0.0f) xdata.accelLimit = std::max(xdata.accelLimit * cfgOvertakeAccelMult, cfgOvertakeMinAccel);
            }
        }
        bool damageLimited = xdata.damageFactor < 1.0f;
        if (damageLimited) desiredLimiter = LIMIT_DAMAGE;
        desired *= xdata.damageFactor;
        desired = std::max(desired, cfgCruiseMinSpeed / MPS_TO_CRUISE);
        if (xdata.speedCap > 0.0f) desired = std::min(desired, xdata.speedCap);
        xdata.desiredSpeed = desired;
        // Queue: the vehicle ahead is stopped a few metres away. The game's own logic stops ~5 m behind it; the driver
        // creeps up to its minimum gap instead (slower and slower), see ApplyCloseUp
        xdata.closeUpTarget = -1.0f;
        if (cfgCloseUpInQueues && xdata.obstacle == OBSTACLE_VEHICLE && xdata.leaderSpeed < 1.0f && xdata.gap < 12.0f) {
            float room = xdata.gap - minGap - 0.3f;
            // At most 2.5 m/s: at 4 m/s a car creeping up behind one that started and stopped again bumped into it
            xdata.closeUpTarget = room > 0.0f ? std::min(std::sqrt(2.0f * 1.2f * room), 2.5f) : 0.0f;
        }

        float traction = std::clamp(vehicle->m_pHandlingData->m_fTractionMultiplier, 0.5f, 1.5f);
        float grip = prof.cornerGrip * traction;
        // Freeways and fast roads are built for speed (wide, banked, long curves): drivers take their curves harder
        if (!racer && (xdata.roadType == ROAD_HIGHWAY || xdata.roadType == ROAD_FAST || xdata.roadType == ROAD_RURAL)) grip *= cfgFastRoadCornerGrip;
        if (racer && cfgAdaptiveLimits) {
            // Drive at what this model has shown it can do (the profile's grip is the fraction of it that's used)
            const ModelLimits& limits = GetModelLimits(vehicle);
            grip = limits.grip * std::min(prof.cornerGrip, 0.98f);
            decel = limits.decel * std::min(cfgLearnedDecelUse * Lerp(0.92f, 1.08f, aggr), 0.95f);
        }
        // Dirt, grass, rain and burst tyres
        grip *= xdata.surfaceFactor * damageGrip;
        decel *= xdata.surfaceFactor * damageGrip;
        // Curves and the braking before them: a share of what this car can really do (learned per model), not only the
        // profile's fixed g. With 0.4-0.6 g and 2-3 m/s2 cars crawled through curves they can take far faster and
        // started braking for them far too early
        float curveDecel = decel;
        if (!racer && cfgAdaptiveLimits) {
            const ModelLimits& limits = GetModelLimits(vehicle);
            const bool fastRoad = xdata.roadType == ROAD_HIGHWAY || xdata.roadType == ROAD_FAST || xdata.roadType == ROAD_RURAL;
            float use = std::min(Lerp(cfgCurveUseMin, cfgCurveUseMax, aggr) * (fastRoad ? 1.1f : 1.0f), 0.92f);
            grip = std::max(grip, limits.grip * use * xdata.surfaceFactor * damageGrip);
            curveDecel = std::max(decel, limits.decel * Lerp(cfgCurveBrakeUseMin, cfgCurveBrakeUseMax, aggr) * xdata.surfaceFactor * damageGrip);
        }

        // Friction circle: the tyres' grip is shared between turning and braking. While cornering hard there's
        // less left to brake with, so the braking points move earlier
        if (cfgFrictionCircle) {
            float latG = std::abs(vehicle->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS) * (vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS) / GRAVITY;
            float usage = std::min(latG / std::max(grip, 0.1f), 1.0f);
            decel *= std::sqrt(std::max(1.0f - usage * usage, 0.2f));
        }
        const float halfWidth = std::max(modelMax.x, -modelMin.x);
        auto refreshRoute = [&](float speed) {
            if (!cfgCurveLookahead && !cfgTrafficLightDetection) return;
            // Drivers that run red lights (fleeing) still see them: through a red one at RED_LIGHT_RUN_SPEED at most
            bool obeysLights = cfgTrafficLightDetection && (DrivingStyleObeysLights(ap.m_nCarDrivingStyle) || prof.runRedLights);
            float lightApproach = obeysLights ? prof.lightApproach / MPS_TO_KMH : 0.0f;
            xdata.runsRedLights = prof.runRedLights;
            UpdateRouteLimits(vehicle, xdata, speed, curveDecel, grip, (racer ? cfgRacerJunctionSpeed : cfgUnplannedJunctionSpeed) / MPS_TO_KMH, lightApproach,
                desired, halfWidth);
        };

        // Chasing/ramming/blocking someone: that's the game's mission logic, leave it alone. Before anything else: setting
        // the cruise speed of a stopped police car in a block mission made it reverse into walls over and over
        if (ap.m_pTargetCar) { xdata.accelLimit = 0.0f; return; }

        // Is valid driving
        bool moving = (vehicle->m_fMovingSpeed * (CTimer::ms_fTimeStep * MAGIC)) > 0.01f &&
            (isBike || vehicle->GetNumContactWheels() > 2);
        if (!moving) {
            // Stopped (lights, jam, parked...): the game decides when to go, we only set how fast.
            // Keep watching the light we stopped for, so we go as soon as it turns green
            xdata.headingSpeed = 1000.0f;
            if (sensorTick) {
                refreshRoute(0.0f);
                if (xdata.turnSide != 0 && xdata.turnDist < 40.0f) SetIndicator(xdata, xdata.turnSide, 400);
            }
            float target = std::min(desired, xdata.curveSpeed);
            bool holdForLight = cfgTrafficLightDetection && xdata.lightColor != LIGHT_NONE && xdata.lightColor != LIGHT_GREEN &&
                xdata.lightSpeed < target;
            if (holdForLight) target = xdata.lightSpeed;
            if (cfgJunctionProtection && xdata.crossSpeed < target) { target = xdata.crossSpeed; holdForLight = true; }
            // Queued behind a stopped vehicle: cruise 0 until it moves. With cruise > 0 the game keeps pushing it and,
            // pressed against it for 1 s, CCarAI::UpdateCarAI (0x41FA99) reverses it into the car behind (TempAction 3)
            if (sensorTick) UpdateObstacles(vehicle, xdata, modelMin, modelMax, isBike, racer, 0.0f, timeGap, decel);
            xdata.stillTime += dt;
            const bool queued = IsQueuedBehind(xdata, minGap);
            xdata.waitState = holdForLight ? 1 : (queued ? 2 : 0);
            if (UpdateBypass(vehicle, xdata, queued, dt)) {
                // Going around a vehicle that blocks the lane: the bike logic steers around it
                target = std::min(target, cfgBypassSpeed / MPS_TO_KMH);
                if (xdata.waitState != 1) holdForLight = false; // a red light still holds it
                xdata.limiter = LIMIT_BYPASS;
                // Style 6 here too: stopped with style 0 (set while crawling at a light) the game held it behind the
                // blocker for minutes, "bypassing" at 0 km/h
                ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
            }
            else if (queued) {
                target = 0.0f;
                holdForLight = true;
                xdata.limiter = LIMIT_LEADER;
            }
            else if (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED) {
                // The car ahead is pulling away: follow it, don't floor it
                float stopCap = xdata.leaderSpeed + std::sqrt(2.0f * std::min(decel, 4.0f) * std::max(xdata.gap - minGap, 0.0f));
                if (stopCap < target) { target = stopCap; holdForLight = true; xdata.limiter = LIMIT_LEADER; }
            }
            // Stopped 25 s with nothing in front, no cross traffic and no reason of ours, the game still holding the brake: a
            // light that doesn't turn green for it (Bayside: red for 60-90 s with the road clear, a whole queue behind).
            // Treated as broken for 8 s: the light-ignoring driving style, crawling through, the cross traffic check on
            const unsigned int nowMs = CTimer::m_snTimeInMilliseconds;
            if (!holdForLight && !queued && xdata.obstacle == OBSTACLE_NONE && xdata.stillTime > 25.0f && nowMs >= xdata.ignoreLightsUntil &&
                xdata.crossSpeed >= 999.0f && !xdata.bypassUntil) {
                xdata.ignoreLightsUntil = nowMs + 8000;
                xdata.stillTime = 0.0f;
                brokenLightCount++;
            }
            if (nowMs < xdata.ignoreLightsUntil) {
                ap.m_nCarDrivingStyle = DRIVINGSTYLE_AVOID_CARS;
                target = std::min(target, 15.0f / MPS_TO_KMH);
            }
            if (holdForLight) KeepGameStuckTimersFresh(ap);
            xdata.targetSpeed = target;
            ap.m_nCruiseSpeed = static_cast<char>(std::clamp(target * MPS_TO_CRUISE, holdForLight ? 0.0f : cfgCruiseMinSpeed, 127.0f));
            return;
        }

        // Siren on (going to an emergency): drive it as a racer, but keep the game's driving style (ignores lights)
        const bool onEmergencyCall = vehicle->bSirenOrAlarm && vehicle->m_nModelIndex != MODEL_MRWHOOP;
        if (onEmergencyCall && !racer) { xdata.accelLimit = 0.0f; return; }

        float speed = vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS; // m/s
        if (speed > 1.0f) xdata.stillTime = 0.0f;
        const bool bypassing = UpdateBypass(vehicle, xdata, false, dt);

        // Looks like it isn't stuck on the ground
        if (speed * MPS_TO_KMH > 10.0f && distanceToCam < 30.0f) xdata.bRunGroundStuckFix = false;

        // Disable exaggerated driving style
        if (ap.m_nCarDrivingStyle == DRIVINGSTYLE_PLOUGH_THROUGH && !vehicle->IsLawEnforcementVehicle()) {
            ap.m_nCarDrivingStyle = DRIVINGSTYLE_AVOID_CARS;
        }

        // Racers: cars don't weave through traffic like bikes (they overtake by changing lanes, see UpdateOvertaking);
        // bikes do. Both obey traffic lights
        const int racerStyle = isBike ? cfgRacerBikeDrivingStyle : cfgRacerDrivingStyle;
        if (prof.runRedLights && !onEmergencyCall && ap.m_nCarDrivingStyle != DRIVINGSTYLE_PLOUGH_THROUGH) {
            // Doesn't stop at lights (the cross traffic check still slows it down before a junction with someone crossing)
            ap.m_nCarDrivingStyle = DRIVINGSTYLE_AVOID_CARS;
        }
        else if (racer && racerStyle >= 0 && !onEmergencyCall && ap.m_nCarDrivingStyle != DRIVINGSTYLE_PLOUGH_THROUGH) {
            ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(racerStyle);
        }
        // Make use of driving style "6" for cars and bicycles
        else if (ap.m_nCarDrivingStyle <= 2 || ap.m_nCarDrivingStyle == 4) {
            if (!isBike) {
                if (cfgUseBikeLogicOnCars) {
                    if (!bypassing && subClass == VEHICLE_AUTOMOBILE && reinterpret_cast<CAutomobile*>(vehicle)->HasCarStoppedBecauseOfLight())
                        ap.m_nCarDrivingStyle = DRIVINGSTYLE_STOP_FOR_CARS;
                    else
                        ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
                }
            }
            else if (subClass == VEHICLE_BMX) {
                if (cfgUseBikeLogicOnBicycles) ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
                if (cfgBicyclesDontStopForRed) ap.m_nCarDrivingStyle = DRIVINGSTYLE_PLOUGH_THROUGH;
            }
        }

        if (bypassing) ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
        if (CTimer::m_snTimeInMilliseconds < xdata.ignoreLightsUntil) ap.m_nCarDrivingStyle = DRIVINGSTYLE_AVOID_CARS; // see the broken light

        // Only in normal behaviour
        if (cfgOnlyInNormal && ap.m_nCarDrivingStyle != DRIVINGSTYLE_STOP_FOR_CARS) return;

        // Check reverse gear
        if (vehicle->m_nCurrentGear <= 0) return;

        xdata.valid = true;
        UpdateLearnedLimits(vehicle, xdata, speed, CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS);
        if (cfgTestMode) {
            debugVehiclesProcessed++;
            if (racer) debugRacers++;
        }

        // Normal drivers slow down while steering. Racers rely on the curve lookahead instead
        if (!racer) {
            float absSteerAngle = std::max(std::abs(vehicle->m_fSteerAngle) - 0.14f, 0.0f);
            if (absSteerAngle > 0.001f) {
                desired = std::max(desired - (cfgTurningSpeedDecrease * absSteerAngle) / MPS_TO_CRUISE, cfgCruiseMinSpeed / MPS_TO_CRUISE);
            }
        }

        // Turn being made right now. Drops instantly, recovers smoothly (no brake/throttle pumping)
        if (cfgHeadingLimiter) {
            float headingSpeed = GetHeadingLimit(vehicle, xdata, modelMin, modelMax, speed, grip * cfgHeadingGripMargin);
            if (headingSpeed < xdata.headingSpeed) xdata.headingSpeed = headingSpeed;
            else xdata.headingSpeed += (headingSpeed - xdata.headingSpeed) * std::min(0.08f * CTimer::ms_fTimeStep, 1.0f);
        }

        // Sensors, staggered between vehicles and less often far from the camera
        if (sensorTick) {
            UpdateObstacles(vehicle, xdata, modelMin, modelMax, isBike, racer, speed, timeGap, decel);
            refreshRoute(speed);
            FindPathLeader(vehicle, xdata, modelMax.y, std::clamp(speed * timeGap + speed * speed / (2.0f * decel) + 15.0f, 30.0f, 160.0f));
            PredictConflict(vehicle, xdata, speed, std::max(modelMax.x, -modelMin.x));
            UpdateBumps(vehicle, xdata, modelMin, modelMax, isBike, speed);
            // Lane changes: not for bikes (they weave), vehicles on a call, or big ones the game keeps in the right lane
            UpdateLanes(vehicle, xdata, modelMax.y, speed, desired, isBike || onEmergencyCall || (ap.m_nCarCtrlFlags & 0x10));
            if (isBike) UpdateBikeWeaving(vehicle, xdata, speed);
            if (xdata.turnSide != 0 && xdata.turnDist < std::max(40.0f, speed * 4.0f)) SetIndicator(xdata, xdata.turnSide, 400);
        }
        else if (xdata.obstacle != OBSTACLE_NONE) {
            xdata.gap = std::max(xdata.gap - (speed - xdata.leaderSpeed) * dt, 0.1f);
        }

        // Intelligent Driver Model: acceleration wanted for the free road plus the interaction with the obstacle ahead
        float v = speed;
        float accelWanted = accel * (1.0f - std::pow(std::min(v / desired, 2.0f), 4.0f));
        if (xdata.obstacle != OBSTACLE_NONE) {
            float dv = v - xdata.leaderSpeed;
            // Freeways and fast roads: closer (HighwayGapFactor of the time gap, at least 0.3 s), as on real freeways
            const bool fastRoadGap = xdata.roadType == ROAD_HIGHWAY || xdata.roadType == ROAD_FAST;
            // Getting ready to overtake: 40-60% of the following gap (at least 0.25 s), like a driver closing up before
            // pulling out to pass
            float tg = fastRoadGap ? std::max(timeGap * cfgHighwayGapFactor, 0.25f) : timeGap;
            if (CTimer::m_snTimeInMilliseconds < xdata.overtakePrepUntil && xdata.leaderSpeed > 1.5f)
                tg = std::max(tg * Lerp(0.6f, 0.4f, xdata.aggression), 0.25f);
            float sStar = minGap + std::max(0.0f, v * tg + (v * dv) / (2.0f * std::sqrt(accel * decel)));
            // IDM+: the lower of the free-road and the following accelerations. Plain IDM subtracts one from the other:
            // behind a car going about the speed it wants, the free-road term is ~0 and it settled far behind (the gap
            // grows as 1/sqrt(1 - (v/v0)^4)); IDM+ settles at the gap it wants
            accelWanted = accel * std::min(1.0f - std::pow(std::min(v / desired, 2.0f), 4.0f),
                1.0f - std::pow(sStar / std::max(xdata.gap, 0.1f), 2.0f));
        }

        // Making room for a vehicle that signals into our lane: follow it as if it were already in front, with a shorter
        // gap and gentle braking (letting someone in is never an emergency stop)
        bool yielding = false;
        if (xdata.yieldTo && CTimer::m_snTimeInMilliseconds - xdata.yieldTime < 600) {
            float dv = v - xdata.yieldSpeed;
            float sStar = minGap * 0.7f + std::max(0.0f, v * timeGap * 0.6f + (v * dv) / (2.0f * std::sqrt(accel * decel)));
            float yieldAccel = accel * (1.0f - std::pow(std::min(v / desired, 2.0f), 4.0f)) -
                accel * std::pow(sStar / std::max(xdata.yieldGap, 0.1f), 2.0f);
            yieldAccel = std::max(yieldAccel, -decel * 0.8f);
            if (yieldAccel < accelWanted) { accelWanted = yieldAccel; yielding = true; }
        }

        // The AI doses its gas by how far it is from the cruise speed. So when we want to speed up we ask for the
        // full desired speed (full throttle); only when we must slow down we ask for the speed wanted in ~1s
        float target = desired;
        xdata.limiter = desiredLimiter;
        if (accelWanted < 0.0f) {
            target = std::clamp(v + accelWanted * 1.0f, 0.0f, desired);
            if (yielding) xdata.limiter = LIMIT_YIELD;
            else if (xdata.obstacle != OBSTACLE_NONE) xdata.limiter = LIMIT_LEADER;
        }
        if (IsQueuedBehind(xdata, minGap)) {
            target = 0.0f;
            xdata.limiter = LIMIT_LEADER;
        }
        // Never faster than what still stops at minGap behind a real vehicle/ped. Close behind a stopped car the IDM
        // wants ~0 acceleration, and asking the full desired speed there made the game floor it into the car ahead
        // Also if the one ahead brakes hard (~6 m/s2, it brakes later and harder for curves and lights now): our stopping
        // distance within the gap plus its own (it was "its speed + what stops in the gap", rear-ends when it braked)
        if (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED) {
            const float b = std::min(decel, 4.0f), vL = std::max(xdata.leaderSpeed, 0.0f);
            float stopCap = std::sqrt(2.0f * b * std::max(xdata.gap - minGap, 0.0f) + vL * vL * b / 8.0f);
            if (stopCap < target) {
                target = stopCap;
                xdata.limiter = LIMIT_LEADER;
            }
        }

        // On the racing line the corners are wider than in the lane center
        bool onRacingLine = racer && cfgRacingLine && xdata.lineCount >= 2 && xdata.lineBlend > 0.5f;
        float curveSpeed = onRacingLine ? xdata.lineCurveSpeed : xdata.curveSpeed;
        if (cfgCurveLookahead && curveSpeed < target) {
            target = curveSpeed;
            xdata.limiter = (!onRacingLine && xdata.curveIsJunction) ? LIMIT_JUNCTION : LIMIT_CURVE;
        }
        // Crests are on the lane center path; they apply on the racing line too
        if (onRacingLine && xdata.curveIsCrest && xdata.curveSpeed < target) {
            target = xdata.curveSpeed;
            xdata.limiter = LIMIT_CREST;
        }
        else if (xdata.limiter == LIMIT_CURVE && xdata.curveIsCrest && !onRacingLine) {
            xdata.limiter = LIMIT_CREST;
        }

        if (cfgHeadingLimiter && xdata.headingSpeed < target) {
            target = xdata.headingSpeed;
            xdata.limiter = LIMIT_HEADING;
        }

        bool stoppingForLight = false;
        if (cfgTrafficLightDetection && xdata.lightSpeed < target) {
            target = xdata.lightSpeed;
            xdata.limiter = LIMIT_LIGHT;
            stoppingForLight = xdata.lightColor != LIGHT_GREEN;
        }

        if (cfgJunctionProtection && xdata.crossSpeed < target) {
            target = xdata.crossSpeed;
            xdata.limiter = LIMIT_CROSS_TRAFFIC;
            stoppingForLight = true; // allowed to stop completely before the junction
        }

        if (xdata.bFoundBump && checkBumps == 1) {
            float bumpSpeed = desired / cfgBumpSpeedDiv;
            if (bumpSpeed < target) { target = bumpSpeed; xdata.limiter = LIMIT_BUMP; }
        }

        // The minimum speed keeps cars from getting stuck on false obstacles (walls, objects). A red light and a real
        // vehicle/ped ahead are allowed to stop them: with the minimum the car creeps into the one ahead and the game
        // then reverses it (see IsQueuedBehind)
        if (bypassing) {
            // Slowly around the blocking vehicle (still stopping for red lights and cross traffic)
            target = std::min({ desired, cfgBypassSpeed / MPS_TO_KMH, xdata.lightSpeed, xdata.crossSpeed });
            xdata.limiter = LIMIT_BYPASS;
        }
        bool realLeader = xdata.limiter == LIMIT_LEADER && (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED);
        xdata.waitState = stoppingForLight ? 1 : ((realLeader && target < 0.5f) ? 2 : 0);
        float cruise = std::max(target * MPS_TO_CRUISE, (stoppingForLight || realLeader) ? 0.0f : cfgCruiseMinSpeed);
        if (stoppingForLight || realLeader) KeepGameStuckTimersFresh(ap);
        if (subClass == VEHICLE_BMX) {
            float bmxCruise = std::min(cruise * cfgBicycleSpeedMult, cfgBicycleCruiseMaxSpeed);
            if (bmxCruise < cruise) xdata.limiter = LIMIT_BICYCLE;
            cruise = bmxCruise;
        }
        cruise = std::min(cruise, 127.0f);
        xdata.targetSpeed = cruise / MPS_TO_CRUISE;
        // Rounded up: truncated, the game's own speed control braked down to just under our target, and we let its brake
        // through (it looked like braking for a reason): a gas/brake loop at the target speed
        ap.m_nCruiseSpeed = static_cast<char>(std::min(std::ceil(cruise), 127.0f));
    }

    // Stopped/slow vehicle or ped right in front. CCarAI::UpdateCarAI (0x41FA99) reverses a car whose cruise speed isn't 0,
    // that is almost stopped and has been colliding for 1 s (driving styles 1/2/3/5/6; 20+ s for 0/4), so a car waiting
    // behind another must have cruise 0, not just a low one
    // While this mod holds a car (cruise 0) CCarAI::UpdateCarAI skips its stuck check before resetting the stuck timer
    // (CAutoPilot +0x1C) and the "not moving" time (+0x20). They went stale while waiting, so the frame the car was
    // allowed to go again the game found it "stuck" and reversed it at once. Waiting is not being stuck
    static void KeepGameStuckTimersFresh(CAutoPilot& ap) {
        ap.m_nTimeToStartMission = CTimer::m_snTimeInMilliseconds;
        ap.m_nTimeSwitchedToRealPhysics = CTimer::m_snTimeInMilliseconds;
    }

    // Is the stopped vehicle ahead blocking the lane, instead of just waiting in traffic?
    // A vehicle (moving or not) right in front of 'veh': it's in a queue, not stopped on its own. Unless the one in front
    // of it is the abandoned/wrecked one
    static bool InQueue(CVehicle* veh) {
        CVector mn, mx;
        GetEntityDimensions(veh, mn, mx);
        CColPoint colPoint;
        CEntity* entity = nullptr;
        CWorld::pIgnoreEntity = veh;
        bool hit = CWorld::ProcessLineOfSight(GetWorldCoordWithOffset(veh, { 0.0f, mx.y - 0.3f, (mn.z + mx.z) * 0.5f }),
            GetWorldCoordWithOffset(veh, { 0.0f, mx.y + 8.0f, (mn.z + mx.z) * 0.5f }), colPoint, entity,
            false, true, false, false, false, false, false, false);
        CWorld::pIgnoreEntity = nullptr;
        if (!hit || !entity || entity->m_nType != ENTITY_TYPE_VEHICLE) return false;
        CVehicle* ahead = static_cast<CVehicle*>(entity);
        return ahead->m_pDriver && ahead->m_nStatus != STATUS_WRECKED && ahead->m_fHealth > 0.0f;
    }

    // The road it's on has lanes one way only (going round something can't take it into oncoming traffic)
    static bool OneWayHere(CVehicle* veh) {
        const CAutoPilot& ap = veh->m_autoPilot;
        CPathNode* curr = GetNode(ap.m_currentAddress);
        CPathNode* next = GetNode(ap.m_startingRouteNode);
        if (!curr || !next) return false;
        CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
        return link && LanesToward(link, ap.m_currentAddress) == 0;
    }

    static bool LeaderIsBlocking(CVehicle* me, CVehicle* leader, float queuedTime) {
        if (!leader->m_pDriver || leader->m_nStatus == STATUS_WRECKED) return true; // abandoned / wrecked
        // Waiting behind someone itself: a queue (in a turn the cars stand across our way). Going round it squeezed past
        // the whole queue scraping it
        const ExtendedData& lx = vehExData.Get(leader);
        // In a queue (someone in front of it): never round it. In long queues the ones stopped 25 s counted as stuck and
        // cars behind went round them into the oncoming lane
        if (InQueue(leader)) return false;
        // Stopped this long, whatever for: stuck (no light holds anyone 25 s). A bike wedged against a wall at the head of a
        // queue, "waiting for the light", held the whole street
        if (lx.stillTime > 25.0f && queuedTime > 10.0f) return true;
        // Stopped across our way (not a queue: a queue points the same way) for a while: stuck in the junction, a gridlock
        if (std::abs(CVector2D::Dot(leader->GetForward().To2D(), me->GetForward().To2D())) < 0.7f && lx.stillTime > 6.0f) return true;
        if (CTimer::m_snTimeInMilliseconds - lx.lastProcessed > 500) return queuedTime > 8.0f; // not ours (mission, player...)
        if (lx.bypassUntil) return false;                // it's getting around something itself, it'll move
        return lx.waitState == 0 && lx.stillTime > 5.0f; // stopped for no traffic reason: crashed, stuck
    }

    // A lane change round a vehicle blocking ours, on a road with 2+ lanes our way: the next lane (left first) when it's
    // clear beside us and nobody is coming up fast in it (within ~4 s)
    static bool BypassByLaneChange(CVehicle* veh, ExtendedData& xdata, CVehicle* blocker, unsigned int now) {
        CAutoPilot& ap = veh->m_autoPilot;
        CPathNode* curr = GetNode(ap.m_currentAddress);
        CPathNode* next = GetNode(ap.m_startingRouteNode);
        if (!curr || !next || xdata.laneState == LANE_CHANGING) return false;
        CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
        if (!link) return false;
        int lanes = std::min(LanesToward(link, ap.m_startingRouteNode), 7);
        if (lanes < 2) return false;
        CVector2D d = (GetNodePos(next) - GetNodePos(curr)).To2D();
        float len = d.Magnitude();
        if (len < 0.5f) return false;
        d *= 1.0f / len;
        const CVector2D r(d.y, -d.x);
        const CVector2D lp(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
        const float ow = link->OneWayLaneOffset();
        const CVector2D pos = veh->GetPosition().To2D();
        float laneF = CVector2D::Dot(pos - lp, r) / LANE_WIDTH - ow;
        int myLane = std::clamp(static_cast<int>(std::floor(laneF + 0.5f)), 0, lanes - 1);
        for (int target : { myLane - 1, myLane + 1 }) {
            if (target < 0 || target >= lanes) continue;
            int side = target > myLane ? 1 : -1;
            if (!SideClear(veh, xdata, side, 0.0f)) continue;
            bool fastBehind = false;
            for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize && !fastBehind; k++) {
                CVehicle* o = CPools::ms_pVehiclePool->GetAt(k);
                if (!o || o == veh || o == blocker || std::abs(o->GetPosition().z - veh->GetPosition().z) > 4.0f) continue;
                CVector2D op = o->GetPosition().To2D();
                float lon = CVector2D::Dot(op - pos, d);
                int oLane = static_cast<int>(std::floor(CVector2D::Dot(op - lp, r) / LANE_WIDTH - ow + 0.5f));
                if (oLane != target || lon > 12.0f || lon < -120.0f) continue;
                float vo = CVector2D::Dot(o->m_vecMoveSpeed.To2D(), d) * GAME_SPEED_TO_MPS;
                fastBehind = lon > -10.0f || (vo > 2.0f && -lon / vo < 4.0f); // beside us, or arriving within 4 s
            }
            if (fastBehind) continue;
            xdata.changeTarget = target;
            xdata.changeOrigin = myLane;
            xdata.changeFromF = laneF;
            xdata.changeStart = now;
            xdata.changeDuration = 2.8f;
            xdata.changeAfter = LANE_KEEP;
            xdata.laneState = LANE_CHANGING;
            xdata.lastLaneChange = now;
            xdata.pendingLane = -1;
            xdata.indicator = side;
            xdata.indicatorUntil = now + 3200;
            return true;
        }
        return false;
    }

    // Queued for a while behind a vehicle that blocks the lane: go around it with the bike logic (driving style 6,
    // which in FollowPath skips the "car ahead" speed multiplier, table @ 0x43580C) until it's behind us
    static bool UpdateBypass(CVehicle* vehicle, ExtendedData& xdata, bool queued, float dt) {
        CAutoPilot& ap = vehicle->m_autoPilot;
        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        if (xdata.bypassUntil) {
            CVehicle* blocker = xdata.bypassBlocker;
            bool passed = !blocker || !CPools::ms_pVehiclePool->IsObjectValid(blocker) ||
                CVector::Dot(blocker->GetPosition() - vehicle->GetPosition(), vehicle->GetForward()) < -2.0f;
            // It's moving again: no longer blocking, just follow it (the bike logic ignores the car ahead and kept
            // squeezing past it at half a metre until it hit a wall)
            bool moving = !passed && blocker->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS > 3.0f;
            // Rubbing against a vehicle while squeezing past: give up and wait (one scraped along a stopped car at 2 km/h
            // for seconds, six contacts)
            bool scraping = now - vehicle->m_nLastCollisionTime < 300 && vehicle->m_pDamageEntity &&
                vehicle->m_pDamageEntity->m_nType == ENTITY_TYPE_VEHICLE;
            if (scraping) { xdata.noBypassUntil = now + 10000; bypassScrapeCount++; }
            if (!passed && !moving && !scraping && now < xdata.bypassUntil) return true;
            ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(xdata.savedStyle);
            xdata.bypassUntil = 0;
            xdata.bypassBlocker = nullptr;
            lg << "Bypass " << (passed ? "done" : "timed out") << " (veh " << CPools::GetVehicleRef(vehicle) << ")\n";
            return false;
        }
        if (!cfgBypassBlocked || !queued || xdata.waitState != 2) { xdata.queuedTime = 0.0f; return false; }
        if (now < xdata.noBypassUntil) return false;
        // Not in or near a junction: going round a car there means going through the junction across everyone
        {
            CPathNode* nextNode = GetNode(vehicle->m_autoPilot.m_startingRouteNode);
            CPathNode* currNode = GetNode(vehicle->m_autoPilot.m_currentAddress);
            if ((nextNode && nextNode->NumberAdjNodes > 2 && (GetNodePos(nextNode) - vehicle->GetPosition()).Magnitude2D() < 25.0f) ||
                (currNode && currNode->NumberAdjNodes > 2 && (GetNodePos(currNode) - vehicle->GetPosition()).Magnitude2D() < 25.0f))
                return false;
        }
        xdata.queuedTime += dt;
        CVehicle* leader = xdata.leaderVeh;
        if (!leader || !CPools::ms_pVehiclePool->IsObjectValid(leader)) return false;
        // Wrecked (exploded) or without a driver: it will never move. Go around it right away, always
        const bool dead = !leader->m_pDriver || leader->m_nStatus == STATUS_WRECKED || leader->m_fHealth <= 0.0f;
        if (xdata.queuedTime < (dead ? 1.5f : cfgBypassWait)) return false;
        if (!dead && !LeaderIsBlocking(vehicle, leader, xdata.queuedTime)) return false;
        // Two or more lanes our way: round it by changing lanes (indicator, the smooth S, side sensors), like passing a car.
        // The bike logic squeezed past broken-down cars on freeways scraping them and the barrier, and gave up
        if (BypassByLaneChange(vehicle, xdata, leader, now)) {
            xdata.savedStyle = static_cast<unsigned char>(ap.m_nCarDrivingStyle);
            // Style 6 only so the game doesn't hold it behind the blocker (car-ahead multiplier); the lane change steers
            ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
            xdata.bypassUntil = now + 12000;
            xdata.bypassBlocker = leader;
            xdata.queuedTime = 0.0f;
            bypassCount++;
            bypassLaneCount++;
            lg << "Bypass by lane change (veh " << CPools::GetVehicleRef(vehicle) << " around veh " << CPools::GetVehicleRef(leader) << ")\n";
            return true;
        }
        // Never through the oncoming lanes: the bike logic squeezes past only on one-way roads. On two-way roads it went
        // round queues into the cars coming the other way (the Bayside gridlock). A dead car there is removed out of sight
        if (!OneWayHere(vehicle)) {
            if (dead && leader->m_nCreatedBy == RANDOM_VEHICLE && !leader->GetIsOnScreen() &&
                std::find(spawnRemovals.begin(), spawnRemovals.end(), leader) == spawnRemovals.end()) {
                spawnRemovals.push_back(leader);
                brokenRemovedCount++;
            }
            noOncomingBypassCount++;
            xdata.noBypassUntil = now + 5000;
            return false;
        }
        // Otherwise only with room beside it to go around (the side sensors, one side or the other)
        if (!dead && !SideClear(vehicle, xdata, -1, 0.0f) && !SideClear(vehicle, xdata, 1, 0.0f)) return false;
        xdata.savedStyle = static_cast<unsigned char>(ap.m_nCarDrivingStyle);
        ap.m_nCarDrivingStyle = static_cast<eCarDrivingStyle>(6);
        xdata.bypassUntil = now + 12000;
        xdata.bypassBlocker = leader;
        xdata.queuedTime = 0.0f;
        bypassCount++;
        lg << "Bypass started (veh " << CPools::GetVehicleRef(vehicle) << " around veh " << CPools::GetVehicleRef(leader) << ")\n";
        return true;
    }

    static bool IsQueuedBehind(const ExtendedData& xdata, float minGap) {
        return (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED) && xdata.leaderSpeed < 1.5f &&
            xdata.gap < minGap + 0.3f; // stops close behind (it was minGap + 5 m: queues at lights were far apart)
    }

    static void UpdateObstacles(CVehicle* vehicle, ExtendedData& xdata, const CVector& modelMin, const CVector& modelMax,
        bool isBike, bool racer, float speed, float timeGap, float decel)
    {
        float speedKmh = speed * MPS_TO_KMH;
        float speedFactor = std::min(speedKmh / 40.0f, 1.0f);
        // Original look distance
        float shortDist = std::max(speed * (cfgFrontMultDist / GAME_SPEED_TO_MPS), 1.0f);
        // Center line looks as far as needed to stop comfortably, and never less than 10 m: cars wait ~3 m behind the
        // one ahead, and with a ~3 m range at standstill that car blinked in/out of view and the car lurched forward
        float longDist = std::clamp(speed * timeGap + (speed * speed) / (2.0f * decel) + 3.0f, std::max(shortDist, 10.0f), 150.0f);
        // Side lines are for things right in front of the corners. Racers keep them short and narrow,
        // otherwise at high speed they "see" cars in other lanes and parked cars and brake for nothing
        float sideDist = racer ? std::min(shortDist, cfgRacerSideRayLength) : shortDist;
        float sideSpread = racer ? 0.0f : speedKmh / cfgSidesSpeedOffsetDiv;

        float steerOffset = vehicle->m_fSteerAngle * (20.0f * speedFactor) * -1.0f;
        float frontHeight = std::clamp(modelMin.z * 0.4f, -1.0f, 1.0f);
        float frontHeightBonus = isBike ? 0.5f : 0.15f;
        float heightDiffLimit = isBike ? cfgHeightDiffLimit * 2.0f : cfgHeightDiffLimit;

        // A real drop/wall only: ground not found (not streamed in) is not an obstacle,
        // and up to a 35% grade over the line length is just a slope
        bool forceObstacle = false;
        auto snapToGround = [&](CVector& coord, float dist, float height) {
            bool found = false;
            float newZ = FindGroundZ({ coord.x, coord.y, coord.z + cfgCheckGroundHeight + 2.0f }, &found);
            if (!found) return;
            if (std::abs(newZ - coord.z) > std::max(heightDiffLimit, dist * 0.35f)) forceObstacle = true;
            else coord.z = newZ + height;
        };

        CVector* a = xdata.rayA;
        CVector* b = xdata.rayB;
        int numRays = 0;

        // C: follows the terrain in ~25m pieces, so crests, dips and bumps of the road itself aren't seen as walls
        // At body height: near the bottom it passed right at the height of the bumper edge of the car ahead and,
        // depending on how both cars pitched, hit it or went under it
        const float centerHeight = std::max(frontHeight + frontHeightBonus, 0.0f);
        CVector start = GetWorldCoordWithOffset(vehicle, { 0.0f, modelMax.y, centerHeight });
        int pieces = std::clamp(static_cast<int>(std::ceil(longDist / 25.0f)), 1, MAX_RAYS - 3);
        float centerOffset = steerOffset * (longDist / shortDist);
        for (int p = 1; p <= pieces; p++) {
            float f = static_cast<float>(p) / pieces;
            CVector end = GetWorldCoordWithOffset(vehicle, { centerOffset * f, modelMax.y + longDist * f, centerHeight });
            snapToGround(end, longDist * f, std::max(cfgSensorHeight, frontHeightBonus * 2.0f));
            a[numRays] = start;
            b[numRays] = end;
            numRays++;
            start = end;
        }
        const int centerRays = numRays;

        if (!isBike) {
            float sideSteerOffset = steerOffset * (sideDist / shortDist);
            // L
            float steerBackOffset = std::max(modelMax.y * vehicle->m_fSteerAngle, 0.0f);
            a[numRays] = GetWorldCoordWithOffset(vehicle, { modelMin.x, modelMax.y - steerBackOffset, frontHeight + frontHeightBonus });
            b[numRays] = GetWorldCoordWithOffset(vehicle, { modelMin.x - sideSpread + sideSteerOffset, modelMax.y + sideDist, frontHeight / 1.5f });
            snapToGround(b[numRays], sideDist, cfgFinalGroundHeight);
            numRays++;

            // R
            steerBackOffset = std::min(modelMax.y * vehicle->m_fSteerAngle, 0.0f);
            a[numRays] = GetWorldCoordWithOffset(vehicle, { modelMax.x, modelMax.y + steerBackOffset, frontHeight + frontHeightBonus });
            b[numRays] = GetWorldCoordWithOffset(vehicle, { modelMax.x + sideSpread + sideSteerOffset, modelMax.y + sideDist, frontHeight / 1.5f });
            snapToGround(b[numRays], sideDist, cfgFinalGroundHeight);
            numRays++;
        }
        xdata.numRays = numRays;

        for (int i = 0; i < numRays; i++) xdata.rayColor[i] = { 0, 255, 0, 255 };

        xdata.obstacle = OBSTACLE_NONE;
        xdata.gap = 1000.0f;
        xdata.leaderSpeed = 0.0f;
        xdata.leaderVeh = nullptr;

        if (forceObstacle) {
            xdata.obstacle = OBSTACLE_FORCED;
            xdata.gap = shortDist;
            for (int i = 0; i < numRays; i++) xdata.rayColor[i] = { 255, 0, 0, 255 };
            return;
        }

        const CVector forward = vehicle->GetForward();
        float centerDistSoFar = 0.0f;
        bool centerBlocked = false;
        CWorld::pIgnoreEntity = vehicle;
        for (int i = 0; i < numRays; i++) {
            bool isCenter = i < centerRays;
            float rayStartDist = isCenter ? centerDistSoFar : 0.0f;
            if (isCenter) centerDistSoFar += a[i].Distance(b[i]);
            if (isCenter && centerBlocked) { xdata.rayColor[i] = { 90, 90, 90, 255 }; continue; } // behind an obstacle

            CColPoint colPoint;
            CEntity* entity = nullptr;
            if (!CWorld::ProcessLineOfSight(a[i], b[i], colPoint, entity, true, true, true, cfgCheckObjects, false, false, false, false)) continue;
            if (entity && entity == xdata.deadlockIgnore && CTimer::m_snTimeInMilliseconds < xdata.deadlockUntil) continue;

            float gap = rayStartDist + a[i].Distance(colPoint.m_vecPoint);
            eObstacleType type = OBSTACLE_BUILDING;
            float leaderSpeed = 0.0f;
            if (entity) {
                switch (entity->m_nType) {
                case ENTITY_TYPE_VEHICLE: {
                    // In another lane: beside our path (the curve we're on taken into account), its body not overlapping
                    // ours sideways, going our way. Not the one ahead: cars braked for one in the next lane with theirs free
                    // (the side rays open out with speed into the next lane, and in curves the centre one points at it)
                    CVehicle* hitVeh = static_cast<CVehicle*>(entity);
                    CVector2D rel = (hitVeh->GetPosition() - vehicle->GetPosition()).To2D();
                    CVector2D f2 = forward.To2D();
                    float lon = CVector2D::Dot(rel, f2);
                    float curv = speed > 3.0f ? xdata.filtYawRate / speed : 0.0f;  // > 0: turning left
                    float lat = f2.x * rel.y - f2.y * rel.x - 0.5f * curv * lon * lon; // left of our path
                    CVector oMin, oMax;
                    GetEntityDimensions(hitVeh, oMin, oMax);
                    bool parallel = CVector2D::Dot(hitVeh->GetForward().To2D(), f2) > 0.85f;
                    if (parallel && lon > 0.0f && std::abs(lat) > modelMax.x + std::max(oMax.x, -oMin.x) + 0.5f) {
                        xdata.rayColor[i] = { 255, 140, 0, 255 };
                        otherLaneHitCount++;
                        continue;
                    }
                    type = OBSTACLE_VEHICLE;
                    leaderSpeed = std::max(CVector::Dot(hitVeh->m_vecMoveSpeed, forward) * GAME_SPEED_TO_MPS, 0.0f);
                    break;
                }
                case ENTITY_TYPE_PED: type = OBSTACLE_PED; break;
                case ENTITY_TYPE_OBJECT: type = OBSTACLE_OBJECT; break;
                default: break;
                }
            }

            if (type == OBSTACLE_BUILDING) {
                // Roads are buildings too: a surface facing up is ground (bump, ramp, crest), not a wall
                if (colPoint.m_vecNormal.z > cfgWallNormalZ) {
                    xdata.rayColor[i] = { 0, 160, 255, 255 };
                    continue;
                }
                // Far walls are what curves look like when looking straight; the curve lookahead handles those
                if (gap > shortDist + 3.0f) {
                    xdata.rayColor[i] = { 255, 255, 0, 255 };
                    continue;
                }
            }

            xdata.rayColor[i] = { 255, 0, 0, 255 };
            if (isCenter) centerBlocked = true;
            if (gap < xdata.gap) {
                xdata.gap = gap;
                xdata.obstacle = type;
                xdata.leaderSpeed = leaderSpeed;
                xdata.leaderVeh = type == OBSTACLE_VEHICLE ? static_cast<CVehicle*>(entity) : nullptr;
            }
        }

        // The ray hits where it hits (a sloped rear window, the top of a truck bed): close up that read metres more than
        // the real gap, and a car creeping up in a queue touched the one ahead. Bumper to bumper from both bodies
        if (xdata.obstacle == OBSTACLE_VEHICLE && xdata.leaderVeh) xdata.gap = std::max(std::min(xdata.gap, BodyGap(vehicle, xdata.leaderVeh)), 0.1f);

        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        if (xdata.obstacle == OBSTACLE_VEHICLE || xdata.obstacle == OBSTACLE_PED) {
            xdata.memObstacle = xdata.obstacle;
            xdata.memGap = xdata.gap;
            xdata.memLeaderSpeed = xdata.leaderSpeed;
            xdata.memLeaderVeh = xdata.leaderVeh;
            xdata.memTime = now;
        }
        // A stopped vehicle close ahead doesn't vanish: remembered longer (missing it for an instant made queued cars floor
        // it and brake again, rocking)
        else if (xdata.obstacle == OBSTACLE_NONE && xdata.memObstacle != OBSTACLE_NONE && xdata.memGap < 15.0f &&
            now - xdata.memTime < ((xdata.memGap < 8.0f && xdata.memLeaderSpeed < 1.5f) ? 1500u : 500u)) {
            float elapsed = (now - xdata.memTime) / 1000.0f;
            xdata.obstacle = xdata.memObstacle;
            xdata.leaderSpeed = xdata.memLeaderSpeed;
            xdata.leaderVeh = xdata.memLeaderVeh;
            xdata.gap = std::max(xdata.memGap - (speed - xdata.memLeaderSpeed) * elapsed, 0.1f);
        }
        CWorld::pIgnoreEntity = nullptr;
    }

    // Car following along the road: the vehicle ahead in our lane on the route, segment by segment, not only on the
    // straight sensor line. In a curve the one ahead isn't on that line: a racer at 140 km/h saw a car at 30 km/h only
    // 9 m before hitting it. While changing lanes both lanes count
    static void FindPathLeader(CVehicle* veh, ExtendedData& xdata, float halfLength, float range) {
        const int n = xdata.pathCount;
        if (n < 3) return;
        float cum[MAX_LOOKAHEAD_NODES + 1] = {};
        for (int i = 2; i < n; i++) cum[i] = cum[i - 1] + (xdata.path[i] - xdata.path[i - 1]).Magnitude2D();

        // Where a point is along the route (m from the current node) and across it (lanes of that segment). With 'links',
        // only on those road links (another vehicle must be on our road, not on a ramp or a road running close beside it)
        auto project = [&](const CVector2D& p, float& along, float& laneF, CVector2D& dirOut, CCarPathLink* const* links) {
            float best = 12.0f;
            bool found = false;
            for (int seg = 1; seg + 1 < n; seg++) {
                if (!xdata.pathLinkOk[seg]) continue;
                if (links && xdata.pathLink[seg] != links[0] && xdata.pathLink[seg] != links[1]) continue;
                CVector2D a = xdata.path[seg].To2D(), d = xdata.path[seg + 1].To2D() - a;
                float len2 = d.MagnitudeSqr();
                if (len2 < 0.25f) continue;
                float t = std::clamp(CVector2D::Dot(p - a, d) / len2, 0.0f, 1.0f);
                float dist = (a + d * t - p).Magnitude();
                if (dist >= best) continue;
                float len = std::sqrt(len2);
                CVector2D dir = d * (1.0f / len);
                best = dist;
                found = true;
                along = cum[seg] + t * len;
                laneF = CVector2D::Dot(p - xdata.pathLinkPos[seg], CVector2D(dir.y, -dir.x)) / LANE_WIDTH - xdata.pathLaneOffset[seg];
                dirOut = dir;
            }
            return found;
        };

        const CVector pos = veh->GetPosition();
        float myAlong, myLaneF;
        CVector2D myDir;
        if (!project(pos.To2D(), myAlong, myLaneF, myDir, nullptr)) return;
        const CAutoPilot& ap = veh->m_autoPilot;
        const int laneB = xdata.laneState == LANE_CHANGING ? xdata.changeTarget : ap.m_nNextLane;
        // Pulling out to overtake, past half way: only the new lane counts (it braked for the car it was passing and lost the
        // momentum it closed up for)
        const bool pullingOut = xdata.laneState == LANE_CHANGING && xdata.laneAction == 1 &&
            std::abs(myLaneF - ap.m_nNextLane) > 0.45f;
        const int laneA = pullingOut ? laneB : ap.m_nNextLane;

        float bestGap = range;
        CVehicle* best = nullptr;
        float bestSpeed = 0.0f;
        for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize; k++) {
            CVehicle* other = CPools::ms_pVehiclePool->GetAt(k);
            if (!other || other == veh || other == veh->m_pTrailer || other == veh->m_pTractor) continue;
            if (other == xdata.deadlockIgnore && CTimer::m_snTimeInMilliseconds < xdata.deadlockUntil) continue;
            if (std::abs(other->GetPosition().z - pos.z) > 4.0f) continue; // another level of the road
            if ((other->GetPosition() - pos).Magnitude2D() > range + 15.0f) continue;
            CCarPathLink* links[2] = { LinkFromAddress(other->m_autoPilot.m_nCurrentPathNodeInfo),
                LinkFromAddress(other->m_autoPilot.m_nNextPathNodeInfo) };
            if (!links[0] && !links[1]) continue; // not following roads (parked, mission...): the sensors see those
            float along, laneF;
            CVector2D dir;
            if (!project(other->GetPosition().To2D(), along, laneF, dir, links) || along <= myAlong) continue;
            if (std::abs(laneF - laneA) > 0.6f && std::abs(laneF - laneB) > 0.6f) continue;
            CVector2D vel = other->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS;
            if (CVector2D::Dot(other->GetForward().To2D(), dir) < -0.3f && vel.Magnitude() > 2.0f) continue; // coming the other way
            CVector otherMin, otherMax;
            GetEntityDimensions(other, otherMin, otherMax);
            float gap = along - myAlong - halfLength - otherMax.y;
            if (gap < -1.0f || gap >= bestGap) continue; // beside us, or not the closest
            bestGap = std::max(gap, 0.1f);
            best = other;
            bestSpeed = std::max(CVector2D::Dot(vel, dir), 0.0f);
        }
        if (!best) return;
        if (xdata.obstacle != OBSTACLE_NONE && xdata.gap <= bestGap + 0.5f) return; // the sensors already see something closer
        xdata.obstacle = OBSTACLE_VEHICLE;
        xdata.gap = bestGap;
        xdata.leaderSpeed = bestSpeed;
        xdata.leaderVeh = best;
        xdata.memObstacle = OBSTACLE_VEHICLE;
        xdata.memGap = bestGap;
        xdata.memLeaderSpeed = bestSpeed;
        xdata.memLeaderVeh = best;
        xdata.memTime = CTimer::m_snTimeInMilliseconds;
        pathLeaderCount++;
    }

    // Slow manoeuvres (turning at a junction, pulling out): where will we be in the next 1.2 s following the turn we're
    // making, and will another vehicle be there? The straight sensor line points out of the turn and missed a car
    // stopped just around the corner
    static void PredictConflict(CVehicle* veh, ExtendedData& xdata, float speed, float halfWidth) {
        if (speed < 1.0f || speed > 12.5f) return;
        const CVector pos = veh->GetPosition();
        CVector2D p = pos.To2D();
        CVector2D fwd = veh->GetForward().To2D();
        float heading = CGeneral::GetATanOfXY(fwd.x, fwd.y);
        const float yawRate = veh->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS;
        const float myRadius = halfWidth + 0.3f;
        CVector2D mine[6];
        // 6 steps of 0.2 s, but at least ~1.5 m each: crawling round a corner 1.2 s is only a couple of metres, and the
        // car stopped just after the corner was hit before it was "in the future"
        const float step = std::min(std::max(0.2f, 1.5f / std::max(speed, 0.5f)), 1.0f);
        for (int i = 0; i < 6; i++) {
            heading += yawRate * step;
            p += CVector2D(std::cos(heading), std::sin(heading)) * (speed * step);
            mine[i] = p;
        }
        float bestTime = 1000.0f;
        CVehicle* best = nullptr;
        for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize; k++) {
            CVehicle* other = CPools::ms_pVehiclePool->GetAt(k);
            if (!other || other == veh || other == veh->m_pTrailer || other == veh->m_pTractor) continue;
            if (other == xdata.deadlockIgnore && CTimer::m_snTimeInMilliseconds < xdata.deadlockUntil) continue;
            CVector2D op = other->GetPosition().To2D();
            if ((op - pos.To2D()).Magnitude() > 25.0f || std::abs(other->GetPosition().z - pos.z) > 3.0f) continue;
            CVector omin, omax;
            GetEntityDimensions(other, omin, omax);
            // The other car as two circles along its length (front and rear half)
            CVector2D ofwd = other->GetForward().To2D();
            CVector2D ovel = other->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS;
            float oRadius = std::max(omax.x, -omin.x) + 0.2f;
            float oOffset = std::max(omax.y - oRadius, 0.0f);
            const bool otherStopped = ovel.Magnitude() < 1.0f;
            for (int i = 0; i < 6; i++) {
                CVector2D oc = op + ovel * (step * (i + 1));
                bool hit = false;
                for (float side : { -oOffset, 0.0f, oOffset }) hit |= (oc + ofwd * side - mine[i]).Magnitude() < myRadius + oRadius;
                if (hit) {
                    // Who goes first: the one closer to where the paths meet (it got there first). Both waiting for the
                    // other froze whole junctions; a tie is broken the same way by both, so one of them always goes
                    float myDist = (mine[i] - pos.To2D()).Magnitude();
                    float otherDist = (mine[i] - op).Magnitude();
                    bool iGoFirst = myDist < otherDist - 1.0f || (std::abs(myDist - otherDist) <= 1.0f && veh < other);
                    // A stopped vehicle in our way (the queue just around the corner we're turning) always stops us;
                    // who-goes-first is only between vehicles that are both moving
                    if (iGoFirst && !otherStopped) break;
                    if (step * (i + 1) < bestTime) { bestTime = step * (i + 1); best = other; }
                    break;
                }
            }
        }
        if (!best) return;
        float gap = std::max(speed * bestTime - 0.5f, 0.1f);
        if (xdata.obstacle != OBSTACLE_NONE && xdata.gap <= gap) return;
        xdata.obstacle = OBSTACLE_VEHICLE;
        xdata.gap = gap;
        xdata.leaderSpeed = std::max(CVector2D::Dot(best->m_vecMoveSpeed.To2D() * GAME_SPEED_TO_MPS, veh->GetForward().To2D()), 0.0f);
        xdata.leaderVeh = best;
        predictedConflictCount++;
    }

    static void UpdateBumps(CVehicle* vehicle, ExtendedData& xdata, const CVector& modelMin, const CVector& modelMax, bool isBike, float speed) {
        if (checkBumps != 1) return;
        xdata.framesDelayCountSearchBump += std::max(cfgSensorInterval, 1);
        if (xdata.framesDelayCountSearchBump < cfgSearchBumpFramesDelay) return;
        xdata.framesDelayCountSearchBump = 0;

        float dist = std::max(speed, 1.0f);
        CVector bumpSearchCoord = GetWorldCoordWithOffset(vehicle, { 0.0f, modelMax.y + (dist * cfgBumpSearchDistMult), 0.0f });
        float bumpSearchRadius = (isBike ? cfgBumpSearchRadius * 0.8f : cfgBumpSearchRadius) + std::abs(modelMin.y);
        xdata.bFoundBump = ThereIsObjectHere(bumpSearchCoord, bumpSearchRadius, cfgBumpModelStart, cfgBumpModelEnd);
    }

    static bool IsDrivenByMod(const ExtendedData& x) { return CTimer::m_snTimeInMilliseconds - x.lastProcessed < 500; }

    // A new road segment after a junction. The game keeps the lane NUMBER (CCarCtrl::PickNextNodeRandomly, 0x42E63B), so a
    // car coming from an on-ramp on the right in its lane 0 crossed the whole road to the leftmost lane, cutting across
    // the others. Instead: the lane it's really in
    static void OnNewSegment(CVehicle* veh, ExtendedData& xdata) {
        CAutoPilot& ap = veh->m_autoPilot;
        CPathNode* prev = GetNode(ap.field_8);
        CPathNode* curr = GetNode(ap.m_currentAddress);
        CPathNode* next = GetNode(ap.m_startingRouteNode);
        if (!prev || !curr || !next || curr->NumberAdjNodes <= 2) return; // plain road: the game's lane is right
        CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode);
        if (!link) return;
        int lanes = std::min(LanesToward(link, ap.m_startingRouteNode), 7);
        if (lanes <= 1) return;
        CVector2D d = (GetNodePos(next) - GetNodePos(curr)).To2D();
        CVector2D in = (GetNodePos(curr) - GetNodePos(prev)).To2D();
        float len = d.Magnitude();
        if (len < 0.5f || in.Magnitude() < 0.5f) return;
        d *= 1.0f / len;
        // The lane it's in (moving to another one is a lane change, with indicator and checks, see UpdateLanes).
        // Putting a car that turned right straight into the right lane cut across whoever was there
        const CVector2D r(d.y, -d.x);
        const CVector2D linkPos(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
        float laneF = CVector2D::Dot(veh->GetPosition().To2D() - linkPos, r) / LANE_WIDTH - link->OneWayLaneOffset();
        int lane = std::clamp(static_cast<int>(std::floor(laneF + 0.5f)), 0, lanes - 1);
        float turn = WrapAngle(CGeneral::GetATanOfXY(d.x, d.y) - CGeneral::GetATanOfXY(in.x, in.y)); // > 0 = left
        // Turning: the position across the new road is how far into the junction the car is, not its lane (two cars turning
        // right side by side were both put in the same lane and steered into each other). Lanes keep their order instead:
        // turning right counted from the right (rightmost -> rightmost), turning left from the left
        CCarPathLink* inLink = GetLinkBetween(ap.field_8, prev, ap.m_currentAddress);
        int inLanes = inLink ? LanesToward(inLink, ap.m_currentAddress) : 0;
        if (std::abs(turn) > 0.6f) {
            if (inLanes > 0) {
                int oldLane = std::clamp(static_cast<int>(ap.m_nCurrentLane), 0, inLanes - 1);
                lane = std::clamp(turn < 0.0f ? lanes - inLanes + oldLane : oldLane, 0, lanes - 1);
            }
        }
        // Going on (freeway forks and ramps: 7-22 degrees): the car isn't lined up with the new link yet, and seen from its
        // line it was 10-18 m "to the left" -> always lane 0, a lane change for nothing (9 of the 10 the road scan found).
        // Same number of lanes: same lane. Otherwise the lane it's in on the link it's lined up with, carried over to the new
        // one where both meet (at the junction node)
        else if (inLink && inLanes > 0) {
            if (inLanes == lanes) lane = std::clamp(static_cast<int>(ap.m_nCurrentLane), 0, lanes - 1);
            else {
                CVector2D din = in * (1.0f / in.Magnitude());
                const CVector2D rin(din.y, -din.x);
                const CVector2D inPos(inLink->m_vecPosn.x / 8.0f, inLink->m_vecPosn.y / 8.0f);
                const CVector2D node = GetNodePos(curr).To2D();
                float laneIn = CVector2D::Dot(veh->GetPosition().To2D() - inPos, rin) / LANE_WIDTH - inLink->OneWayLaneOffset();
                CVector2D lane0In = inPos + din * CVector2D::Dot(node - inPos, din) + rin * (inLink->OneWayLaneOffset() * LANE_WIDTH);
                CVector2D lane0Out = linkPos + d * CVector2D::Dot(node - linkPos, d) + r * (link->OneWayLaneOffset() * LANE_WIDTH);
                float laneOut = laneIn + CVector2D::Dot(lane0In - lane0Out, r) / LANE_WIDTH;
                lane = std::clamp(static_cast<int>(std::floor(laneOut + 0.5f)), 0, lanes - 1);
            }
        }
        if (lane != ap.m_nNextLane) {
            ap.m_nNextLane = static_cast<char>(lane);
            junctionLaneCount++;
        }
        // Going on along the same road (freeway ramps join and leave at junction nodes all the time): an overtake or a lane
        // change in progress goes on. After a turn, whatever it was signalling/overtaking for was on the old road
        if (std::abs(turn) < 0.6f) return;
        if (xdata.laneState == LANE_CHANGING) ap.m_nCurrentLane = static_cast<char>(std::clamp(xdata.changeTarget, 0, 7));
        if (xdata.laneState == LANE_OVERTAKING || (xdata.laneState == LANE_CHANGING && xdata.changeAfter == LANE_OVERTAKING)) overtakeEndTurn++;
        xdata.laneState = LANE_KEEP;
        xdata.pendingLane = -1;
        xdata.returnLane = -1;
        xdata.overtakeTarget = nullptr;
    }

    // Side sensors: three lines from the side of the vehicle (front corner forward, middle, rear corner backwards), one
    // lane wide towards 'side'. A vehicle there = that lane isn't free beside us
    static bool SideClear(CVehicle* veh, ExtendedData& xdata, int side, float speed, float lanes = 0.9f) {
        CVector mn, mx;
        GetEntityDimensions(veh, mn, mx);
        const float x0 = (side > 0 ? mx.x : mn.x) - side * 0.4f; // from inside the body: something already touching counts
        // 'lanes' wide: during a lane change only what's left to the target lane (a full lane from halfway across reached
        // into the lane beyond it, and cars there aborted most overtakes)
        const float reach = side * (LANE_WIDTH * lanes + 0.4f);
        const float z = (mn.z + mx.z) * 0.5f;
        const float yFrom[3] = { mx.y * 0.9f, 0.0f, mn.y * 0.9f };
        // Backwards far enough for someone fast coming up behind (a slow car only looked ~6 m back)
        const float yTo[3] = { mx.y + std::min(speed * 0.6f, 15.0f), 0.0f, mn.y - std::clamp(speed * 0.8f, 15.0f, 25.0f) };
        bool clear = true;
        CWorld::pIgnoreEntity = veh;
        for (int i = 0; i < 3; i++) {
            CVector a = GetWorldCoordWithOffset(veh, { x0, yFrom[i], z });
            CVector b = GetWorldCoordWithOffset(veh, { x0 + reach, yTo[i], z });
            CColPoint colPoint;
            CEntity* entity = nullptr;
            // Vehicles only: that the lane exists comes from the road data, and a barrier beside the lane (the diagonal lines
            // reach it in curves) isn't someone in it
            bool hit = CWorld::ProcessLineOfSight(a, b, colPoint, entity, false, true, false, false, false, false, false, false);
            if (hit && entity == veh->m_pTrailer) hit = false;
            clear &= !hit;
            xdata.sideRayA[i] = a;
            xdata.sideRayB[i] = b;
            xdata.sideRayColor[i] = hit ? CRGBA(255, 0, 0, 255) : CRGBA(0, 255, 255, 255);
        }
        CWorld::pIgnoreEntity = nullptr;
        xdata.numSideRays = 3;
        xdata.sideRayTime = CTimer::m_snTimeInMilliseconds;
        return clear;
    }

    // Bikes weave through traffic with the game's logic, which doesn't look sideways: one cut diagonally across a lane
    // into a car beside it. Drifting towards a side with a vehicle there: held parallel to the road (FollowPathHook)
    static void UpdateBikeWeaving(CVehicle* veh, ExtendedData& xdata, float speed) {
        if (speed < 5.0f || xdata.pathCount < 3) return;
        CVector2D d = (xdata.path[2] - xdata.path[1]).To2D();
        float len = d.Magnitude();
        if (len < 0.5f) return;
        d *= 1.0f / len;
        float drift = CVector2D::Dot(veh->GetForward().To2D(), CVector2D(d.y, -d.x)); // > 0: towards the right
        if (std::abs(drift) < 0.06f) return;
        if (!SideClear(veh, xdata, drift > 0.0f ? 1 : -1, speed, 0.5f)) {
            xdata.bikeRoadDir = d;
            xdata.bikeHoldUntil = CTimer::m_snTimeInMilliseconds + 300;
            bikeHoldCount++;
        }
    }

    static void SetIndicator(ExtendedData& xdata, int side, unsigned int ms) {
        if (!cfgTurnSignals || side == 0) return;
        xdata.indicator = side;
        xdata.indicatorUntil = std::max(xdata.indicatorUntil, CTimer::m_snTimeInMilliseconds + ms);
    }

    // Lane changes of the mod. The game AI no longer changes lanes at random (see LaneDiscipline): a driver changes lanes
    // only for a reason, in this order:
    //  1. to be in the lane its route needs: right lane before an exit/right turn, left lane before a left turn
    //     (LanePrepareTime s before the junction);
    //  2. to overtake a vehicle clearly slower than it wants to go (OvertakeSpeedRatio of the profile);
    //  3. back to the right after overtaking, or when the right lane is free (KeepRight profiles); to the fast lane when
    //     it's free (profiles that prefer it).
    // Only on roads with 2+ lanes in OUR direction (never through oncoming traffic), into a lane that's clear ahead and
    // behind (also of faster vehicles coming from behind and of vehicles moving into it). Overtaking and keeping right
    // only away from junctions, lights and corners, and at least LaneChangeInterval s apart.
    // Communication: before changing the driver turns the indicator on and waits (0.5-1.5 s, by aggression); it changes
    // only if the lane is still clear. Cooperative drivers (aggression < 0.65) behind it in that lane see the indicator
    // and back off to make room, and a driver that sees the one behind making room accepts a smaller gap.
    // The lane change itself is the game's: m_nNextLane, which the AI reaches by the end of the link
    static void UpdateLanes(CVehicle* veh, ExtendedData& xdata, float halfLength, float speed, float desired, bool laneChangesDisabled) {
        CAutoPilot& ap = veh->m_autoPilot;
        const DriverProfile& prof = profiles[xdata.profile];
        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        const float aggr = xdata.aggression;
        xdata.trafficAround = false;
        xdata.lanesHere = 0;
        CVehicle* oldYield = xdata.yieldTo;
        xdata.yieldTo = nullptr;
        xdata.yieldGap = 1000.0f;
        auto stopLaneChanges = [&]() {
            if (xdata.laneState == LANE_OVERTAKING) overtakeAbortCount++;
            xdata.laneState = LANE_KEEP;
            xdata.pendingLane = -1;
            xdata.returnLane = -1;
            xdata.overtakeTarget = nullptr;
        };

        CPathNode* curr = GetNode(ap.m_currentAddress);
        CPathNode* next = GetNode(ap.m_startingRouteNode);
        if (!curr || !next) return;
        CCarPathLink* link = GetLinkBetween(ap.m_currentAddress, curr, ap.m_startingRouteNode); // link of m_nNextLane
        if (!link) return;
        int lanes = std::min(LanesToward(link, ap.m_startingRouteNode), 7);
        CVector2D d = (GetNodePos(next) - GetNodePos(curr)).To2D();
        float len = d.Magnitude();
        if (lanes <= 0 || len < 0.5f) return;
        d *= 1.0f / len;
        // The game lays its lanes along the link's own direction, not the line between the nodes: on a road that curves
        // between them the two differ, and lanes measured along the nodes were metres off (on Grove -> Bayside the car read
        // -10 m across a link at its start; the lane sync then moved it out of its lane: a forced lane change)
        {
            CVector2D ld(link->m_nDirX / 100.0f, link->m_nDirY / 100.0f);
            float ll = ld.Magnitude();
            if (ll > 0.5f) {
                ld *= 1.0f / ll;
                d = CVector2D::Dot(ld, d) < 0.0f ? ld * -1.0f : ld;
            }
        }
        const CVector2D r(d.y, -d.x);
        const CVector2D linkPos(link->m_vecPosn.x / 8.0f, link->m_vecPosn.y / 8.0f);
        const float ow = link->OneWayLaneOffset();
        const CVector2D pos = veh->GetPosition().To2D();
        // Across the road is only a lane near the link's point: away from it the game's path curves towards the next link
        const bool nearLinkPoint = std::abs(CVector2D::Dot(pos - linkPos, d)) < 10.0f;
        // The game's lane number and where the car really is disagree (the number is carried over from a road with other
        // lanes, a lane that ended...): the game steered straight across to "its" lane, into whoever was there. The lane
        // it's in is its lane; going somewhere else is a lane change (indicator, side check, board)
        // Not in or just out of a junction, nor while still turning: there the position across the road isn't a lane (a car
        // swinging wide round a right turn was given the lane of the one turning beside it)
        const bool afterJunction = curr->NumberAdjNodes > 2 && (GetNodePos(curr).To2D() - veh->GetPosition().To2D()).Magnitude() < 30.0f;
        const bool beforeJunction = next->NumberAdjNodes > 2 && (GetNodePos(next).To2D() - veh->GetPosition().To2D()).Magnitude() < 15.0f;
        // Only the number carried over onto a new segment (in its first 2 s), held for 0.6 s, and with nobody beside us in
        // the lane we're in: anywhere else a mismatch is the game's avoidance weaving towards the next lane, and making
        // that the car's lane sent it on into whoever was there
        bool mismatch = false;
        if (xdata.laneState != LANE_CHANGING && xdata.limiter != LIMIT_BYPASS && xdata.lineBlend <= 0.0f &&
            !afterJunction && !beforeJunction && std::abs(xdata.filtYawRate) < 0.1f && nearLinkPoint &&
            CTimer::m_snTimeInMilliseconds - xdata.newLinkTime < 4000) {
            float laneNow = CVector2D::Dot(pos - linkPos, r) / LANE_WIDTH - ow;
            int physical = static_cast<int>(std::floor(laneNow + 0.5f));
            if (physical >= 0 && physical < lanes && std::abs(laneNow - veh->m_autoPilot.m_nNextLane) > 0.7f) {
                mismatch = true;
                if (!xdata.laneMismatchSince) xdata.laneMismatchSince = CTimer::m_snTimeInMilliseconds;
                bool someoneBeside = false;
                for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize && !someoneBeside; k++) {
                    CVehicle* other = CPools::ms_pVehiclePool->GetAt(k);
                    if (!other || other == veh || std::abs(other->GetPosition().z - veh->GetPosition().z) > 4.0f) continue;
                    CVector2D op = other->GetPosition().To2D();
                    float lon = CVector2D::Dot(op - pos, d);
                    int lane = static_cast<int>(std::floor(CVector2D::Dot(op - linkPos, r) / LANE_WIDTH - ow + 0.5f));
                    someoneBeside = lane == physical && std::abs(lon) < halfLength * 2.0f + 6.0f;
                }
                if (!someoneBeside && CTimer::m_snTimeInMilliseconds - xdata.laneMismatchSince > 600) {
                    veh->m_autoPilot.m_nCurrentLane = veh->m_autoPilot.m_nNextLane = static_cast<char>(physical);
                    laneSyncCount++;
                    mismatch = false;
                }
            }
        }
        if (!mismatch) xdata.laneMismatchSince = 0;
        const int myLane = std::clamp(static_cast<int>(ap.m_nNextLane), 0, lanes - 1);
        const UInt16 myLink = *reinterpret_cast<const UInt16*>(&ap.m_nNextPathNodeInfo);
        xdata.lanesHere = lanes;
        const bool cooperative = cfgYieldToSignals && aggr < 0.65f;
        const bool zipper = xdata.dropLanes > 0 && xdata.dropDist < std::max(60.0f, speed * 4.0f);
        bool zipperYield = false;

        // What's in each of our lanes: closest vehicle ahead and behind
        struct LaneInfo {
            float frontGap = 1e9f, frontSpeed = 0.0f, backGap = 1e9f, backSpeed = 0.0f;
            CVehicle* frontVeh = nullptr;
            CVehicle* backVeh = nullptr;
            bool overtakerBehind = false; // someone behind us is overtaking into this lane (or about to): it goes first
        } laneInfo[7];
        auto addToLane = [&](int lane, float lon, float bodies, float otherSpeed, CVehicle* other) {
            LaneInfo& info = laneInfo[lane];
            if (lon >= 0.0f) {
                float gap = std::max(lon - bodies, 0.0f);
                if (gap < info.frontGap) { info.frontGap = gap; info.frontSpeed = otherSpeed; info.frontVeh = other; }
            }
            else {
                float gap = std::max(-lon - bodies, 0.0f);
                if (gap < info.backGap) { info.backGap = gap; info.backSpeed = otherSpeed; info.backVeh = other; }
            }
        };
        for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize; k++) {
            CVehicle* other = CPools::ms_pVehiclePool->GetAt(k);
            if (!other || other == veh || other == veh->m_pTrailer || other == veh->m_pTractor) continue;
            if (std::abs(other->GetPosition().z - veh->GetPosition().z) > 4.0f) continue;
            CVector2D otherPos = other->GetPosition().To2D();
            float lon = CVector2D::Dot(otherPos - pos, d);
            // Ahead 120 m; behind up to 200 m (someone at 165 km/h is 120 m away 2.6 s before it's on us)
            if (lon > 120.0f || lon < -200.0f) continue;
            float lat = CVector2D::Dot(otherPos - linkPos, r);
            float laneF = lat / LANE_WIDTH - ow;
            int lane = static_cast<int>(std::floor(laneF + 0.5f));
            if (lane < 0 || lane >= lanes) continue; // not in one of our lanes

            CVector otherMin, otherMax;
            GetEntityDimensions(other, otherMin, otherMax);
            float bodies = halfLength + otherMax.y;
            float otherSpeed = CVector2D::Dot(other->m_vecMoveSpeed.To2D(), d) * GAME_SPEED_TO_MPS;
            addToLane(lane, lon, bodies, otherSpeed, other);
            if (lane != myLane && std::abs(lon) < bodies + 8.0f) xdata.trafficAround = true; // beside us

            bool sameLink = other->m_pDriver && *reinterpret_cast<const UInt16*>(&other->m_autoPilot.m_nNextPathNodeInfo) == myLink;
            if (!sameLink) continue;
            // Moving into another lane of the same link: it's in that one too
            const ExtendedData& ox = vehExData.Get(other);
            bool oxChanging = IsDrivenByMod(ox) && ox.laneState == LANE_CHANGING;
            int target = oxChanging ? ox.changeTarget : other->m_autoPilot.m_nNextLane;
            if (target != lane && target >= 0 && target < lanes) addToLane(target, lon, bodies, otherSpeed, other);
            // Signalling into our lane, ahead of us: make room for it
            bool signalsIntoMine = IsDrivenByMod(ox) && ox.pendingLane == myLane &&
                (ox.laneState == LANE_SIGNAL_OVERTAKE || ox.laneState == LANE_SIGNAL_MOVE);
            bool movingIntoMine = lane != myLane && target == myLane;
            // An overtake behind us into some lane (signalling, moving over, or already passing in it)
            if (IsDrivenByMod(ox) && lon < 0.0f && lon > -80.0f) {
                int overtakeLane = -1;
                if (ox.laneState == LANE_SIGNAL_OVERTAKE) overtakeLane = ox.pendingLane;
                else if (ox.laneState == LANE_CHANGING && ox.changeAfter == LANE_OVERTAKING) overtakeLane = ox.changeTarget;
                else if (ox.laneState == LANE_OVERTAKING) overtakeLane = lane;
                if (overtakeLane >= 0 && overtakeLane < lanes) laneInfo[overtakeLane].overtakerBehind = true;
            }
            // Lane ending ahead: zipper. Whoever is ahead (level or in front) in the lane next to ours goes first, the other
            // one lets it in. Everyone, at any speed (side by side at the end of the lane, the game put both in one lane)
            if (zipper && std::abs(lane - myLane) == 1 && (lane >= xdata.dropLanes || myLane >= xdata.dropLanes) &&
                (lon > halfLength * 0.3f || (lon > -halfLength * 0.3f && other < veh)) && lon < 40.0f) // level: one of them
            {
                float gap = std::max(lon - bodies, 0.1f);
                if (gap < xdata.yieldGap) {
                    xdata.yieldTo = other;
                    xdata.yieldGap = gap;
                    xdata.yieldSpeed = std::max(otherSpeed, 0.0f);
                    zipperYield = true;
                }
            }
            if (cooperative && lane != myLane && (signalsIntoMine || movingIntoMine) && lon > -2.0f && lon < 60.0f) {
                float gap = std::max(lon - bodies, 0.1f);
                if (gap < xdata.yieldGap) {
                    xdata.yieldTo = other;
                    xdata.yieldGap = gap;
                    xdata.yieldSpeed = std::max(otherSpeed, 0.0f);
                }
            }
        }
        if (laneInfo[myLane].frontGap < std::max(25.0f, speed * 2.0f)) xdata.trafficAround = true;
        if (xdata.yieldTo) {
            if (xdata.yieldTo != oldYield) { yieldCount++; if (zipperYield) zipperYieldCount++; }
            xdata.yieldTime = now;
        }

        // Asking for the way: a faster driver behind in our lane, held up by us. We move over for it if it wants to go
        // clearly faster than we do (PassRequestMinDiff, 20 km/h: who wants 200 stays ahead of who wants 180). Only in
        // the left (overtaking) lane: in the others it can overtake by the left. It asks flashing the high beams
        bool passRequested = false;
        const float passDiff = cfgPassRequestMinDiff / MPS_TO_KMH;
        // Moving in the left lane of a road with 2+ lanes, away from junctions: not in queues at lights
        const bool passContext = lanes >= 2 && myLane == 0 && speed > 12.0f && !(next->NumberAdjNodes > 2 &&
            (GetNodePos(next).To2D() - pos).Magnitude() < 60.0f) && !(xdata.lightColor != LIGHT_NONE && xdata.lightDist < 100.0f);
        if (passContext) {
            const LaneInfo& mineInfo = laneInfo[myLane];
            if (mineInfo.backVeh && mineInfo.backGap < std::max(25.0f, speed * 1.5f)) {
                const ExtendedData& bx = vehExData.Get(mineInfo.backVeh);
                if (IsDrivenByMod(bx) && bx.desiredSpeed >= xdata.desiredSpeed + passDiff && bx.desiredSpeed > speed + 1.5f &&
                    bx.leaderVeh == veh) {
                    passRequested = true;
                    xdata.passRequester = mineInfo.backVeh;
                }
            }
            const LaneInfo& ahead = laneInfo[myLane];
            if (ahead.frontVeh && ahead.frontGap < std::max(25.0f, speed * 1.5f) && xdata.leaderVeh == ahead.frontVeh) {
                const ExtendedData& fx = vehExData.Get(ahead.frontVeh);
                if (IsDrivenByMod(fx) && xdata.desiredSpeed >= fx.desiredSpeed + passDiff && xdata.desiredSpeed > ahead.frontSpeed + 1.5f) {
                    if (now - xdata.lastPassRequest > 4000) { passRequestCount++; StartPassMeasure(veh, false); }
                    xdata.lastPassRequest = now;
                    if (xdata.laneState == LANE_KEEP) SetIndicator(xdata, -1, 400);
                }
            }
        }

        // Time stuck behind a slower vehicle: one glimpse of it isn't a reason to overtake
        const LaneInfo& mine = laneInfo[myLane];
        bool blocked = prof.overtakeRatio > 0.0f && mine.frontVeh && mine.frontGap < std::max(30.0f, speed * 3.0f) &&
            mine.frontSpeed < desired * prof.overtakeRatio && mine.frontSpeed > 1.5f;
        float tick = std::min((now - xdata.lastLaneTick) / 1000.0f, 0.5f);
        xdata.lastLaneTick = now;
        xdata.blockedTime = blocked ? xdata.blockedTime + tick : 0.0f;
        // About to overtake: right up behind it (shorter following gap, see the IDM) so it pulls out with momentum
        if (blocked && lanes >= 2 && !laneChangesDisabled && cfgOvertaking) xdata.overtakePrepUntil = now + 600;
        const float myLaneF = CVector2D::Dot(pos - linkPos, r) / LANE_WIDTH - ow; // where it really is, in lanes

        if (passRequested && xdata.laneState != LANE_KEEP) CountPassRefusal(xdata, 5); // signalling, overtaking, moving over
        else if (passRequested && laneChangesDisabled) CountPassRefusal(xdata, 7);
        if (laneChangesDisabled || lanes < 2 || !cfgOvertaking) { stopLaneChanges(); return; }
        if (xdata.returnLane >= lanes) xdata.returnLane = -1;
        if (xdata.pendingLane >= lanes) { xdata.pendingLane = -1; xdata.laneState = LANE_KEEP; }

        // Never right before a junction, a traffic light or the braking zone of a corner
        float distToNext = (GetNodePos(next).To2D() - pos).Magnitude();
        bool nearJunction = next->NumberAdjNodes > 2 && distToNext < 40.0f;
        bool zoneOk = !nearJunction &&
            !(xdata.lightColor != LIGHT_NONE && xdata.lightDist < 80.0f) &&
            !(xdata.curveSpeed < speed * 1.15f && xdata.curveDist < 100.0f) &&
            xdata.crossSpeed >= 999.0f;

        // Lane the route needs soon (the turn/exit is within LanePrepareTime s)
        float prepareDist = std::max(speed * cfgLanePrepareTime, 80.0f);
        int requiredLane = -1;
        if (cfgDestinations && xdata.requiredSide != 0 && xdata.requiredDist < prepareDist && xdata.requiredDist > 20.0f) {
            requiredLane = xdata.requiredSide > 0 ? lanes - 1 : 0;
        }
        if (xdata.dropLanes > 0 && xdata.dropDist < prepareDist && xdata.dropDist > 5.0f) {
            int maxLane = xdata.dropLanes - 1;
            if (requiredLane > maxLane || (requiredLane < 0 && myLane > maxLane)) requiredLane = maxLane;
        }

        auto laneSafe = [&](int lane) {
            const LaneInfo& info = laneInfo[lane];
            float frontNeed = std::max(10.0f, speed * prof.timeGap * 2.4f); // x2: the following gap was halved
            bool frontOk = info.frontGap > frontNeed || (info.frontSpeed > speed + 2.0f && info.frontGap > frontNeed * 0.6f);
            // The one behind is making room for us (it saw the indicator): a smaller gap is enough
            bool backYields = false;
            if (info.backVeh) {
                const ExtendedData& bx = vehExData.Get(info.backVeh);
                backYields = IsDrivenByMod(bx) && bx.yieldTo == veh && now - bx.yieldTime < 1000;
            }
            float backNeed = backYields ? std::max(4.0f, speed * 0.25f) : std::max(8.0f, speed * 0.4f);
            bool backOk = info.backGap > backNeed;
            // The one behind speeds up to what it wants (a slow car moving right in front of one at 70 that went on to 100)
            float backSpeed = info.backSpeed;
            if (info.backVeh && !backYields) {
                const ExtendedData& bx = vehExData.Get(info.backVeh);
                if (IsDrivenByMod(bx)) backSpeed = std::max(backSpeed, std::min(bx.desiredSpeed, backSpeed + 8.0f));
            }
            float closing = backSpeed - speed;
            if (backOk && closing > 0.0f && !backYields) backOk = info.backGap / closing > cfgOvertakeRearTime * Lerp(1.3f, 0.7f, aggr);
            return frontOk && backOk;
        };
        // Overtake planned with what the cars tell each other (positions, speeds, the speed each one wants): how long the
        // whole pass takes, and where everyone in that lane will be meanwhile (the ones behind speed up to what they want).
        // Only when the whole manoeuvre fits (and before the route's turn) it goes, without the fixed gap rules
        auto overtakeClear = [&](int lane) {
            const LaneInfo& m = laneInfo[myLane];
            if (!m.frontVeh) return false;
            CVector tmn, tmx;
            GetEntityDimensions(m.frontVeh, tmn, tmx);
            const float vTarget = std::max(m.frontSpeed, 0.0f);
            const float vPass = std::max(std::min(GetVehicleTopSpeed(veh), std::max(desired, vTarget + 6.0f)), speed);
            if (vPass < vTarget + 2.0f) return false;
            const float passAccel = 3.0f;
            // Distance to gain on it: the gap, both bodies, and room to come back in ahead of it
            const float toGain = m.frontGap + halfLength * 2.0f + (tmx.y - tmn.y) + std::max(8.0f, vTarget * 0.8f);
            // Time: accelerating to vPass, then at vPass
            float T = 0.0f, gained = 0.0f, v = speed;
            while (gained < toGain && T < 20.0f) {
                v = std::min(v + passAccel * 0.25f, vPass);
                gained += (v - vTarget) * 0.25f;
                T += 0.25f;
            }
            if (T >= 20.0f) return false;
            T += xdata.changeDuration * 0.5f;
            // The route's turn/exit isn't before the end of it
            float covered = speed * T + 0.5f * passAccel * T * T;
            if (xdata.requiredSide > 0 && xdata.requiredDist < covered + 60.0f) return false;
            // Everyone in that lane over that time
            for (int k = 0; k < CPools::ms_pVehiclePool->m_nSize; k++) {
                CVehicle* o = CPools::ms_pVehiclePool->GetAt(k);
                if (!o || o == veh || o == m.frontVeh || std::abs(o->GetPosition().z - veh->GetPosition().z) > 4.0f) continue;
                CVector2D op = o->GetPosition().To2D();
                float lon0 = CVector2D::Dot(op - pos, d);
                if (lon0 < -250.0f || lon0 > 300.0f) continue;
                int oLane = static_cast<int>(std::floor(CVector2D::Dot(op - linkPos, r) / LANE_WIDTH - ow + 0.5f));
                if (oLane != lane) continue;
                CVector omn, omx;
                GetEntityDimensions(o, omn, omx);
                float vo = CVector2D::Dot(o->m_vecMoveSpeed.To2D(), d) * GAME_SPEED_TO_MPS;
                const ExtendedData& ox = vehExData.Get(o);
                // Behind us: it may speed up to what it wants; ahead: it may slow down (keeps its speed, at best)
                float voMax = lon0 < 0.0f && IsDrivenByMod(ox) ? std::max(vo, ox.desiredSpeed) : vo;
                float bodies = halfLength + omx.y + 2.0f;
                float vMe = speed, lonMe = 0.0f, lonO = lon0, vO = vo;
                for (float t = 0.0f; t <= T; t += 0.25f) {
                    vMe = std::min(vMe + passAccel * 0.25f, vPass);
                    lonMe += vMe * 0.25f;
                    vO = lon0 < 0.0f ? std::min(vO + 2.5f * 0.25f, voMax) : vO;
                    lonO += vO * 0.25f;
                    float gapAhead = lonO - lonMe, rel = vMe - vO;
                    // Ahead of us in that lane: room to brake to its speed; behind: it must not reach us
                    if (gapAhead > 0.0f && gapAhead < bodies + std::max(0.0f, rel) * 1.0f + 4.0f) return false;
                    if (gapAhead <= 0.0f && -gapAhead < bodies + std::max(0.0f, -rel) * 1.2f + 3.0f) return false;
                }
            }
            return true;
        };
        // Moving over to the right to let a faster one by: room to slot in, someone behind in that lane not much faster
        auto yieldSafe = [&](int lane) {
            const LaneInfo& info = laneInfo[lane];
            bool frontOk = info.frontGap > std::max(6.0f, speed * 0.5f) || (info.frontSpeed > speed - 2.0f && info.frontGap > 4.0f);
            float closing = info.backSpeed - speed;
            bool backOk = info.backGap > std::max(5.0f, speed * 0.25f) && (closing <= 0.0f || info.backGap / closing > 2.0f);
            return frontOk && backOk;
        };
        // Overtake: the vehicle ahead is clearly slower than we want to go (and moving: queues at lights/jams aren't
        // overtaken) and we've been behind it for a while
        // Decided quickly (it waited 5 s behind a slow car before): 0.8-2.5 s by aggression
        const bool wantsToOvertake = blocked && xdata.blockedTime >= Lerp(1.5f, 0.4f, aggr);
        // A lane change takes changeDuration s: not if the next junction comes before it's done (except for the route)
        // The smoothstep's peak lateral speed is 1.5 x lane / duration: from the driver (2.2 - 3.4 m/s) but keeping the
        // path angle natural (tan 7 deg ~ 0.12 of the speed). 3-4 s at freeway speed; slower ones take longer
        const float latSpeed = std::max(std::min(Lerp(2.2f, 3.4f, aggr), speed * 0.12f), 0.8f);
        // At most 3.5 s: a slow car taking 5 s across the lanes of a freeway had someone fast arrive behind it mid-way
        const float changeDuration = std::clamp(1.5f * LANE_WIDTH / latSpeed, 2.2f, 3.5f);
        // Overtakes: out of the slipstream quickly and decisively, 1.8-2.6 s (1.3 s cut into a car beside it in a freeway
        // curve before the side check could see it)
        const float overtakeLat = std::max(std::min(Lerp(3.1f, 4.5f, aggr), speed * 0.16f), 1.2f);
        const float overtakeDuration = std::clamp(1.5f * LANE_WIDTH / overtakeLat, 1.8f, 2.6f);
        auto durationFor = [&](unsigned char action) { return action == 1 ? overtakeDuration : changeDuration; };
        const bool roomToChange = xdata.requiredDist > speed * changeDuration + 40.0f;
        auto betterThanMine = [&](int lane) {
            return laneInfo[lane].frontGap > mine.frontGap + 15.0f || laneInfo[lane].frontSpeed > mine.frontSpeed + 3.0f;
        };
        // Priority on the board. The route's lane first (missing the exit is worse than a slower lap), then the overtakes:
        // the more it's held up (speed it wants minus the speed of the one ahead), the longer it's been waiting and the
        // more aggressive, the higher. Moving back to the right last
        auto priorityOf = [&](unsigned char action) {
            float pr = 0.0f;
            if (action == 4) pr = 40.0f;                                            // route
            else if (action == 1) pr = 10.0f + std::max(desired - mine.frontSpeed, 0.0f) * MPS_TO_KMH * 0.25f +
                std::min(xdata.blockedTime, 10.0f) + aggr * 5.0f;                  // overtake
            else if (action == 5) pr = 20.0f;                                       // making way for a faster car
            else pr = 2.0f;                                                         // keep right, fast lane
            return pr;
        };
        // Ask the board for the road this lane change needs; signal only if it's granted
        auto signal = [&](eLaneState state, int lane, unsigned char action) {
            // Told no: asks again in a second (it asked on every sensor tick: thousands of "denied" for a hundred waits)
            if (cfgLaneBoard && now - xdata.lastBoardDenied < 1000) return false;
            if (cfgLaneBoard) {
                float lonSelf = CVector2D::Dot(pos - linkPos, d);
                // From ~0.5 s behind it to where it'll be most of the way through the change (a box of the whole manoeuvre
                // at speed was ~110 m long and blocked everyone around it)
                float lonStart = lonSelf - halfLength - std::max(5.0f, speed * 0.5f);
                float lonEnd = lonSelf + halfLength + speed * durationFor(action) * 0.6f + 5.0f;
                float latCenter = (ow + (myLane + lane) * 0.5f) * LANE_WIDTH;
                LaneReservation request;
                request.veh = veh;
                request.center = linkPos + d * ((lonStart + lonEnd) * 0.5f) + r * latCenter;
                request.axis = d;
                request.halfLen = (lonEnd - lonStart) * 0.5f;
                request.halfWid = (std::abs(lane - myLane) + 1) * LANE_WIDTH * 0.5f - 0.6f;
                request.z = veh->GetPosition().z;
                request.expires = now + 2000;
                request.priority = priorityOf(action);
                if (!RequestLaneSpace(request)) { // someone else has that space: not now
                    if (now - xdata.lastBoardDenied > 3000) boardDeniedAttempts++;
                    xdata.lastBoardDenied = now;
                    return false;
                }
            }
            xdata.laneState = state;
            xdata.pendingLane = lane;
            xdata.pendingAction = action;
            xdata.signalStart = now;
            // Indicator on before moving: shorter for overtakes (the others see it, the board already gave it the space)
            xdata.signalUntil = now + static_cast<unsigned int>(action == 1 ? Lerp(600.0f, 300.0f, aggr) : Lerp(1500.0f, 500.0f, aggr));
            SetIndicator(xdata, lane > myLane ? 1 : -1, 6000);
            return true;
        };
        // Starts the lane change: steered smoothly by FollowPathHook, the game's lane is only set when it's done
        auto change = [&](int lane, unsigned char action, eLaneState after) {
            xdata.changeTarget = lane;
            xdata.changeOrigin = myLane;
            xdata.changeFromF = myLaneF;
            xdata.changeStart = now;
            xdata.changeDuration = durationFor(action);
            xdata.changeAfter = after;
            xdata.laneState = LANE_CHANGING;
            xdata.lastLaneChange = now;
            CommitLaneSpace(veh);
            xdata.laneAction = action;
            xdata.pendingLane = -1;
            SetIndicator(xdata, lane > myLane ? 1 : -1, static_cast<unsigned int>(durationFor(action) * 1000.0f) + 300); // on while moving over
        };

        switch (xdata.laneState) {
        case LANE_SIGNAL_OVERTAKE:
        case LANE_SIGNAL_MOVE: {
            int lane = xdata.pendingLane;
            bool overtake = xdata.laneState == LANE_SIGNAL_OVERTAKE;
            bool prepare = xdata.pendingAction == 4;
            // Someone with a higher priority took that space: give it up (and ask again a bit later)
            if (TakeBoardEviction(veh)) {
                xdata.laneState = LANE_KEEP;
                xdata.pendingLane = -1;
                xdata.indicatorUntil = now;
                xdata.lastBoardDenied = now;
                signalCancelCount++;
                return;
            }
            SetIndicator(xdata, lane > myLane ? 1 : -1, 500);
            HoldLaneSpace(veh, now + 1500);
            if (now < xdata.signalUntil) return;
            bool stillWanted = lane >= 0 && lane != myLane && (overtake ? blocked && zoneOk : (prepare ? requiredLane >= 0 : zoneOk));
            stillWanted &= speed > (prepare ? (zipper ? 1.5f : MIN_PREPARE_SPEED) : MIN_LANE_CHANGE_SPEED);
            bool safeNow = xdata.pendingAction == 5 ? yieldSafe(lane) :
                (overtake && cfgV2VOvertakes ? overtakeClear(lane) : laneSafe(lane));
            if (stillWanted && safeNow && SideClear(veh, xdata, lane > myLane ? 1 : -1, speed)) {
                change(lane, xdata.pendingAction, overtake ? LANE_OVERTAKING : LANE_KEEP);
                if (overtake) {
                    xdata.overtakeTarget = mine.frontVeh;
                    xdata.overtakeStart = now;
                    if (prof.keepRight >= 0 && cfgKeepRight && xdata.returnLane < 0) xdata.returnLane = myLane;
                    overtakeCount++;
                }
                else {
                    if (lane == xdata.returnLane || lane > myLane) xdata.returnLane = -1;
                    if (xdata.pendingAction == 2) keepRightCount++;
                    else if (xdata.pendingAction == 5) { passGivenCount++; StartPassMeasure(xdata.passRequester, true); }
                    else if (xdata.pendingAction == 3) fastLaneCount++;
                    else if (prepare) prepareLaneCount++;
                }
                return;
            }
            // Not clear yet: keep signalling a while (the others may make room); a lane the route needs is waited for longer
            if (stillWanted && now - xdata.signalStart < (prepare ? 5000u : 3000u)) return;
            xdata.laneState = LANE_KEEP;
            xdata.pendingLane = -1;
            xdata.indicatorUntil = now;
            signalCancelCount++;
            ReleaseLaneSpace(veh);
            return;
        }
        case LANE_CHANGING: {
            HoldLaneSpace(veh, now + 1500);
            // Almost stopped, or it hit something: the lane change is over, the game's lane (the old one) stays
            bool hitSomething = now - veh->m_nLastCollisionTime < 300 && veh->m_pDamageEntity;
            if (speed < 3.0f || hitSomething) {
                sideAbortCount++;
                if (xdata.changeAfter == LANE_OVERTAKING) overtakeAbortCount++;
                xdata.laneState = LANE_KEEP;
                xdata.overtakeTarget = nullptr;
                xdata.indicatorUntil = now;
                xdata.lastLaneChange = now;
                return;
            }
            // Watching the side it's moving to: someone there -> back to the lane it came from. Also someone coming up fast
            // in that lane (less than 1.5 s away) while it's not halfway yet: the side rays only reach 25 m back
            float lanesLeft = std::clamp(std::abs(xdata.changeTarget - myLaneF), 0.25f, 0.9f);
            bool fastBehind = false;
            if (xdata.changeTarget >= 0 && xdata.changeTarget < lanes && lanesLeft > 0.5f) {
                const LaneInfo& target = laneInfo[xdata.changeTarget];
                float closing = target.backSpeed - speed;
                fastBehind = target.backVeh && closing > 3.0f && target.backGap / closing < 1.5f;
            }
            if (xdata.changeTarget != xdata.changeOrigin && (fastBehind ||
                !SideClear(veh, xdata, xdata.changeTarget > xdata.changeOrigin ? 1 : -1, speed, lanesLeft))) {
                sideAbortCount++;
                if (xdata.changeAfter == LANE_OVERTAKING) overtakeAbortCount++;
                xdata.changeTarget = xdata.changeOrigin;
                xdata.changeFromF = myLaneF;
                xdata.changeStart = now;
                xdata.changeDuration = 1.5f;
                xdata.changeAfter = LANE_KEEP;
                xdata.overtakeTarget = nullptr;
                xdata.indicatorUntil = now;
            }
            return;
        }
        case LANE_OVERTAKING: {
            CVehicle* target = xdata.overtakeTarget;
            if (!target || !CPools::ms_pVehiclePool->IsObjectValid(target)) { overtakeEndLost++; xdata.laneState = LANE_KEEP; xdata.overtakeTarget = nullptr; return; }
            CVector targetMin, targetMax;
            GetEntityDimensions(target, targetMin, targetMax);
            float lon = CVector2D::Dot(target->GetPosition().To2D() - pos, d);
            if (lon < -(halfLength + targetMax.y + std::max(6.0f, speed * 0.5f))) { // well behind us: passed
                overtakeDoneCount++;
                xdata.laneState = LANE_KEEP;
                xdata.overtakeTarget = nullptr;
            }
            else if (now - xdata.overtakeStart > 30000 || lon > 150.0f) { // it didn't work out (it sped up, we got stuck)
                overtakeAbortCount++;
                xdata.laneState = LANE_KEEP;
                xdata.overtakeTarget = nullptr;
            }
            // The route needs another lane: only when it's getting close (it used to end most overtakes halfway)
            if (requiredLane < 0 || xdata.requiredDist > prepareDist * 0.6f) return;
            if (xdata.laneState == LANE_OVERTAKING) overtakeEndRoute++;
            xdata.laneState = LANE_KEEP;
            xdata.overtakeTarget = nullptr;
            break;
        }
        default: break;
        }

        // LANE_KEEP
        const unsigned int sinceChange = now - xdata.lastLaneChange;
        // Lane changes only while driving: at low speed the smooth S needs the wheel turned all the way (a freshly
        // spawned car turned into the wall next to it)
        if (speed < (zipper ? 1.5f : MIN_PREPARE_SPEED) || (requiredLane < 0 && speed < MIN_LANE_CHANGE_SPEED)) return;

        // 1. The route needs another lane: one lane at a time towards it (also close to the junction, not inside it)
        if (requiredLane >= 0) {
            xdata.returnLane = -1;
            bool nearDrop = xdata.dropLanes > 0 && xdata.dropDist < 15.0f && speed > 6.0f; // crawling: merge right there
            if (myLane != requiredLane && sinceChange >= 2500 && !nearJunction && !nearDrop) {
                signal(LANE_SIGNAL_MOVE, myLane + (requiredLane > myLane ? 1 : -1), 4);
            }
            if (passRequested) CountPassRefusal(xdata, 6);
            return; // in the lane it needs: stays there
        }
        // Someone faster asks for the way: move over to the right (if there is a right lane). Into the slower lane a smaller
        // gap is enough, and curves and lights further on don't matter (only ~13% of the requests were answered)
        if (passRequested) {
            int why = -1;
            bool lightClose = xdata.lightColor != LIGHT_NONE && xdata.lightDist < 50.0f;
            if (myLane >= lanes - 1) why = 0;
            else if (nearJunction || lightClose) why = 1;
            else if (sinceChange < 1000) why = 2;
            else if (!yieldSafe(myLane + 1)) why = 3;
            else if (!signal(LANE_SIGNAL_MOVE, myLane + 1, 5)) why = 4;
            else return;
            CountPassRefusal(xdata, why);
        }
        if (!zoneOk) return;
        const unsigned int interval = static_cast<unsigned int>(prof.laneChangeInterval * 1000.0f);

        if (!roomToChange) return;
        // 2. Overtake: by the left (left lane = overtaking, right lane = normal traffic); only fleeing drivers pass on the right
        // Not with the route's turn/exit coming before it can be done (~8 s to pass and get back)
        // A left turn needs the left lane anyway; a right one only when there's no time to pass and get back (it asked
        // for ~500 m at 90 km/h and blocked most overtakes)
        const bool overtakeFits = xdata.requiredSide <= 0 || xdata.requiredDist > prepareDist * 0.5f + speed * 6.0f;
        if (wantsToOvertake && !overtakeFits) overtakeSkippedForRoute++;
        // Always-right drivers don't overtake; asked for the way, they move over (below)
        // Overtakes don't wait the whole lane change interval (that's against weaving, not against passing): 4 s at most
        const unsigned int overtakeInterval = std::min(interval, 4000u);
        if (wantsToOvertake && overtakeFits && prof.keepRight != 2 && sinceChange >= overtakeInterval) {
            const int candidates[2] = { myLane - 1, myLane + 1 };
            for (int cand : candidates) {
                if (cand < 0 || cand >= lanes || (cand > myLane && !prof.runRedLights)) continue; // right: only when fleeing
                // Teamwork: the one behind already started passing (into that lane): let it go by first
                if (laneInfo[cand].overtakerBehind) { overtakeWaitCount++; continue; }
                if (cfgV2VOvertakes ? !overtakeClear(cand) : !laneSafe(cand)) continue;
                if (signal(LANE_SIGNAL_OVERTAKE, cand, 1)) return;
                overtakeWaitCount++; // the board said no: someone else is using that space
            }
        }

        // 3. Back to the lane it left after an overtake, once that lane won't make it overtake again right away
        if (xdata.returnLane >= 0 && xdata.returnLane != myLane && sinceChange >= 3000) {
            const LaneInfo& back = laneInfo[xdata.returnLane];
            bool worthIt = back.frontGap > std::max(40.0f, speed * 3.0f) || back.frontSpeed >= desired * prof.overtakeRatio;
            if (laneSafe(xdata.returnLane) && back.backGap > std::max(12.0f, speed * 0.6f) && worthIt) {
                signal(LANE_SIGNAL_MOVE, xdata.returnLane, 2);
                return;
            }
        }
        if (xdata.returnLane == myLane) xdata.returnLane = -1;

        // 5. Always-right drivers: back to the right whenever it's safe, traffic there or not
        if (cfgKeepRight && prof.keepRight == 2 && myLane < lanes - 1 && sinceChange >= 3000) {
            if (laneSafe(myLane + 1) && laneInfo[myLane + 1].backGap > std::max(15.0f, speed)) signal(LANE_SIGNAL_MOVE, myLane + 1, 2);
            return;
        }

        if (!cfgKeepRight || sinceChange < interval * 3 / 2) return;
        // Keep right: the lane to the right is free for a good while
        if (prof.keepRight == 1 && myLane < lanes - 1) {
            const LaneInfo& right = laneInfo[myLane + 1];
            bool free = right.frontGap > std::max(80.0f, speed * 5.0f) || right.frontSpeed >= desired * 0.95f;
            if (free && laneSafe(myLane + 1) && right.backGap > std::max(15.0f, speed)) signal(LANE_SIGNAL_MOVE, myLane + 1, 2);
        }
        // Fast lane: free and better than ours
        else if (prof.keepRight == -1 && myLane > 0) {
            const LaneInfo& left = laneInfo[myLane - 1];
            bool free = left.frontGap > std::max(80.0f, speed * 5.0f);
            if (free && betterThanMine(myLane - 1) && laneSafe(myLane - 1)) signal(LANE_SIGNAL_MOVE, myLane - 1, 3);
        }
    }

    static bool ThereIsObjectHere(const CVector& coord, float radius, int modelStart, int modelEnd) {
        for (int index = 0; index < CPools::ms_pObjectPool->m_nSize; ++index) {
            if (auto obj = CPools::ms_pObjectPool->GetAt(index)) {
                if (obj->m_nModelIndex >= modelStart && obj->m_nModelIndex <= modelEnd &&
                    obj->GetPosition().Distance(coord) <= radius)
                {
                    return true;
                }
            }
        }
        return false;
    }

    static bool FixGroundStuck(CVehicle* vehicle, const CVector& modelMin, const CVector& modelMax) {
        bool foundGround;
        for (float y : { modelMin.y * 0.9f, modelMax.y * 0.9f }) {
            CVector coord = GetWorldCoordWithOffset(vehicle, { 0.0f, y, 0.0f });
            FindGroundZ(coord, &foundGround);
            if (!foundGround) {
                // Ground found above this point = the vehicle is sunk into it
                FindGroundZ({ coord.x, coord.y, coord.z + modelMax.z }, &foundGround);
                if (foundGround) return ReposVehicle(vehicle);
            }
        }
        return false;
    }

    static void PlaceVehicle(CVehicle* vehicle, const CVector& coord) {
        vehicle->SetPosn(coord);
        vehicle->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f);
        vehicle->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f);
        int subClass = vehicle->m_nVehicleSubClass;
        if (subClass == VEHICLE_AUTOMOBILE || subClass == VEHICLE_MTRUCK || subClass == VEHICLE_QUAD) {
            reinterpret_cast<CAutomobile*>(vehicle)->PlaceOnRoadProperly();
        }
    }

    // Puts the vehicle upright on its current route node, facing the next one
    static bool PutBackOnRoute(CVehicle* vehicle) {
        CPathNode* curr = GetNode(vehicle->m_autoPilot.m_currentAddress);
        CPathNode* next = GetNode(vehicle->m_autoPilot.m_startingRouteNode);
        if (!curr) return false;
        CVector coord = GetNodePos(curr);
        bool foundGround;
        float newZ = FindGroundZ({ coord.x, coord.y, coord.z + 2.0f }, &foundGround);
        if (!foundGround) return false;
        coord.z = newZ + 0.5f;
        if (next) {
            CVector dir = GetNodePos(next) - coord;
            vehicle->SetHeading(CGeneral::GetATanOfXY(dir.x, dir.y) - 1.5707964f); // also makes it upright
        }
        PlaceVehicle(vehicle, coord);
        vehicle->m_autoPilot.m_nTempAction = 0;
        return true;
    }

    static bool IsBlocked(CVehicle* vehicle, const CVector& from, const CVector& to) {
        CColPoint colPoint;
        CEntity* entity = nullptr;
        CWorld::pIgnoreEntity = vehicle;
        bool hit = CWorld::ProcessLineOfSight(from, to, colPoint, entity, true, true, true, true, false, false, false, false);
        CWorld::pIgnoreEntity = nullptr;
        return hit && (entity == nullptr || entity->m_nType != ENTITY_TYPE_BUILDING || colPoint.m_vecNormal.z < cfgWallNormalZ);
    }

    // A vehicle with a driver (or a ped) just in front: we're in a queue, it will move on its own
    static bool IsQueuedBehindTraffic(CVehicle* vehicle) {
        CVector modelMin, modelMax;
        GetEntityDimensions(vehicle, modelMin, modelMax);
        for (float x : { modelMin.x * 0.6f, 0.0f, modelMax.x * 0.6f }) {
            CColPoint colPoint;
            CEntity* entity = nullptr;
            CWorld::pIgnoreEntity = vehicle;
            bool hit = CWorld::ProcessLineOfSight(GetWorldCoordWithOffset(vehicle, { x, modelMax.y, 0.0f }),
                GetWorldCoordWithOffset(vehicle, { x, modelMax.y + 5.0f, 0.0f }), colPoint, entity,
                false, true, true, false, false, false, false, false);
            CWorld::pIgnoreEntity = nullptr;
            if (!hit || !entity) continue;
            if (entity->m_nType == ENTITY_TYPE_PED) return true;
            if (entity->m_nType == ENTITY_TYPE_VEHICLE && static_cast<CVehicle*>(entity)->m_pDriver) return true;
        }
        return false;
    }

    // Crash recovery: back off from what we hit (turning away from it), or put the vehicle back on the road
    // when it's upside down/hopelessly stuck - only when the player can't see it happen
    static void UpdateCrashRecovery(CVehicle* vehicle, ExtendedData& xdata, bool isBike, float dt) {
        CAutoPilot& ap = vehicle->m_autoPilot;
        float speed = vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;

        // Backing off: never faster than ~14 km/h (the game reverses with 75% gas: 27 km/h after a second, into whatever is behind)
        // Any of the game's temp actions (going round a blocked lane reversed at 27 km/h into the queue behind), and not at
        // all with a vehicle right behind
        const float fwdSpeed = CVector::Dot(vehicle->m_vecMoveSpeed, vehicle->GetForward()) * GAME_SPEED_TO_MPS;
        if (ap.m_nTempAction != 0 && fwdSpeed < -0.5f) {
            bool stop = fwdSpeed < -4.0f;
            if (!stop) {
                CVector mn, mx;
                GetEntityDimensions(vehicle, mn, mx);
                CColPoint colPoint;
                CEntity* entity = nullptr;
                CWorld::pIgnoreEntity = vehicle;
                stop = CWorld::ProcessLineOfSight(GetWorldCoordWithOffset(vehicle, { 0.0f, mn.y + 0.3f, (mn.z + mx.z) * 0.5f }),
                    GetWorldCoordWithOffset(vehicle, { 0.0f, mn.y - 1.0f + fwdSpeed * 0.3f, (mn.z + mx.z) * 0.5f }),
                    colPoint, entity, false, true, false, false, false, false, false, false);
                CWorld::pIgnoreEntity = nullptr;
            }
            if (stop) {
                ap.m_nTempAction = 0; // done: far enough (ending its time isn't enough, the game keeps reversing)
                ap.m_nTempActionTime = CTimer::m_snTimeInMilliseconds;
            }
        }

        if (!isBike && vehicle->GetUp().z < 0.3f && speed < 2.0f) { // upside down or on its side
            xdata.flippedTime += dt;
            xdata.stuckTime = 0.0f;
            if (xdata.flippedTime > cfgFlipResetTime && !vehicle->GetIsOnScreen() && PutBackOnRoute(vehicle)) {
                lg << "Recovery: flipped vehicle put back on the road\n";
                xdata.flippedTime = 0.0f;
            }
            return;
        }
        xdata.flippedTime = 0.0f;

        bool tryingToMove = vehicle->m_fGasPedal > 0.25f && ap.m_nTempAction == 0 && xdata.targetSpeed > 3.0f;
        // Waiting is not being stuck: red/yellow light ahead, or a vehicle with a driver right in front (queue, jam)
        bool waitingForLight = xdata.lightColor != LIGHT_NONE && xdata.lightColor != LIGHT_GREEN && xdata.lightDist < 80.0f;
        if (tryingToMove && speed < 1.0f && !waitingForLight && !IsQueuedBehindTraffic(vehicle)) xdata.stuckTime += dt;
        else xdata.stuckTime = std::max(xdata.stuckTime - dt * 2.0f, 0.0f);
        if (xdata.stuckTime < cfgStuckTime) return;
        xdata.stuckTime = 0.0f;

        // Never back into someone: if there's something right behind, wait and try again later
        {
            CVector modelMin, modelMax;
            GetEntityDimensions(vehicle, modelMin, modelMax);
            bool rearBlocked = false;
            for (float x : { modelMin.x * 0.7f, 0.0f, modelMax.x * 0.7f }) {
                rearBlocked |= IsBlocked(vehicle, GetWorldCoordWithOffset(vehicle, { x, modelMin.y, 0.0f }),
                    GetWorldCoordWithOffset(vehicle, { x, modelMin.y - 4.0f, 0.0f }));
            }
            if (rearBlocked) return;
        }

        unsigned int now = CTimer::m_snTimeInMilliseconds;
        if (now - xdata.lastRecoveryTime > 30000) xdata.recoveries = 0;
        xdata.recoveries++;
        xdata.lastRecoveryTime = now;

        if (xdata.recoveries > 3 && !vehicle->GetIsOnScreen() && PutBackOnRoute(vehicle)) {
            lg << "Recovery: stuck vehicle put back on the road\n";
            xdata.recoveries = 0;
            return;
        }

        // Which front corner is blocked? Reversing with the wheels turned to one side swings the nose to the other
        CVector modelMin, modelMax;
        GetEntityDimensions(vehicle, modelMin, modelMax);
        bool leftBlocked = IsBlocked(vehicle, GetWorldCoordWithOffset(vehicle, { modelMin.x * 0.8f, modelMax.y, 0.0f }),
            GetWorldCoordWithOffset(vehicle, { modelMin.x - 0.5f, modelMax.y + 3.0f, 0.0f }));
        bool rightBlocked = IsBlocked(vehicle, GetWorldCoordWithOffset(vehicle, { modelMax.x * 0.8f, modelMax.y, 0.0f }),
            GetWorldCoordWithOffset(vehicle, { modelMax.x + 0.5f, modelMax.y + 3.0f, 0.0f }));

        bool noseToRight;
        if (leftBlocked != rightBlocked) noseToRight = leftBlocked;
        else {
            // Turn towards the route
            CPathNode* next = GetNode(ap.m_startingRouteNode);
            CVector2D toNext = next ? (GetNodePos(next) - vehicle->GetPosition()).To2D() : CVector2D(0.0f, 0.0f);
            noseToRight = CVector2D::Cross(vehicle->GetForward().To2D(), toNext) < 0.0f;
        }
        ap.m_nTempAction = noseToRight ? TEMPACT_REVERSE_LEFT : TEMPACT_REVERSE_RIGHT;
        ap.m_nTempActionTime = now + 1200 + xdata.recoveries * 400;
    }

    static bool ReposVehicle(CVehicle* vehicle) {
        const CVector origin = vehicle->GetPosition();
        lg << "Fixing ground stuck at " << origin.x << " " << origin.y << " " << origin.z << "\n";

        // Try spots around the vehicle, increasing the radius each attempt
        for (int i = 0; i < cfgGroundStuckAttempts; i++) {
            float radius = 0.5f + i * 0.5f;
            CVector coord = origin;
            coord.x += CGeneral::GetRandomNumberInRange(-radius, radius);
            coord.y += CGeneral::GetRandomNumberInRange(-radius, radius);
            coord.z += 2.0f;

            bool foundGround;
            float newZ = FindGroundZ(coord, &foundGround);
            if (foundGround && std::abs(newZ - origin.z) < 4.0f) {
                coord.z = newZ + 0.5f;
                PlaceVehicle(vehicle, coord);
                lg << "FIXED (attempt " << i + 1 << ")\n";
                return true;
            }
        }

        // Fallback: put it back on the AI route, facing the next node
        CPathNode* curr = GetNode(vehicle->m_autoPilot.m_currentAddress);
        CPathNode* next = GetNode(vehicle->m_autoPilot.m_startingRouteNode);
        if (curr) {
            CVector coord = GetNodePos(curr);
            bool foundGround;
            float newZ = FindGroundZ({ coord.x, coord.y, coord.z + 2.0f }, &foundGround);
            if (foundGround) {
                coord.z = newZ + 0.5f;
                PlaceVehicle(vehicle, coord);
                if (next) {
                    CVector dir = GetNodePos(next) - coord;
                    vehicle->SetHeading(CGeneral::GetATanOfXY(dir.x, dir.y) - 1.5707964f);
                }
                lg << "FIXED (moved to route node)\n";
                return true;
            }
        }

        lg << "FAILED\n";
        return false;
    }

    // Asking for the way: flashing the high beams (two quick flashes, then a pause)
    static void DrawPassFlashes() {
        if (!cfgTurnSignals) return;
        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        const unsigned int phase = now % 1200;
        if (!(phase < 150 || (phase >= 300 && phase < 450))) return;
        const CVector camPos = TheCamera.GetPosition();
        int drawn = 0;
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize && drawn < 10; i++) {
            CVehicle* vehicle = CPools::ms_pVehiclePool->GetAt(i);
            if (!vehicle || !vehicle->m_pDriver) continue;
            const ExtendedData& xdata = vehExData.Get(vehicle);
            if (now - xdata.lastPassRequest > 500 || !IsDrivenByMod(xdata)) continue;
            if (camPos.Distance(vehicle->GetPosition()) > 120.0f || !vehicle->GetIsOnScreen()) continue;
            CVector mn, mx;
            GetEntityDimensions(vehicle, mn, mx);
            float z = mn.z + (mx.z - mn.z) * 0.4f;
            const CVector lights[2] = { { mn.x + 0.35f, mx.y - 0.05f, z }, { mx.x - 0.35f, mx.y - 0.05f, z } };
            for (int c = 0; c < 2; c++) {
                unsigned int id = reinterpret_cast<unsigned int>(vehicle) + 0x5200 + c;
                CCoronas::RegisterCorona(id, nullptr, 230, 240, 255, 255, GetWorldCoordWithOffset(vehicle, lights[c]), 0.9f, 150.0f,
                    CORONATYPE_HEADLIGHT, FLARETYPE_HEADLIGHTS, false, false, 0, 0.0f, false, 0.3f, 0, 255.0f, false, false);
            }
            drawn++;
        }
    }

    static void DrawIndicators() {
        DrawPassFlashes();
        if (!cfgTurnSignals) return;
        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        if ((now / 350) % 2) return; // off half of the time
        const CVector camPos = TheCamera.GetPosition();
        int drawn = 0;
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize && drawn < 20; i++) {
            CVehicle* vehicle = CPools::ms_pVehiclePool->GetAt(i);
            if (!vehicle || !vehicle->m_pDriver) continue;
            const ExtendedData& xdata = vehExData.Get(vehicle);
            if (xdata.indicator == 0 || now >= xdata.indicatorUntil || !IsDrivenByMod(xdata)) continue;
            if (camPos.Distance(vehicle->GetPosition()) > 90.0f || !vehicle->GetIsOnScreen()) continue;
            CVector mn, mx;
            GetEntityDimensions(vehicle, mn, mx);
            float x = xdata.indicator > 0 ? mx.x - 0.1f : mn.x + 0.1f;
            float z = (mn.z + mx.z) * 0.5f;
            const CVector corners[2] = { { x, mx.y - 0.15f, z }, { x, mn.y + 0.15f, z } };
            for (int c = 0; c < 2; c++) {
                unsigned int id = reinterpret_cast<unsigned int>(vehicle) + 0x5100 + c; // unique per vehicle and corner
                CCoronas::RegisterCorona(id, nullptr, 255, 140, 0, 255, GetWorldCoordWithOffset(vehicle, corners[c]), 0.35f, 90.0f,
                    CORONATYPE_HEADLIGHT, FLARETYPE_NONE, false, false, 0, 0.0f, false, 0.2f, 0, 255.0f, false, false);
            }
            drawn++;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Debug

    static void DrawLine(const CVector& a, const CVector& b, CRGBA color) {
        RwIm3DVertex verts[2];
        RwIm3DVertexSetPos(&verts[0], a.x, a.y, a.z);
        RwIm3DVertexSetPos(&verts[1], b.x, b.y, b.z);
        RwIm3DVertexSetRGBA(&verts[0], color.r, color.g, color.b, color.a);
        RwIm3DVertexSetRGBA(&verts[1], color.r, color.g, color.b, color.a);
        if (RwIm3DTransform(verts, 2, nullptr, 0)) {
            RwIm3DRenderLine(0, 1);
            RwIm3DEnd();
        }
    }

    static void DrawDebugText(CVehicle* vehicle, const CVector& modelMax, int line, const std::string& text, CRGBA color) {
        CVector fontPos3D = GetWorldCoordWithOffset(vehicle, { 0.0f, 0.0f, modelMax.z + 0.5f });
        RwV3d fontPos2D; float w, h;
        if (!CSprite::CalcScreenCoors({ fontPos3D.x, fontPos3D.y, fontPos3D.z }, &fontPos2D, &w, &h, true, true)) return;

        float scale = std::clamp(w / 120.0f, 0.25f, 0.6f);
        CFont::SetBackground(false, false);
        CFont::SetJustify(false);
        CFont::SetProportional(true);
        CFont::SetOrientation(ALIGN_CENTER);
        CFont::SetFontStyle(FONT_SUBTITLES);
        CFont::SetColor(color);
        CFont::SetEdge(1);
        CFont::SetScale(scale, scale * 2.0f);
        CFont::PrintString(fontPos2D.x, fontPos2D.y - line * scale * 36.0f, text.c_str());
    }

    // One short line per vehicle, in its profile's colour: profile, speed > target, what limits it, and what it's doing.
    // The watched vehicle also gets its sensors, route and a second line
    static void DrawCompactDebug(CVehicle* vehicle, ExtendedData& xdata, bool focus) {
        static const CRGBA profileColors[NUM_PROFILES] = { CRGBA(110, 200, 255, 255), CRGBA(120, 255, 120, 255), CRGBA(255, 255, 255, 255),
            CRGBA(255, 230, 60, 255), CRGBA(255, 150, 40, 255), CRGBA(255, 60, 60, 255), CRGBA(230, 90, 255, 255) };
        const unsigned int now = CTimer::m_snTimeInMilliseconds;
        CVector modelMin, modelMax;
        GetEntityDimensions(vehicle, modelMin, modelMax);
        float speedKmh = vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH;
        std::string tags;
        if (xdata.indicator != 0 && now < xdata.indicatorUntil) tags += xdata.indicator > 0 ? " >" : " <";
        if (xdata.laneState == LANE_SIGNAL_OVERTAKE || xdata.laneState == LANE_OVERTAKING ||
            (xdata.laneState == LANE_CHANGING && xdata.changeAfter == LANE_OVERTAKING)) tags += " PASS";
        else if (xdata.laneState == LANE_CHANGING) tags += " CHG";
        if (xdata.yieldTo && now - xdata.yieldTime < 600) tags += " YIELD";
        if (xdata.stuckTime > 0.5f) tags += " STUCK?";
        DrawDebugText(vehicle, modelMax, 0, Format("%s %.0f>%.0f %s%s", profileNames[xdata.profile], speedKmh, xdata.targetSpeed * MPS_TO_KMH,
            limiterNames[xdata.limiter], tags.c_str()), profileColors[xdata.profile]);
        if (!focus) return;

        for (int r = 0; r < xdata.numRays; r++) DrawLine(xdata.rayA[r], xdata.rayB[r], xdata.rayColor[r]);
        if (now - xdata.sideRayTime < 300) {
            for (int r = 0; r < xdata.numSideRays; r++) DrawLine(xdata.sideRayA[r], xdata.sideRayB[r], xdata.sideRayColor[r]);
        }
        int routeEnd = std::min(static_cast<int>(xdata.route.size()) - 1, xdata.routePos + 25);
        for (int p = std::max(xdata.routePos, 0); p < routeEnd; p++) {
            CPathNode* a = GetNode(xdata.route[p]);
            CPathNode* b = GetNode(xdata.route[p + 1]);
            if (a && b) DrawLine(GetNodePos(a) + CVector(0.0f, 0.0f, 1.5f), GetNodePos(b) + CVector(0.0f, 0.0f, 1.5f), CRGBA(255, 220, 0, 255));
        }
        std::string ahead = xdata.obstacle == OBSTACLE_NONE ? "free" : Format("%s %.0fm", obstacleNames[xdata.obstacle], xdata.gap);
        DrawDebugText(vehicle, modelMax, 1, Format("%s %.0f   lane %d/%d   %s   dest %.1f km", roadNames[xdata.roadType],
            std::min(xdata.roadLimit, 999.0f) * MPS_TO_KMH, (int)vehicle->m_autoPilot.m_nNextLane, xdata.lanesHere, ahead.c_str(),
            (xdata.destination - vehicle->GetPosition().To2D()).Magnitude() / 1000.0f), CRGBA(200, 200, 200, 255));
    }

    static void DrawDebug() {
        if (!cfgTestMode) return;

        CFont::SetBackground(false, false);
        CFont::SetJustify(false);
        CFont::SetProportional(true);
        CFont::SetOrientation(ALIGN_LEFT);
        CFont::SetFontStyle(FONT_SUBTITLES);
        CFont::SetColor({ 255, 200, 0, 255 });
        CFont::SetEdge(1);
        CFont::SetScale(0.4f, 0.9f);
        CFont::SetScale(0.3f, 0.7f);
        CFont::PrintString(10.0f, 10.0f, Format("RealTrafficFix %s - %d veh  [F9: %s]", cfgDebugLevel == 1 ? "compact" : "full",
            debugVehiclesProcessed, cfgDebugLevel == 1 ? "full" : "off").c_str());

        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)FALSE);
        RwRenderStateSet(rwRENDERSTATETEXTURERASTER, nullptr);
        RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)TRUE);

        const CVector camPos = TheCamera.GetPosition();
        // The vehicle being watched: the one closest to the camera (in the compact view only it gets lines and details)
        CVehicle* focus = nullptr;
        float focusDist = cfgDebugDistance;
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
            CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
            if (!v || !vehExData.Get(v).valid) continue;
            float dist = camPos.Distance(v->GetPosition());
            if (dist < focusDist) { focusDist = dist; focus = v; }
        }
        const bool compact = cfgDebugLevel == 1;
        // Lane change board: the road each driver has reserved (cyan)
        for (const LaneReservation& r : laneBoard) {
            CVector rc(r.center.x, r.center.y, r.z);
            if ((rc - camPos).Magnitude() > cfgDebugDistance * 3.0f) continue;
            if (compact && (!focus || (rc - focus->GetPosition()).Magnitude() > 60.0f)) continue;
            CVector2D side(-r.axis.y, r.axis.x);
            CVector2D c[4] = { r.center + r.axis * r.halfLen + side * r.halfWid, r.center + r.axis * r.halfLen - side * r.halfWid,
                r.center - r.axis * r.halfLen - side * r.halfWid, r.center - r.axis * r.halfLen + side * r.halfWid };
            for (int k = 0; k < 4; k++) {
                DrawLine(CVector(c[k].x, c[k].y, r.z), CVector(c[(k + 1) % 4].x, c[(k + 1) % 4].y, r.z), CRGBA(0, 220, 255, 255));
            }
        }
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
            CVehicle* vehicle = CPools::ms_pVehiclePool->GetAt(i);
            if (!vehicle) continue;
            ExtendedData& xdata = vehExData.Get(vehicle);
            if (!xdata.valid || camPos.Distance(vehicle->GetPosition()) > cfgDebugDistance) continue;
            if (compact) { DrawCompactDebug(vehicle, xdata, vehicle == focus); continue; }

            for (int r = 0; r < xdata.numRays; r++) DrawLine(xdata.rayA[r], xdata.rayB[r], xdata.rayColor[r]);
            if (CTimer::m_snTimeInMilliseconds - xdata.sideRayTime < 300) {
                for (int r = 0; r < xdata.numSideRays; r++) DrawLine(xdata.sideRayA[r], xdata.sideRayB[r], xdata.sideRayColor[r]);
            }

            // Racing line: bright when it's steering the vehicle, dim when the game is
            CRGBA lineColor = xdata.lineBlend > 0.5f ? CRGBA(0, 255, 120, 255) : CRGBA(0, 120, 60, 255);
            for (int p = 0; p + 1 < xdata.lineCount; p++) DrawLine(xdata.line[p], xdata.line[p + 1], lineColor);

            for (int p = 0; p + 1 < xdata.pathCount; p++) {
                CVector a = xdata.path[p] + CVector(0.0f, 0.0f, 1.0f);
                CVector b = xdata.path[p + 1] + CVector(0.0f, 0.0f, 1.0f);
                CRGBA color = (p + 1 == xdata.limitingNode) ? CRGBA(255, 0, 255, 255) : CRGBA(0, 180, 255, 255);
                DrawLine(a, b, color);
            }

            // Planned route (yellow), the next ~40 nodes
            int routeEnd = std::min(static_cast<int>(xdata.route.size()) - 1, xdata.routePos + 40);
            for (int p = std::max(xdata.routePos, 0); p < routeEnd; p++) {
                CPathNode* a = GetNode(xdata.route[p]);
                CPathNode* b = GetNode(xdata.route[p + 1]);
                if (a && b) DrawLine(GetNodePos(a) + CVector(0.0f, 0.0f, 1.5f), GetNodePos(b) + CVector(0.0f, 0.0f, 1.5f), CRGBA(255, 220, 0, 255));
            }

            CVector modelMin, modelMax;
            GetEntityDimensions(vehicle, modelMin, modelMax);
            const CAutoPilot& ap = vehicle->m_autoPilot;
            float speedKmh = vehicle->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * MPS_TO_KMH;
            CRGBA modeColor = xdata.bRacer ? CRGBA(255, 120, 0, 255) : CRGBA(255, 255, 255, 255);

            int line = 0;
            std::string obstacleText = xdata.obstacle == OBSTACLE_NONE ? "free" :
                Format("%s %.0fm lead %.0f", obstacleNames[xdata.obstacle], xdata.gap, xdata.leaderSpeed * MPS_TO_KMH);
            DrawDebugText(vehicle, modelMax, line++, obstacleText, xdata.obstacle == OBSTACLE_NONE ? CRGBA(0, 255, 0, 255) : CRGBA(255, 80, 80, 255));
            if (xdata.lightColor != LIGHT_NONE) {
                static const CRGBA lightColors[] = { CRGBA(0, 255, 0, 255), CRGBA(255, 220, 0, 255), CRGBA(255, 0, 0, 255) };
                CRGBA c = lightColors[xdata.lightColor];
                DrawLine(xdata.lightPos, xdata.lightPos + CVector(0.0f, 0.0f, 5.0f), c);
                DrawDebugText(vehicle, modelMax, line++, Format("light %s %.0fm max %.0f", lightNames[xdata.lightColor],
                    xdata.lightDist, std::min(xdata.lightSpeed, 999.0f) * MPS_TO_KMH), c);
            }
            if (xdata.headingSpeed < 999.0f && xdata.limiter == LIMIT_HEADING) {
                DrawDebugText(vehicle, modelMax, line++, Format("turning max %.0f", xdata.headingSpeed * MPS_TO_KMH), CRGBA(0, 220, 255, 255));
            }
            if (xdata.curveSpeed < 999.0f) {
                DrawDebugText(vehicle, modelMax, line++, Format("%s max %.0f in %.0fm", xdata.curveIsJunction ? "junction" : "curve",
                    xdata.curveSpeed * MPS_TO_KMH, xdata.curveDist), CRGBA(255, 0, 255, 255));
            }
            bool laneChangeRecent = CTimer::m_snTimeInMilliseconds - xdata.lastLaneChange < 3000;
            static const char* laneActionNames[] = { "", "  overtake", "  keep right", "  fast lane" };
            const char* laneText = xdata.laneState != LANE_KEEP ? laneStateNames[xdata.laneState] :
                (laneChangeRecent && xdata.laneAction < 4 ? laneActionNames[xdata.laneAction] : "");
            DrawDebugText(vehicle, modelMax, line++, Format("lane %d>%d of %d  style %d  %s", (int)ap.m_nCurrentLane, (int)ap.m_nNextLane,
                xdata.lanesHere, (int)ap.m_nCarDrivingStyle, laneText),
                (laneChangeRecent && xdata.laneAction) || xdata.laneState != LANE_KEEP ? CRGBA(0, 200, 255, 255) : CRGBA(200, 200, 200, 255));
            DrawDebugText(vehicle, modelMax, line++, Format("%.0f > %.0f km/h (%s)", speedKmh, xdata.targetSpeed * MPS_TO_KMH,
                limiterNames[xdata.limiter]), CRGBA(255, 255, 255, 255));
            DrawDebugText(vehicle, modelMax, line++, Format("road %s max %.0f  car max %.0f", roadNames[xdata.roadType],
                std::min(xdata.roadLimit, 999.0f) * MPS_TO_KMH, std::min(xdata.carTopSpeed, 999.0f) * MPS_TO_KMH),
                CRGBA(180, 220, 255, 255));
            if (xdata.crossSpeed < 999.0f) {
                DrawLine(xdata.crossPos, xdata.crossPos + CVector(0.0f, 0.0f, 4.0f), CRGBA(255, 120, 0, 255));
                DrawDebugText(vehicle, modelMax, line++, Format("cross traffic! stop in %.0fm", xdata.crossDist), CRGBA(255, 120, 0, 255));
            }
            if (xdata.aidFlags & ~AID_PEDALS) {
                DrawDebugText(vehicle, modelMax, line++, Format("%s%s%s%s%s", (xdata.aidFlags & AID_ABS) ? "ABS " : "", (xdata.aidFlags & AID_TC) ? "TC " : "",
                    (xdata.aidFlags & AID_ESC) ? "ESC " : "", (xdata.aidFlags & AID_COUNTERSTEER) ? "COUNTERSTEER " : "",
                    (xdata.aidFlags & AID_ACCEL) ? Format("accel %.1f/%.1f", xdata.measAccel, xdata.accelLimit).c_str() : ""), CRGBA(255, 160, 0, 255));
            }
            if (xdata.surfaceFactor < 0.95f || xdata.damageFactor < 1.0f || xdata.stuckTime > 0.5f) {
                DrawDebugText(vehicle, modelMax, line++, Format("surface %.0f%%  damage %.0f%%%s", xdata.surfaceFactor * 100.0f,
                    (1.0f - xdata.damageFactor) * 100.0f, xdata.stuckTime > 0.5f ? "  STUCK?" : ""), CRGBA(200, 150, 100, 255));
            }
            if (xdata.bRacer) {
                const ModelLimits& limits = GetModelLimits(vehicle);
                DrawDebugText(vehicle, modelMax, line++, Format("grip %.2fg  brake %.1f  (%d)  %s", limits.grip, limits.decel, limits.samples,
                    xdata.lineBlend > 0.5f ? "LINE" : (xdata.inCorridor ? "lane" : "OUT")), CRGBA(0, 255, 120, 255));
            }
            if (xdata.hasDestination) {
                const char* need = xdata.requiredSide > 0 ? "  needs RIGHT lane" : (xdata.requiredSide < 0 ? "  needs LEFT lane" : "");
                DrawDebugText(vehicle, modelMax, line++, Format("dest %.0fm  route %d/%d%s%s  junction in %.0fm%s",
                    (xdata.destination - vehicle->GetPosition().To2D()).Magnitude(), xdata.routePos, (int)xdata.route.size(),
                    xdata.routeComplete ? "" : " (partial)", xdata.blockedTime > 0.0f ? Format("  behind slower %.1fs", xdata.blockedTime).c_str() : "",
                    std::min(xdata.requiredDist, 999.0f), need), CRGBA(255, 220, 0, 255));
            }
            DrawDebugText(vehicle, modelMax, line++, Format("%s  aggr %.0f%%  want %.0f%s", profileNames[xdata.profile],
                xdata.aggression * 100.0f, xdata.desiredSpeed * MPS_TO_KMH, (xdata.aidFlags & AID_PEDALS) ? "  [pedals]" : ""), modeColor);
        }

        RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)TRUE);
    }
} realTrafficFix;

extern "C" void __declspec(dllexport) Ext_TogglePlayerAutoDrive(int toggle) {
    playerAutoDrive = (toggle == 1);
}
