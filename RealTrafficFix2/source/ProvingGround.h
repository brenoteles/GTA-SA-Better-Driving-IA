// Proving ground: a flat, empty test area (the proving grid, a platform built by tools/make_gridmap.py) where one car at a time drives
// test courses drawn on the ground (circles, slaloms, lane changes, open-loop steering inputs) with a steering controller
// set in an ini, while every frame of its state is written to a CSV. For learning how GTA SA's car physics behave and how a
// car follows a line, without recompiling: RealTrafficFix.proving.ini (scripts folder) is read again whenever it changes,
// and the program starts over.
//
// The test car is an ordinary AI car (STATUS_PHYSICS: the same tyre and grip code as the traffic) whose
// CCarCtrl::SteerAICarWithPhysics call (in CAutomobile::ProcessAI, 0x6B52A8) is replaced: this module sets the wheel,
// gas, brake and handbrake right before the physics. Driver = player puts the player at the wheel instead (STATUS_PLAYER:
// the game's player physics) and overrides the pad's inputs after CAutomobile::ProcessControlInputs.
// tools/proving.py writes the ini, starts the game and analyses the CSVs.

#pragma once

#include <random>
#include <map>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include "CSprite2d.h"

namespace ProvingGround {

static bool enabled = false;
static std::string iniPath, baseDir, runDir;
static FILETIME iniStamp{};
static unsigned int lastIniCheck = 0;
static std::ofstream summary;
static float lineRate = 3.0f; // [Steer] LineRate: the weight of the change of curvature in the racing line (0: curvature only)
static int lineTime = 2;       // [Steer] LineTime: minimum-time rounds on the racing line (0: minimum curvature only)

// --- Settings (ini) --------------------------------------------------------------------------------------------------

struct TestDef {
    std::string name, shape = "line";
    float out = 30.0f, hold = 25.0f, back = 25.0f;  // lanechange: m over, m there, m back
    float length = 300.0f, radius = 40.0f, spacing = 30.0f, amplitude = 3.0f, runIn = 60.0f;
    int dir = 1;                         // circles: 1 left, -1 right
    float duration = 30.0f;              // s per speed step (also ends at the end of an open course)
    std::vector<float> speeds{ 60.0f };  // km/h, one run each: the speed held (closed loop: the most it goes)
    float startSpeed = -1.0f;            // km/h at the start (flying start); < 0: the run's speed
    float gripG = -1.0f, brakeG = -1.0f; // speed planning for the course's curves (< 0: [Speed] GripG / BrakeG)
    float brakeTo = 0.0f;                // Pedal = brake: down to this speed (km/h), then held there
    float width = 0.0f;                  // m between the two edges of the road (> 0: a corridor; the car's body is judged
                                         // against its edges, not its centre against the centre line)
    std::string segments;                // Shape = track: "S 300 / L 90 30 / R 45 120" (straight m; left/right degrees radius)
    std::vector<CVector2D> points;       // shape = points (course frame, m)
    bool closed = false;
    // Open loop: the wheel follows a signal, the course is only drawn
    std::string steer = "none";          // none | step | ramp | sine | sweep | hold
    float steerAmp = 0.1f, steerStart = 2.0f, steerRate = 0.05f, freq = 0.5f, freqEnd = 3.0f;
    std::string pedal = "hold";          // hold (keep the speed) | full | coast | brake | lift
    float pedalStart = 2.0f, brakeLevel = 1.0f;
    bool handbrakeAt = false;
    bool hasStart = false;               // Start = x, y, heading: this test's own course frame (world, degrees from east)
    CVector2D start;
    float startHeading = 0.0f;
};

struct Settings {
    CVector origin{ 1700.0f, -2550.0f, 300.0f };
    float headingDeg = 0.0f;                          // the course's +x axis, degrees from east, counter-clockwise
    std::vector<int> cars{ 541 };
    std::vector<std::string> tests;
    std::map<std::string, TestDef> defs;
    std::string driver = "cjai";                      // cjai: CJ at the wheel, the AI's physics | ai: an AI driver | player: CJ, the player's physics
    bool grid = true, exitWhenDone = false, pause = false;
    float gridSize = 10.0f, gridExtent = 300.0f;
    // Steering
    std::string mode = "kinematic";                   // kinematic | pursuit | stanley | mod
    float lookTime = 0.6f, lookMin = 6.0f, lookMax = 60.0f, gain = 1.0f;
    float bandwidth = 1.5f, damping = 1.0f, preview = 0.03f;   // kinematic: rad/s, -, s
    float stanleyK = 1.5f, stanleySoft = 2.0f;
    bool feedForward = true;
    float maxSteer = 0.0f;                            // rad, 0: the handling's steering lock
    float steerRate = 0.0f;                           // rad/s, 0: unlimited
    float smoothing = 0.0f;                           // s (first order)
    float gripClampG = 0.0f;                          // > 0: never more wheel than this many g need (atan(L g G / v2))
    bool racer = true;                                // mode = mod: a racing driver's weights
    // Speed
    float speedKp = 0.35f, brakeKp = 0.25f, cruiseGas = 0.15f;
    float speedGripG = 0.0f;                          // > 0: slows for the course's curves to this many g
    float speedBrakeG = 0.8f;                         // ... braking for them at this many g
    bool tractionControl = true;
    bool abs = true;                                  // closed loop: brake under the wheels' lock point and the curve's grip                      // closed loop: no gas past the grip a curve leaves, none sliding
    float gripFactor = 0.9f;                          // a test's GripG = 0: this share of the car's computed AI grip
    // the grip by the kind of curve: x GripTight at a radius of 15 m and less, x 1 at 45 m, x GripFast at 150 m and more
    // (in between along the log of the radius); and how fast the car learns each kind itself while it drives (per s)
    float gripTight = 1.0f, gripFast = 1.0f, learnRate = 0.0f;
    // its own line inside the road (a course with Width): LineBias -1..1 of the room to one side (right..left), LineCut
    // -0.5..1 towards the inside of the curves (< 0: the outside); the room = the half width - the car's - 0.1 m
    float lineBias = 0.0f, lineCut = 0.0f;
    float lineRace = 1.0f; // its own line: how much of the racing line (RaceLine: wide in, the apex, wide out) it takes
    // mode gt4: how hard the speed profile accelerates (g); mode mpc: how far ahead it simulates (s), the weight of the
    // heading at the horizon
    float gtAccel = 0.6f, mpcHorizon = 0.8f, mpcWeight = 1.0f;
    float counterSteer = 0.8f; // neural2's teacher: its countersteer and yaw damping (Neural2Snapshot)
    // rpp: how much a bend ahead shortens the look-ahead; blend: the share of pursuit (the rest the kinematic law);
    // gap: how much its own line ahead pulls the chosen gap
    float rppCurv = 2.0f, blendW = 0.5f, gapBias = 0.5f;
    std::vector<float> nn;                            // mode neural: its weights (the GA's genome)
    // ABS: the pedal at AbsMargin of the wheels' lock point, x the friction circle's share by AbsCircle (0 = ignore the
    // curve, 1 = all of it); a lock takes AbsDrop off, AbsRecover a frame back
    float absMargin = 0.9f, absDrop = 0.15f, absRecover = 0.05f, absCircle = 1.0f;
    // TC: no gas past TcSlip rad of slip at the rear axle; in a curve at least TcMinGas, the friction circle's share by
    // TcCircle; spinning wheels take TcDrop off, TcRecover a frame back
    float tcSlip = 0.08f, tcMinGas = 0.15f, tcDrop = 0.1f, tcRecover = 0.03f, tcCircle = 1.0f;
    float abortOffset = 25.0f;
    float areaRadius = 1300.0f;                       // a run ends this far from the origin
    float carTraction = 0.0f;                         // > 0: CAutomobile::m_fCarTraction of the AI car (normally 1)
};

static std::vector<float> ParseFloats(const std::string& s) {
    std::vector<float> out;
    std::string cur;
    for (char c : s + ",") {
        if (c == ',' || c == ';' || c == ' ' || c == '\t') {
            if (!cur.empty()) { out.push_back(static_cast<float>(atof(cur.c_str()))); cur.clear(); }
        }
        else cur += c;
    }
    return out;
}
static std::vector<std::string> ParseWords(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s + ",") {
        if (c == ',' || c == ' ' || c == '\t') { if (!cur.empty()) { out.push_back(cur); cur.clear(); } }
        else cur += c;
    }
    return out;
}

// Up to 60 cars on the same course at once, each LaneShift m to the right of the one before, LanesPerDeck to a deck of
// the platform, the next ones on the deck DeckHeight m above (A/B tests; the GA's population)
static constexpr int kMaxRunners = 500;
// the cars' names: A..Z, a..z, 0..9, then AA, AB...
static char kTagBuf[kMaxRunners][4];
static const char* kTags[kMaxRunners];
static const bool kTagsMade = [] {
    for (int k = 0; k < kMaxRunners; k++) {
        if (k < 26) snprintf(kTagBuf[k], 4, "%c", 'A' + k);
        else if (k < 52) snprintf(kTagBuf[k], 4, "%c", 'a' + k - 26);
        else if (k < 62) snprintf(kTagBuf[k], 4, "%c", '0' + k - 52);
        else snprintf(kTagBuf[k], 4, "%c%c", 'A' + (k - 62) / 26, 'A' + (k - 62) % 26);
        kTags[k] = kTagBuf[k];
    }
    return true;
}();
static int lanesPerDeck = 10;
// Driver = human: the user drives car A with the game's own controls (the player's physics), scored like the others
// and logged as HUMANO, but never chosen by the GA; a 3 s countdown holds everyone at the start
static bool human = false;
static float countdownS = 0.0f;
// mode neural (after LucasHJin/self-play-racing's policy): 11 rays to the road's edges over the front 180 degrees,
// forward/sideways speed, yaw rate, the wheel, the road's curvature 20/50/100 m ahead -> 10 tanh -> wheel and pedal;
// the GA evolves its weights (tools/nn_clone.py can start them from a human's lap)
static constexpr int kNnIn = 18, kNnHid = 10, kNN = (kNnIn + 1) * kNnHid + (kNnHid + 1) * 2;
// mode neural2: every sensor there is (63: rays, the road and its own line, curvature ahead, obstacles, the car's
// motion, slip, attitude, each wheel, the teacher's wheel and the speed error) -> 24 tanh -> two residuals on its
// teacher (pursuit with its genes, and the speed planner): wheel = teacher + kNn2SteerSpan x lock x o[0], target speed
// x (1 + kNn2SpeedSpan x o[1]); the pedals, the ABS and the TC as for everyone. With zero output weights it IS its
// teacher (home from its first run); the GA's ES then looks for what beats pursuit. Taught first by imitation
// (tools/nn2_train.py: the residual the better finishers drove over pursuit), then by evolution strategies in the game.
// (Before, its outputs were the whole wheel and pedal, taught by behaviour cloning, and it never got home: its inputs
// n49..n51 were its own wheel and pedals, and the trace, written after Control, logged this frame's command there, so
// the teachers' labels were these very inputs (correlation 1.000); the net learned to copy them, and in the loop it
// twitched the wheel 10x more than pursuit on the straight before the 220 m bends and spun in them)
// (two hidden layers, 48 and 24: the user wants it more complex and faster; one of 24 before)
// outputs: wheel, pedal, line, TC, ABS, handbrake, lookahead, aggressiveness (x the speed profile)
static constexpr int kNn2In = 80, kNn2Hid = 48, kNn2Hid2 = 24, kNn2Out = 8;
static constexpr int kNn2L2 = (kNn2In + 1) * kNn2Hid, kNn2L3 = kNn2L2 + (kNn2Hid + 1) * kNn2Hid2;
static constexpr int kNN2 = kNn2L3 + (kNn2Hid2 + 1) * kNn2Out;
static constexpr float kNn2SteerSpan = 0.25f, kNn2SpeedSpan = 0.5f; // the most it can add: a quarter of the lock, 30% of the speed
static float edgeComfortGA = 1.0f; // a corridor: closer than this to an edge costs (stats.deficit; [GA] EdgeComfort)
static float deckHeight = 80.0f;
static void SetRunnerSettings(const Settings* rs);
static int numRunners = 1;
static float laneShiftB = 40.0f;
static void OpenTrace();
namespace GA {
static bool on = false;
static bool traceOn = true; // ([GA] Trace)
static void Read(CIniReader& ini, const Settings& s);
static void Start();
static void Score(int k, const std::string& why, float time, double mean, double rms, double maxE, double out, float sMax, float length,
    double deficit, double minMargin);
static void NextGeneration();
static void SetSectors(int k, const std::vector<float>& sec);
}

static void ReadSettings() {
    CIniReader ini(iniPath);
    Settings s;
    auto f = [&](const char* sec, const char* key, float& v) { v = ini.ReadFloat(sec, key, v); };
    auto b = [&](const char* sec, const char* key, bool& v) { v = ini.ReadInteger(sec, key, v ? 1 : 0) != 0; };
    auto str = [&](const char* sec, const char* key, std::string& v) { v = ini.ReadString(sec, key, v); };
    auto o = ParseFloats(ini.ReadString("Proving", "Origin", ""));
    if (o.size() >= 3) s.origin = CVector(o[0], o[1], o[2]);
    f("Proving", "Heading", s.headingDeg);
    auto cars = ParseFloats(ini.ReadString("Proving", "Cars", "541"));
    s.cars.clear();
    for (float c : cars) s.cars.push_back(static_cast<int>(c));
    s.tests = ParseWords(ini.ReadString("Proving", "Tests", "circle"));
    str("Proving", "Driver", s.driver);
    human = s.driver == "human";
    countdownS = human ? 3.0f : 0.0f;
    b("Proving", "Grid", s.grid);
    f("Proving", "GridSize", s.gridSize);
    f("Proving", "GridExtent", s.gridExtent);
    b("Proving", "ExitWhenDone", s.exitWhenDone);
    b("Proving", "Pause", s.pause);
    f("Proving", "CarTraction", s.carTraction);

    auto control = [&](Settings& s, const std::string& suffix) {
    const std::string st = "Steer" + suffix, sp = "Speed" + suffix;
    auto f = [&](const std::string& sec, const char* key, float& v) { v = ini.ReadFloat(sec, key, v); };
    auto b = [&](const std::string& sec, const char* key, bool& v) { v = ini.ReadInteger(sec, key, v ? 1 : 0) != 0; };
    auto str = [&](const std::string& sec, const char* key, std::string& v) { v = ini.ReadString(sec, key, v); };
    str(st, "Mode", s.mode);
    f(st, "LookaheadTime", s.lookTime);
    f(st, "LookaheadMin", s.lookMin);
    f(st, "LookaheadMax", s.lookMax);
    f(st, "Gain", s.gain);
    f(st, "Bandwidth", s.bandwidth);
    f(st, "Damping", s.damping);
    f(st, "Preview", s.preview);
    f(st, "StanleyK", s.stanleyK);
    f(st, "StanleySoft", s.stanleySoft);
    b(st, "FeedForward", s.feedForward);
    f(st, "MaxSteer", s.maxSteer);
    f(st, "SteerRate", s.steerRate);
    f(st, "Smoothing", s.smoothing);
    f(st, "GripClampG", s.gripClampG);
    b(st, "Racer", s.racer);

    f(sp, "Kp", s.speedKp);
    f(sp, "BrakeKp", s.brakeKp);
    f(sp, "CruiseGas", s.cruiseGas);
    f(sp, "GripG", s.speedGripG);
    f(sp, "BrakeG", s.speedBrakeG);
    f(sp, "GripFactor", s.gripFactor);
    f(sp, "GripTight", s.gripTight);
    f(sp, "GripFast", s.gripFast);
    f(sp, "LearnRate", s.learnRate);
    f(st, "LineBias", s.lineBias);
    f(st, "LineCut", s.lineCut);
    f(st, "LineRace", s.lineRace);
    lineRate = ini.ReadFloat(st, "LineRate", lineRate);
    lineTime = ini.ReadInteger(st, "LineTime", lineTime);
    f(st, "MpcHorizon", s.mpcHorizon);
    f(st, "MpcWeight", s.mpcWeight);
    f(sp, "GtAccel", s.gtAccel);
    f(st, "CounterSteer", s.counterSteer);
    f(st, "RppCurv", s.rppCurv);
    f(st, "BlendW", s.blendW);
    f(st, "GapBias", s.gapBias);
    f(sp, "AbsMargin", s.absMargin);
    f(sp, "AbsDrop", s.absDrop);
    f(sp, "AbsRecover", s.absRecover);
    f(sp, "AbsCircle", s.absCircle);
    f(sp, "TcSlip", s.tcSlip);
    f(sp, "TcMinGas", s.tcMinGas);
    f(sp, "TcDrop", s.tcDrop);
    f(sp, "TcRecover", s.tcRecover);
    f(sp, "TcCircle", s.tcCircle);
    b(sp, "TractionControl", s.tractionControl);
    b(sp, "ABS", s.abs);
    f(sp, "AbortOffset", s.abortOffset);
    };
    control(s, "");
    f("Proving", "AreaRadius", s.areaRadius);

    for (const std::string& t : s.tests) {
        const std::string sec = "Test." + t;
        TestDef d;
        d.name = t;
        d.shape = ini.ReadString(sec, "Shape", t);
        d.length = ini.ReadFloat(sec, "Length", d.length);
        d.radius = ini.ReadFloat(sec, "Radius", d.radius);
        d.spacing = ini.ReadFloat(sec, "Spacing", d.spacing);
        d.amplitude = ini.ReadFloat(sec, "Amplitude", d.amplitude);
        d.runIn = ini.ReadFloat(sec, "RunIn", d.runIn);
        d.out = ini.ReadFloat(sec, "Out", d.out);
        d.hold = ini.ReadFloat(sec, "Hold", d.hold);
        d.back = ini.ReadFloat(sec, "Back", d.back);
        d.dir = ini.ReadInteger(sec, "Dir", d.dir) < 0 ? -1 : 1;
        d.duration = ini.ReadFloat(sec, "Duration", d.duration);
        auto sp = ParseFloats(ini.ReadString(sec, "Speed", "60"));
        if (!sp.empty()) d.speeds = sp;
        d.startSpeed = ini.ReadFloat(sec, "StartSpeed", d.startSpeed);
        d.gripG = ini.ReadFloat(sec, "GripG", d.gripG);
        d.brakeG = ini.ReadFloat(sec, "BrakeG", d.brakeG);
        d.brakeTo = ini.ReadFloat(sec, "BrakeTo", d.brakeTo);
        d.width = ini.ReadFloat(sec, "Width", d.width);
        d.segments = ini.ReadString(sec, "Segments", "");
        // a long track over several lines: Segments2, Segments3...
        for (int k = 2; k <= 40; k++) {
            const std::string more = ini.ReadString(sec, Format("Segments%d", k), "");
            if (!more.empty()) d.segments += "/" + more;
        }
        auto pts = ParseFloats(ini.ReadString(sec, "Points", ""));
        for (size_t i = 0; i + 1 < pts.size(); i += 2) d.points.emplace_back(pts[i], pts[i + 1]);
        d.closed = ini.ReadInteger(sec, "Closed", 0) != 0;
        d.steer = ini.ReadString(sec, "Steer", d.steer);
        d.steerAmp = ini.ReadFloat(sec, "SteerAmp", d.steerAmp);
        d.steerStart = ini.ReadFloat(sec, "SteerStart", d.steerStart);
        d.steerRate = ini.ReadFloat(sec, "SteerRate", d.steerRate);
        d.freq = ini.ReadFloat(sec, "Freq", d.freq);
        d.freqEnd = ini.ReadFloat(sec, "FreqEnd", d.freqEnd);
        d.pedal = ini.ReadString(sec, "Pedal", d.pedal);
        d.pedalStart = ini.ReadFloat(sec, "PedalStart", d.pedalStart);
        d.brakeLevel = ini.ReadFloat(sec, "BrakeLevel", d.brakeLevel);
        d.handbrakeAt = ini.ReadInteger(sec, "Handbrake", 0) != 0;
        auto st = ParseFloats(ini.ReadString(sec, "Start", ""));
        if (st.size() >= 3) { d.hasStart = true; d.start = CVector2D(st[0], st[1]); d.startHeading = st[2]; }
        s.defs[t] = d;
    }
    // A/B (Runners = 2) and more: cars B, C... on the same course, each LaneShift m to the right of the one before, with
    // [Steer.B] / [Speed.B] (C, D...) over A's settings
    numRunners = std::clamp(ini.ReadInteger("Proving", "Runners", ini.ReadInteger("Proving", "AB", 0) ? 2 : 1), 1, kMaxRunners);
    laneShiftB = ini.ReadFloat("Proving", "LaneShift", laneShiftB);
    lanesPerDeck = std::max(ini.ReadInteger("Proving", "LanesPerDeck", lanesPerDeck), 1);
    deckHeight = ini.ReadFloat("Proving", "DeckHeight", deckHeight);
    std::vector<Settings> rs(kMaxRunners, s);
    for (int k = 1; k < kMaxRunners; k++) control(rs[k], std::string(".") + kTags[k]);
    SetRunnerSettings(rs.data());
    GA::Read(ini, s);
}

// --- Courses ---------------------------------------------------------------------------------------------------------

struct Course {
    std::vector<CVector2D> p;      // world, every ~0.5 m
    std::vector<float> s, head, curv; // m along, direction (rad, world), curvature (1/m, > 0 left)
    std::vector<CVector2D> cones;  // world
    std::vector<float> vcap, vsud; // m/s at each point: a posted limit (planned for), a sudden one (seen only there)
    std::vector<float> wind;       // m/s2 of crosswind at each point (> 0: pushing to the course's left)
    std::vector<char> centre;      // 1: the car's own line in the middle of the road here (the lane changes, the wind)
    std::vector<std::pair<float, std::string>> sections; // track: where each named part starts (m)
    struct Obstacle { int i, model; float lat; };
    std::vector<Obstacle> obst;    // track piece O: a bump or a kicker at point i, lat x the half width to the left
    bool closed = false;
    float length = 0.0f;
    float halfWidth = 0.0f;        // the corridor: its edges this far each side of the points (0: none)
};
// The grip for a curve of curvature c (1/m): x GripTight at a radius <= 15 m, x 1 at 45 m, x GripFast >= 150 m
static float CurveGrip(float c);
// The kind of curve (what a car learns): radius < 12 m, < 25, < 60, < 150, more
static int CurveBand(float c) {
    const float r = 1.0f / std::max(c, 1e-4f);
    return r < 12.0f ? 0 : r < 25.0f ? 1 : r < 60.0f ? 2 : r < 150.0f ? 3 : 4;
}
static const char* const kBandNames[5] = { "<12", "<25", "<60", "<150", ">150" };
static std::string SectionAt(const Course& c, float s) {
    std::string name;
    for (const auto& p : c.sections) { if (p.first > s + 0.1f) break; name = p.second; }
    return name;
}

struct Stats {
    double n = 0, absE = 0, sqE = 0, maxE = 0, speedErr = 0, maxLatG = 0, steerAbs = 0, reversals = 0;
    double skidFront = 0, skidRear = 0, spin = 0, time = 0;
    double outTime = 0;             // s further than the GA's OffLimit from the course (a corridor: the body past an edge)
    double deficit = 0;             // a corridor: m x s the body was closer to an edge than the GA's EdgeComfort
    double minMargin = 1e9;         // ... the least room it had (m; < 0: past the edge)
    double gainSum = 0, gainN = 0;  // yaw rate / (v tan(steer)): 1/L of the car's turn while the tyres hold
    float lastSteerDelta = 0.0f, lastSign = 0.0f;
    float maxSpeed = 0.0f;
    float t100 = 0.0f;              // full-throttle tests: time to 100 km/h
    float stopDist = 0.0f;          // brake tests
};

// A change a car is trying while it drives (LearnByResult): which number, its value before, and how it went
struct Trial { int band = -1, which = -1; float old = 0, sum = 0, off = 0, maxOff = 0, t = 0; int n = 0; };

// One car on the course (A, and B for A/B tests): its settings, course, car and controller state
struct Runner {
    const char* tag = "A";
    int index = 0;
    float laneShift = 0.0f;         // m to the right of the test's course
    float baseZ = 300.0f;           // its deck's surface
    Settings cfg;
    Course course;
    CVehicle* car = nullptr;
    int carHandle = -1, driverHandle = -1;
    int nearIdx = 0;
    float steerCmd = 0.0f, steerOut = 0.0f, gasOut = 0.0f, brakeOut = 0.0f;
    bool handbrakeOut = false, brakeDone = false, finished = false;
    float targetKmh = 0.0f, offsetNow = 0.0f, headErrNow = 0.0f, curvNow = 0.0f;
    CVector prevVel;
    bool havePrevVel = false;
    float wheelBase = 2.7f, rearAxleY = -1.35f;    // m; the rear axle from the car's origin (along its forward)
    float aiGripG = 1.0f, aiAdhesive = 1.0f;       // the AI's sideways grip limit (g) on tarmac, from the game's formula
    float absScale = 1.0f, tcScale = 1.0f;
    float playerSteerFactor = 1.0f;                // what the game keeps of a player's wheel at this speed (logged)
    std::deque<CVector> trail;
    CVector2D lookPoint;
    std::string abortWhy;
    Stats stats;
    std::ofstream csv;
    Course road;                    // the road itself (course is then the car's own line on it: OwnLine)
    std::string roadKey;            // ... the test and lane it was built for (a new brain on the lane: only its own line again)
    std::vector<float> profile;     // mode gt4: the speed at every point of its line, worked out before the run (m/s)
    // the run so far (HUD, GA)
    std::string result;             // how its run ended ("" while it runs)
    float endTime = 0.0f, sMax = 0.0f, slowTime = 0.0f;
    float progressS = 0.0f, progressT = 0.0f; // the last 5 m of progress: where and when
    float latGNow = 0.0f, slipNow = 0.0f, speedNow = 0.0f;
    bool skidFNow = false, skidRNow = false, absOn = false, tcOn = false;
    float windNow = 0.0f;
    std::vector<float> secT;        // the time it reached the start of each part of the course (-1: not yet)
    float bodyFront = 2.2f, bodyRear = -2.2f, bodyHalfW = 1.0f; // the body's corners (m, car frame)
    float marginNow = 99.0f;        // a corridor: the room between its nearest corner and the nearest edge (m)
    float nnO0 = 0.0f, nnO1 = 0.0f; // neural2: its outputs, held a frame (Neural2Steer)
    float nnLine = 0.0f, lineRoom = 0.0f; // neural2: its line output (-1..1 of lineRoom, m, across from its own line)
    float nnTc = 0.0f, nnAbs = 0.0f, nnHb = 0.0f; // neural2: its TC, ABS (x e^(0.7 / 0.5 out)) and handbrake (> 0.5) outputs
    float nnLook = 0.0f;                          // ... its lookahead (x e^(0.5 out))
    bool profileTried = false;                    // ... its speed profile worked out this run (Control)
    float nnAct[48 + 24 + 8] = {};
    float nnAgg = 0.0f;                           // ... its aggressiveness: the speed profile x e^(0.25 out)               // ... the net's last activations: its layers and outputs (the HUD)
    bool nnHeld = false;
    float nnPedal = 0.0f;           // mode neural: its pedal (> 0 gas, < 0 brake); neural2: its speed residual, o[1]
    float teacherSteer = 0.0f, teacherPedal = 0.0f; // mode neural2: what pursuit would do here (DAgger's labels)
    float nnIn[kNn2In] = {};        // neural2's inputs as the net saw them this frame, before the command (Trace logs these:
                                    // taken again after Control, n49..n51 held the command itself, the label: see kNn2In)
    std::vector<int> obstacles;     // its lane's obstacle objects (handles)
    float runT = 0.0f;              // s since its run started (steady state: every lane on its own clock)
    float doneAt = 0.0f;            // the clock when its run ended
    int runId = 0;                  // its run's number (steady state: the name of the run in the logs, tag.runId)
    int bornGen = 0;                // the generation its brain was made in (HUD: cars of several generations together)
    float learned[5] = { 1, 1, 1, 1, 1 }; // its own grip multiplier for each kind of curve (CurveBand), learned on the way
    float tcBand[5] = { 1, 1, 1, 1, 1 };  // ... the gas its TC lets through in each kind of curve
    float absLearn = 1.0f;                // ... and the brake pedal its ABS allows
    float bandBase[5] = { NAN, NAN, NAN, NAN, NAN }, absBase = NAN; // its average result in each kind / braking
    Trial curveTrial, absTrial;
    int trials = 0, kept = 0;
    float aLongNow = 0.0f;
};
static Runner runners[kMaxRunners];
static Runner* R = &runners[0];
static void SetRunnerSettings(const Settings* rs) {
    for (int k = 0; k < kMaxRunners; k++) {
        runners[k].cfg = rs[k];
        runners[k].tag = kTags[k];
        runners[k].index = k;
        runners[k].laneShift = (k % lanesPerDeck) * laneShiftB;
        runners[k].baseZ = rs[k].origin.z + (k / lanesPerDeck) * deckHeight;
    }
}
static Settings& CfgOf(int k) { return runners[k].cfg; }
static CVehicle* CarOf(int k) { return runners[k].car; }
static int HandleOf(int k) { return runners[k].carHandle; }
// the current runner's state under the names the code had for one car
#define cfg (R->cfg)
#define course (R->course)
#define car (R->car)
#define carHandle (R->carHandle)
#define driverHandle (R->driverHandle)
#define nearIdx (R->nearIdx)
#define steerCmd (R->steerCmd)
#define steerOut (R->steerOut)
#define gasOut (R->gasOut)
#define brakeOut (R->brakeOut)
#define handbrakeOut (R->handbrakeOut)
#define brakeDone (R->brakeDone)
#define targetKmh (R->targetKmh)
#define offsetNow (R->offsetNow)
#define headErrNow (R->headErrNow)
#define curvNow (R->curvNow)
#define prevVel (R->prevVel)
#define havePrevVel (R->havePrevVel)
#define wheelBase (R->wheelBase)
#define rearAxleY (R->rearAxleY)
#define aiGripG (R->aiGripG)
#define aiAdhesive (R->aiAdhesive)
#define absScale (R->absScale)
#define tcScale (R->tcScale)
#define playerSteerFactor (R->playerSteerFactor)
#define trail (R->trail)
#define lookPoint (R->lookPoint)
#define abortWhy (R->abortWhy)
#define stats (R->stats)
#define csv (R->csv)
#define forRunners for (Runner* rr = runners; rr < runners + numRunners && ((R = rr), true); rr++)
#define runT (R->runT)

// The course frame of the test being built (its Start, else the ini's Origin/Heading)
static CVector2D frameOrigin;
static float frameHeading = 0.0f;
static CVector2D ToWorld(const CVector2D& local) {
    const float a = frameHeading * 3.14159265f / 180.0f;
    const CVector2D d(std::cos(a), std::sin(a)), l(-std::sin(a), std::cos(a));
    return frameOrigin + d * local.x + l * local.y;
}

static float WrapPi(float a) {
    while (a > 3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

// Distance along, direction and curvature of a course from its points
static void Geometry(Course& c) {
    const int n = static_cast<int>(c.p.size());
    c.s.resize(n);
    c.head.resize(n);
    c.curv.resize(n);
    for (int i = 0; i < n; i++) c.s[i] = i ? c.s[i - 1] + (c.p[i] - c.p[i - 1]).Magnitude() : 0.0f;
    c.length = c.s[n - 1] + (c.closed ? (c.p[0] - c.p[n - 1]).Magnitude() : 0.0f);
    auto idx = [&](int i) { return c.closed ? ((i % n) + n) % n : std::clamp(i, 0, n - 1); };
    for (int i = 0; i < n; i++) {
        const CVector2D t = c.p[idx(i + 2)] - c.p[idx(i - 2)];
        c.head[i] = std::atan2(t.y, t.x);
    }
    for (int i = 0; i < n; i++) {
        // over +-3 m (6 points each way)
        const int a = idx(i - 6), b = idx(i + 6);
        const float ds = (c.p[b] - c.p[i]).Magnitude() + (c.p[i] - c.p[a]).Magnitude();
        c.curv[i] = ds > 0.5f ? WrapPi(c.head[b] - c.head[a]) / ds : 0.0f;
    }
}

// The racing line of a road: the offset across it (m, > 0 left) at each point, within +-room. An elastic band pulled
// straight inside the road: each point moved toward the middle of its neighbours ahead and behind, from far apart
// (64 points, 32 m) down to close (4 points), clamped to the room; what is left is the shortest, least curved line,
// wide into a curve, on the inside at the apex, wide out of it (the user: "formato de corrida, fazendo curvas por
// dentro"; the centre line was the car's own line before). The same for every lane (only moved over): worked out once
// (pinned to 0 where the road says centre: the moose and the crosswind, the user: "o carro tem que realmente passar no
// meio"; the band bends into and out of them by itself)
static void Log(const std::string& s);
static std::vector<float> raceOff;
static std::map<std::string, std::vector<float>> raceCache; // (by the road and the room: worked out once each)
static std::vector<float> raceRoom;                         // the room each point may use (0: a test in the middle)
static std::map<std::string, std::vector<float>> raceRoomCache;
static void RaceLine(const Course& rd, float room) {
    const int n = static_cast<int>(rd.p.size());
    const std::string key = Format("%d|%.2f|%.2f|%.2f|%.2f|%.2f|%d", n, room, rd.length, rd.head.empty() ? 0.0f : rd.head[n / 2], rd.curv.empty() ? 0.0f : rd.curv[n / 3], lineRate, lineTime);
    auto hit = raceCache.find(key);
    if (hit != raceCache.end()) { raceOff = hit->second; raceRoom = raceRoomCache[key]; return; }
    raceOff.assign(n, 0.0f);
    if (n < 50 || room <= 0.0f) { raceRoom.assign(n, std::max(room, 0.0f)); raceCache[key] = raceOff; raceRoomCache[key] = raceRoom; return; }
    const bool pins = static_cast<int>(rd.centre.size()) == n;
    std::vector<CVector2D> nrm(n);
    for (int i = 0; i < n; i++) nrm[i] = CVector2D(-std::sin(rd.head[i]), std::cos(rd.head[i]));
    // the room each point may use: none in a test kept in the middle, opening over 80 m (160 points, a smoothstep) out
    // of it (a hard pin made a kink where a part began: the user, "entre a reta e o vento"; 150 m closed it too early)
    std::vector<float> roomAt(n, room);
    if (pins) {
        std::vector<int> d(n, 1 << 20);
        for (int i = 0, last = -(1 << 20); i < n; i++) { if (rd.centre[i]) last = i; d[i] = i - last; }
        for (int i = n - 1, last = 1 << 21; i >= 0; i--) { if (rd.centre[i]) last = i; d[i] = std::min(d[i], last - i); }
        for (int i = 0; i < n; i++) { const float x = std::clamp(d[i] / 160.0f, 0.0f, 1.0f); roomAt[i] = room * x * x * (3.0f - 2.0f * x); }
    }
    raceRoom = roomAt;
    raceRoomCache[key] = roomAt;
    std::vector<float> next(n);
    for (int span : { 64, 32, 16, 8, 4 })
        for (int it = 0; it < 60; it++) {
            next = raceOff;
            for (int i = span; i < n - span; i++) {
                const CVector2D a = rd.p[i - span] + nrm[i - span] * raceOff[i - span], b = rd.p[i + span] + nrm[i + span] * raceOff[i + span];
                const CVector2D here = rd.p[i] + nrm[i] * raceOff[i];
                const float want = raceOff[i] + CVector2D::Dot((a + b) * 0.5f - here, nrm[i]);
                next[i] = std::clamp(raceOff[i] + 0.6f * (want - raceOff[i]), -roomAt[i], roomAt[i]);
            }
            raceOff.swap(next);
        }
    // The minimum-curvature line (TUM's global_racetrajectory_optimization, opt_min_curv: the least sum of squared
    // curvature with the car inside the road, a QP; here by projected gradient with Nesterov's momentum, from the band
    // above): every 2 m (4 points), the curvature ~ the second difference of the path, each point within its room
    {
        const int st = 4, m = (n - 1) / st + 1;
        std::vector<CVector2D> c(m), nn(m);
        std::vector<float> x(m), y(m), prev(m), lim(m), g(m);
        for (int j = 0; j < m; j++) {
            const int i = std::min(j * st, n - 1);
            c[j] = rd.p[i]; nn[j] = nrm[i]; x[j] = raceOff[i]; lim[j] = roomAt[i];
        }
        x[0] = x[m - 1] = 0.0f;
        prev = x;
        // (+ kRate x the change of curvature between neighbours, squared: the line eases into and out of every curve
        // and every part, as a clothoid; with the curvature alone it bent at once where a straight met a curve or a
        // part kept in the middle began, the joins the user saw)
        const float kRate = std::max(lineRate, 0.0f);
        std::vector<CVector2D> P(m), D(m), Q(m);
        std::vector<float> wt(m, 1.0f); // (each point's weight: 1, minimum curvature; the minimum time rounds below)
        auto solve = [&](int iters) {
        float wMax = 1.0f;
        for (float w : wt) wMax = std::max(wMax, w);
        const float stepLen = 1.0f / (16.0f * wMax + 64.0f * kRate);
        prev = x;
        for (int it = 0; it < iters; it++) {
            const float mom = static_cast<float>(it % 3000) / (it % 3000 + 3); // (restarted now and then: steadier)
            for (int j = 0; j < m; j++) y[j] = x[j] + mom * (x[j] - prev[j]);
            for (int j = 0; j < m; j++) P[j] = c[j] + nn[j] * y[j];
            for (int j = 1; j + 1 < m; j++) D[j] = P[j - 1] - P[j] * 2.0f + P[j + 1];
            D[0] = CVector2D(0.0f, 0.0f);
            D[m - 1] = CVector2D(0.0f, 0.0f);
            // the gradient of sum |D|^2 + kRate sum |D(j+1) - D(j)|^2 with respect to each D(j)
            for (int j = 0; j < m; j++) {
                const CVector2D ePrev = j > 0 ? D[j] - D[j - 1] : CVector2D(0.0f, 0.0f), eNext = j + 1 < m ? D[j + 1] - D[j] : CVector2D(0.0f, 0.0f);
                Q[j] = D[j] * wt[j] + (ePrev - eNext) * kRate;
            }
            Q[0] = CVector2D(0.0f, 0.0f);
            Q[m - 1] = CVector2D(0.0f, 0.0f);
            prev = x;
            for (int j = 1; j + 1 < m; j++) {
                const CVector2D gd = Q[j - 1] - Q[j] * 2.0f + Q[j + 1];
                x[j] = std::clamp(y[j] - stepLen * CVector2D::Dot(gd, nn[j]), -lim[j], lim[j]);
            }
        }
        };
        // the lap time of a line, roughly (its curvature, 1.9 g of grip and of braking, 0.8 of it to speed up, 130 m/s
        // at most: TUM's velocity profile, simplified) and each point's speed
        std::vector<float> vel(m);
        auto lapOf = [&]() {
            for (int j = 0; j < m; j++) P[j] = c[j] + nn[j] * x[j];
            const float G = 1.9f * 9.81f;
            for (int j = 0; j < m; j++) {
                const float k = j > 0 && j + 1 < m ? (P[j - 1] - P[j] * 2.0f + P[j + 1]).Magnitude() / 4.0f : 0.0f;
                vel[j] = k > 1e-5f ? std::min(130.0f, std::sqrt(G / k)) : 130.0f;
            }
            for (int j = m - 2; j >= 0; j--) vel[j] = std::min(vel[j], std::sqrt(vel[j + 1] * vel[j + 1] + 2.0f * G * 2.0f));
            for (int j = 1; j < m; j++) vel[j] = std::min(vel[j], std::sqrt(vel[j - 1] * vel[j - 1] + 2.0f * 0.8f * G * 2.0f));
            double t = 0.0;
            for (int j = 1; j < m; j++) t += (P[j] - P[j - 1]).Magnitude() / std::max(0.5f * (vel[j] + vel[j - 1]), 1.0f);
            return t;
        };
        solve(12000);
        const double tCurv = lapOf();
        // Minimum time (TUM's opt_mintime, approximated): the curvature weighted by how slow the car is there, (40 m/s
        // / its speed)^2 between 0.3 and 3, solved again from the line before; the slow curves count more, the fast
        // parts give up a little straightness for them ([Steer] LineTime: the rounds, 0: minimum curvature only)
        double tTime = tCurv;
        for (int round = 0; round < lineTime; round++) {
            for (int j = 0; j < m; j++) wt[j] = std::clamp((40.0f / std::max(vel[j], 5.0f)) * (40.0f / std::max(vel[j], 5.0f)), 0.3f, 3.0f);
            const std::vector<float> keep = x;
            solve(8000);
            const double t2 = lapOf();
            if (t2 > tTime) { x = keep; lapOf(); break; } // (no better: the line before)
            tTime = t2;
        }
        Log(Format("racing line: minimum curvature ~%.1f s a lap, minimum time ~%.1f s (%d rounds)", tCurv, tTime, lineTime));
        for (int i = 0; i < n; i++) {
            const int j = std::min(i / st, m - 2);
            const float t = std::clamp((i - j * st) / static_cast<float>(st), 0.0f, 1.0f);
            raceOff[i] = x[j] + (x[j + 1] - x[j]) * t;
        }
    }
    // (the ends where they are: the start box and the finish line in the middle)
    for (int i = 0; i < n; i++) raceOff[i] *= std::clamp(std::min(i, n - 1 - i) / 64.0f, 0.0f, 1.0f);
    raceCache[key] = raceOff;
    // (the summary: how far off the middle it goes in each part, and how much of each part is kept in the middle)
    for (size_t k = 0; k < rd.sections.size(); k++) {
        const float s0 = rd.sections[k].first, s1 = k + 1 < rd.sections.size() ? rd.sections[k + 1].first : rd.length;
        float most = 0.0f;
        int in = 0, pinned = 0;
        float slope = 0.0f; // (the steepest the line crosses the road: cm a metre)
        float jerk = 0.0f;  // (the sharpest change of the line's curvature: 1/m over 10 m, x 1000)
        for (int i = 0; i < n; i++) if (rd.s[i] >= s0 && rd.s[i] < s1) {
            most = std::max(most, std::abs(raceOff[i])); in++; pinned += pins && rd.centre[i];
            if (i > 0 && rd.s[i] > rd.s[i - 1]) slope = std::max(slope, 100.0f * std::abs(raceOff[i] - raceOff[i - 1]) / (rd.s[i] - rd.s[i - 1]));
            if (i >= 20 && i + 20 < n) {
                // the line's curvature 10 m back and on (its points: the road's moved across)
                auto at = [&](int q) { return rd.p[q] + nrm[q] * raceOff[q]; };
                auto kap = [&](int q) { const CVector2D a0 = at(q - 4), a1 = at(q), a2 = at(q + 4); const CVector2D dd = a0 - a1 * 2.0f + a2; return (CVector2D::Dot(dd, nrm[q])) / 4.0f; };
                jerk = std::max(jerk, 1000.0f * std::abs(kap(i + 16) - kap(i - 16)));
            }
        }
        Log(Format("racing line (room %.2f m): %-16s up to %.2f m off the middle, %3.0f%% kept in it, steepest %.1f cm/m, curvature change %.1f", room,
            rd.sections[k].second.c_str(), most, in ? 100.0f * pinned / in : 0.0f, slope, jerk));
    }
}

// The car's own line inside the road (its genes LineRace, LineBias, LineCut): the racing line x LineRace, plus the
// room x (bias + cut x the curve's direction ahead and behind), smoothed over +-10 m (no step for the wheel), never
// past the room. The car steers along it and plans its speed on it (a straighter line is a faster one); the road
// (R->road) stays what its body is judged against
static void OwnLine() {
    const int n = static_cast<int>(course.p.size());
    if (course.halfWidth <= 0.0f || n < 50 || (human && R == runners)) return;
    if (cfg.lineBias == 0.0f && cfg.lineCut == 0.0f && cfg.lineRace == 0.0f && cfg.mode != "neural2") return;
    const float body = R->bodyHalfW > 0.1f ? R->bodyHalfW : 1.0f; // (the first build, before the car is made)
    const float room = std::max(course.halfWidth - body - 0.05f, 0.0f);
    RaceLine(R->road, std::max(room - 0.1f, 0.0f)); // (a little inside the room: the smoothing and the wheel's error)
    R->lineRoom = room;
    const bool netLine = cfg.mode == "neural2";
    std::vector<float> o(n), t(n);
    for (int i = 0; i < n; i++) {
        float k = 0.0f;
        int m = 0;
        for (int j = std::max(i - 24, 0); j <= std::min(i + 24, n - 1); j++, m++) k += course.curv[j];
        k /= std::max(m, 1);
        const float race = i < static_cast<int>(raceOff.size()) ? raceOff[i] * (netLine ? 1.0f : cfg.lineRace) : 0.0f;
        o[i] = netLine ? race : std::clamp(race + room * (cfg.lineBias + cfg.lineCut * std::clamp(k * 25.0f, -1.0f, 1.0f)), -room, room);
    }
    for (int pass = 0; pass < 1; pass++) {
        for (int i = 0; i < n; i++) {
            float sum = 0.0f;
            int m = 0;
            for (int j = std::max(i - 10, 0); j <= std::min(i + 10, n - 1); j++, m++) sum += o[j];
            t[i] = sum / std::max(m, 1);
        }
        o.swap(t);
    }
    // the centre stretches: nothing of the genes' own offsets there, faded over 80 m each side (160 points, smoothstep)
    if (static_cast<int>(R->road.centre.size()) == n) {
        std::vector<int> d(n, 1 << 20);
        for (int i = 0, last = -(1 << 20); i < n; i++) { if (R->road.centre[i]) last = i; d[i] = i - last; }
        for (int i = n - 1, last = 1 << 21; i >= 0; i--) { if (R->road.centre[i]) last = i; d[i] = std::min(d[i], last - i); }
        for (int i = 0; i < n; i++) {
            const float x = std::clamp(d[i] / 160.0f, 0.0f, 1.0f), w = 1.0f - x * x * (3.0f - 2.0f * x);
            const float race = i < static_cast<int>(raceOff.size()) ? raceOff[i] * (netLine ? 1.0f : cfg.lineRace) : 0.0f;
            o[i] = race + (o[i] - race) * (1.0f - w);
        }
    }
    for (int i = 0; i < n; i++)
        course.p[i] += CVector2D(-std::sin(course.head[i]), std::cos(course.head[i])) * std::clamp(o[i], -room, room);
    Geometry(course);
}

// Mode gt4 (after Gran Turismo 4's AI: a fixed line and braking points worked out in advance, the same lap every
// time, no reacting): the speed at every point of the car's own line before the run. The most each curve allows
// (grip x the kind of curve), then backwards the braking to reach it and forwards the acceleration out of it, both
// with what the curve leaves of the grip (friction ellipse); the car then only follows it (the sudden brakes still
// stop it: they are unseen)
static void SpeedProfile(const TestDef& d) {
    R->profile.clear();
    const int n = static_cast<int>(course.p.size());
    // (gt4's, and neural2's: the speed its teacher drives is this profile of its own line, as TUM's calc_vel_profile:
    // each point's most from its curvature and grip, then backwards the braking and forwards the acceleration inside
    // the friction ellipse; the planner's look over the curves ahead not used then)
    if ((cfg.mode != "gt4" && cfg.mode != "neural2") || n < 2 || d.steer != "none") return;
    const float G0 = (d.gripG > 0.0f ? d.gripG : d.gripG == 0.0f ? cfg.gripFactor * aiGripG : cfg.speedGripG) * 9.81f;
    if (G0 <= 0.0f) return;
    // (neural2: GtAccel x the grip, the GA finding how much pays: GtAccel's fraction of a g kept it off full throttle
    // out of every curve, "muito medo de acelerar", the user, g140 142.9; all the grip spun 446 of 500, g141)
    // (neural2 brakes on up to the grip itself, the ABS keeping the wheels turning: BrakeG's 1.1 g against ~1.44 g of
    // grip braked early, the user)
    const float brakeA = cfg.mode == "neural2" ? std::max(cfg.speedBrakeG * 9.81f, G0) : (d.brakeG > 0.0f ? d.brakeG : cfg.speedBrakeG) * 9.81f, accelA = cfg.mode == "neural2" ? std::clamp(cfg.gtAccel, 0.3f, 1.0f) * G0 : cfg.gtAccel * 9.81f;
    auto gripAt = [&](int i) { const float c = std::abs(course.curv[i]); return G0 * (c > 1e-4f ? CurveGrip(c) : 1.0f); };
    std::vector<float>& v = R->profile;
    v.resize(n);
    for (int i = 0; i < n; i++) {
        const float c = std::abs(course.curv[i]);
        v[i] = std::min(d.speeds[0] / 3.6f, course.vcap[i]);
        if (c > 1e-4f) v[i] = std::min(v[i], std::sqrt(gripAt(i) / c));
    }
    // (the crosswind takes its share of the grip sideways too: braking for the spiral in the second gust at 450 km/h
    // went past the friction ellipse, 119 of 500 off at 2400-2600 m, g159)
    auto room = [&](int i, float speed, float a) {
        const float G = gripAt(i), lat = speed * speed * std::abs(course.curv[i]) + (course.wind.empty() ? 0.0f : std::abs(course.wind[i]));
        return a * std::sqrt(std::max(1.0f - (lat / G) * (lat / G), 0.0f));
    };
    for (int i = n - 2; i >= 0; i--) {
        const float ds = course.s[i + 1] - course.s[i];
        v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2.0f * room(i + 1, v[i + 1], brakeA) * ds));
    }
    v[0] = std::min(v[0], d.startSpeed >= 0.0f ? d.startSpeed / 3.6f : v[0]);
    for (int i = 0; i + 1 < n; i++) {
        const float ds = course.s[i + 1] - course.s[i];
        v[i + 1] = std::min(v[i + 1], std::sqrt(v[i] * v[i] + 2.0f * room(i, v[i], accelA) * ds));
    }
}

static void BuildCourse(const TestDef& d) {
    course = Course();
    frameOrigin = d.hasStart ? d.start : CVector2D(cfg.origin.x, cfg.origin.y);
    frameHeading = d.hasStart ? d.startHeading : cfg.headingDeg;
    if (R->laneShift != 0.0f) {
        const float a = frameHeading * 3.14159265f / 180.0f;
        frameOrigin += CVector2D(std::sin(a), -std::cos(a)) * R->laneShift;
    }
    // the same road as the lane's last run: only the new brain's own line and speeds (the whole road again took 15-65 ms
    // each new brain, and hundreds of cars spun out together at the start: 13 fps)
    const std::string key = Format("%s|%s|%s|%.2f|%.2f|%.1f", d.name.c_str(), d.shape.c_str(), d.segments.c_str(), d.width, R->laneShift, frameHeading);
    if (key == R->roadKey && !R->road.p.empty()) {
        course = R->road;
        OwnLine();
        SpeedProfile(d);
        return;
    }
    R->roadKey = key;
    std::vector<CVector2D> local;
    struct Zone { int i; float len, value; char kind; };
    std::vector<Zone> zones;
    std::vector<std::pair<int, std::string>> names;
    const float kPi = 3.14159265f;
    auto smoothstep = [](float t) { t = std::clamp(t, 0.0f, 1.0f); return 0.5f - 0.5f * std::cos(t * 3.14159265f); };
    if (d.shape == "circle") {
        const float r = d.radius;
        const int n = std::max(static_cast<int>(2.0f * kPi * r / 0.5f), 32);
        for (int i = 0; i < n; i++) {
            const float a = 2.0f * kPi * i / n;
            local.emplace_back(r * std::sin(a), d.dir * r * (1.0f - std::cos(a)));
        }
        course.closed = true;
    }
    else if (d.shape == "figure8") {
        const float r = d.radius;
        const int n = std::max(static_cast<int>(2.0f * kPi * r / 0.5f), 32);
        for (int i = 0; i < n; i++) { const float a = 2.0f * kPi * i / n; local.emplace_back(r * std::sin(a), r * (1.0f - std::cos(a))); }
        for (int i = 0; i < n; i++) { const float a = 2.0f * kPi * i / n; local.emplace_back(r * std::sin(a), -r * (1.0f - std::cos(a))); }
        course.closed = true;
    }
    else if (d.shape == "slalom") {
        // run-in, then a sine through cones 'spacing' apart (cone k at x = runIn + k spacing, alternately left/right),
        // then a run-out
        // (a whole number of gates: ending mid-wave the course jumped sideways by the amplitude)
        const float len = std::max(std::round(d.length / d.spacing), 1.0f) * d.spacing;
        for (float x = 0.0f; x <= d.runIn + len + 60.0f; x += 0.5f) {
            float y = 0.0f;
            if (x > d.runIn && x < d.runIn + len) y = d.amplitude * std::sin(kPi * (x - d.runIn) / d.spacing);
            local.emplace_back(x, y);
        }
        // the cones on the centre line, where the course passes them furthest out
        for (float x = d.runIn + d.spacing * 0.5f; x < d.runIn + len; x += d.spacing) course.cones.push_back(ToWorld({ x, 0.0f }));
    }
    else if (d.shape == "lanechange") {
        // ISO 3888 style: run-in, Out m over to 'amplitude', Hold m there, Back m back, run-out (moose test: 13.5 / 11 /
        // 12.5)
        const float a = d.amplitude, x0 = d.runIn, x1 = x0 + d.out, x2 = x1 + d.hold, x3 = x2 + d.back;
        for (float x = 0.0f; x <= x3 + 80.0f; x += 0.5f) {
            float y = 0.0f;
            if (x < x0) y = 0.0f;
            else if (x < x1) y = a * smoothstep((x - x0) / d.out);
            else if (x < x2) y = a;
            else if (x < x3) y = a * (1.0f - smoothstep((x - x2) / d.back));
            local.emplace_back(x, y);
        }
        for (float x : { x0 - 12.0f, x0, x1, x2, x3, x3 + 12.0f })
            for (float side : { -1.2f, 1.2f }) {
                const float y = (x >= x1 - 0.1f && x <= x2 + 0.1f) ? a : 0.0f;
                course.cones.push_back(ToWorld({ x, y + side }));
            }
    }
    else if (d.shape == "points" && d.points.size() >= 2) {
        // Catmull-Rom through the points
        const auto& c = d.points;
        const int m = static_cast<int>(c.size());
        auto at = [&](int i) {
            if (d.closed) return c[((i % m) + m) % m];
            return c[std::clamp(i, 0, m - 1)];
        };
        const int segs = d.closed ? m : m - 1;
        for (int i = 0; i < segs; i++) {
            const CVector2D p0 = at(i - 1), p1 = at(i), p2 = at(i + 1), p3 = at(i + 2);
            const int steps = std::max(static_cast<int>((p2 - p1).Magnitude() / 0.5f), 1);
            for (int k = 0; k < steps; k++) {
                const float t = static_cast<float>(k) / steps, t2 = t * t, t3 = t2 * t;
                local.push_back((p1 * 2.0f + (p2 - p0) * t + (p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3) * t2 +
                    (p1 * 3.0f - p0 - p2 * 3.0f + p3) * t3) * 0.5f);
            }
        }
        if (!d.closed) local.push_back(c.back());
        course.closed = d.closed;
    }
    else if (d.shape == "track") {
        // pieces one after the other: "S 300 / L 90 30 / R 45 120": straight m, left/right degrees radius m;
        //   W amplitude spacing gates   slalom: a sine through 'gates' cones 'spacing' m apart
        //   J amplitude out hold back   lane change and back (ISO 3888 moose test: 3.5 13.5 11 12.5)
        //   V km/h m                    a posted speed limit over the next m (the speed plan brakes for it)
        //   B km/h m                    a sudden brake: down to km/h over the next m, unseen until the car is there
        //   X m/s2 m                    crosswind over the next m: a sideways push (> 0 to the left) the steering must
        //                               correct, like a gust on a motorway
        //   O bump|kicker lateral       an obstacle: a one-side bump (a wheel goes over it) or a kicker across the
        //                               road (the car hops), lateral x the half width to the left
        //   N name                      the name of the part that starts here (HUD, logs)
        CVector2D at(0.0f, 0.0f);
        float h = 0.0f;
        local.push_back(at);
        // a piece along the current heading with a sideways offset y(x), x from 0 to len
        auto sideways = [&](float len, const std::function<float(float)>& y) {
            const CVector2D f(std::cos(h), std::sin(h)), l(-std::sin(h), std::cos(h));
            const CVector2D a0 = at;
            for (float x = 0.5f; x <= len + 1e-3f; x += 0.5f) local.push_back(a0 + f * x + l * y(x));
            at = local.back();
        };
        auto inFrame = [&](float x, float y) {
            return at + CVector2D(std::cos(h), std::sin(h)) * x + CVector2D(-std::sin(h), std::cos(h)) * y;
        };
        std::string seg;
        // (separated by '/': the ini reader cuts a value at ';')
        for (char c : d.segments + "/") {
            if (c != '/' && c != ';') { seg += c; continue; }
            char kind = 0;
            size_t k0 = 0;
            while (k0 < seg.size() && (seg[k0] == ' ' || seg[k0] == '\t')) k0++;
            if (k0 < seg.size()) kind = static_cast<char>(toupper(seg[k0]));
            const std::string rest = k0 + 1 < seg.size() ? seg.substr(k0 + 1) : "";
            auto nums = ParseFloats(rest);
            seg.clear();
            if (kind == 'N') {
                const size_t a = rest.find_first_not_of(" \t"), b = rest.find_last_not_of(" \t");
                if (a != std::string::npos) names.emplace_back(static_cast<int>(local.size()) - 1, rest.substr(a, b - a + 1));
            }
            else if (kind == 'O') {
                // O bump|kicker lateral: an obstacle here (tools/make_gridmap.py's models), across the road at lateral
                // x the half width (> 0 left)
                // O kind lateral [count spacing]: count of them every spacing m from here on (the points to come:
                // the next pieces must be at least that long). Kinds: bump (one side, a wheel over it), kicker (a hop),
                // ripple (a field of asphalt waves across the lane, 8 of 6 cm over 24 m: the braking zones, the ABS
                // and the tyres lose the ground; one object, centred 12 m on), hump (a speed hump across the lane, 12 cm over 3.5 m: unsettles the car accelerating out)
                const auto w = ParseWords(rest);
                if (!w.empty()) {
                    const int model = w[0] == "kicker" ? 19402 : w[0] == "ripple" ? 19403 : w[0] == "hump" ? 19404 : 19401;
                    const float lat = w.size() > 1 ? static_cast<float>(atof(w[1].c_str())) : 0.0f;
                    const int count = w.size() > 2 ? std::max(atoi(w[2].c_str()), 1) : 1;
                    const float every = w.size() > 3 ? static_cast<float>(atof(w[3].c_str())) : 3.0f;
                    for (int c = 0; c < count; c++)
                        course.obst.push_back({ static_cast<int>(local.size()) - 1 + static_cast<int>(std::lround(c * every / 0.5f)), model, lat });
                }
            }
            else if ((kind == 'V' || kind == 'B' || kind == 'X') && nums.size() >= 2)
                zones.push_back({ static_cast<int>(local.size()) - 1, nums[1], nums[0], kind });
            else if (kind == 'W' && nums.size() >= 3) {
                const float amp = nums[0], sp = std::max(nums[1], 2.0f);
                const int gates = std::max(static_cast<int>(std::lround(nums[2])), 1);
                for (int k = 0; k < gates; k++) course.cones.push_back(ToWorld(inFrame(sp * (k + 0.5f), 0.0f)));
                // (the sine eased in over the first half gate and out over the last: it began at its full slope, the road
                // turning 13.5 degrees at once off the straight, "a entrada do primeiro slalom tem um join horrivel")
                const float len = sp * gates;
                sideways(len, [&](float x) { return amp * smoothstep(std::min(x, len - x) / (0.5f * sp)) * std::sin(kPi * x / sp); });
            }
            else if (kind == 'J' && nums.size() >= 4) {
                const float a = nums[0], o = std::max(nums[1], 1.0f), ho = nums[2], b = std::max(nums[3], 1.0f);
                // (the moose: through the middle of the lane, the line its test is about; from 20 m before it)
                zones.push_back({ std::max(static_cast<int>(local.size()) - 41, 0), o + ho + b + 20.0f, 0.0f, 'C' });
                for (float x : { 0.0f, o, o + ho, o + ho + b })
                    for (float side : { -1.2f, 1.2f }) {
                        const float y = (x >= o - 0.1f && x <= o + ho + 0.1f) ? a : 0.0f;
                        course.cones.push_back(ToWorld(inFrame(x, y + side)));
                    }
                sideways(o + ho + b, [&](float x) {
                    if (x < o) return a * smoothstep(x / o);
                    if (x < o + ho) return a;
                    return a * (1.0f - smoothstep((x - o - ho) / b));
                });
            }
            else if (kind == 'S' && !nums.empty()) {
                for (float m = 0.5f; m <= nums[0] + 1e-3f; m += 0.5f) local.push_back(at + CVector2D(std::cos(h), std::sin(h)) * m);
                at = local.back();
            }
            else if ((kind == 'L' || kind == 'R') && nums.size() >= 2) {
                const float sign = kind == 'L' ? 1.0f : -1.0f, r = std::max(nums[1], 1.0f), ang = nums[0] * kPi / 180.0f;
                const CVector2D c = at + CVector2D(-std::sin(h), std::cos(h)) * (sign * r);
                const int steps = std::max(static_cast<int>(ang * r / 0.5f), 1);
                for (int k = 1; k <= steps; k++) {
                    const float hh = h + sign * ang * k / steps;
                    local.push_back(c + CVector2D(std::sin(hh), -std::cos(hh)) * (sign * r));
                }
                h += sign * ang;
                at = local.back();
            }
        }
    }
    else { // line (also the drawn line of the open-loop tests)
        for (float x = 0.0f; x <= d.length; x += 0.5f) local.emplace_back(x, 0.0f);
    }
    for (const auto& q : local) course.p.push_back(ToWorld(q));
    const int n = static_cast<int>(course.p.size());
    Geometry(course);
    course.halfWidth = 0.5f * d.width;
    course.vcap.assign(n, 1e9f);
    course.vsud.assign(n, 1e9f);
    course.wind.assign(n, 0.0f);
    course.centre.assign(n, 0);
    for (const Zone& z : zones)
        for (int i = std::clamp(z.i, 0, n - 1); i < n && course.s[i] - course.s[z.i] <= z.len; i++) {
            if (z.kind == 'X') { course.wind[i] = z.value; course.centre[i] = 1; continue; }
            if (z.kind == 'C') { course.centre[i] = 1; continue; }
            float& v = z.kind == 'B' ? course.vsud[i] : course.vcap[i];
            v = std::min(v, z.value / 3.6f);
        }
    for (const auto& nm : names) course.sections.emplace_back(course.s[std::clamp(nm.first, 0, n - 1)], nm.second);
    // (only the lane changes and the gusts themselves in the middle: their whole parts were, and the line closed in
    // where nothing needed it, at the joins: the user)
    R->road = course;
    OwnLine();
    SpeedProfile(d);
}

// Nearest course point to 'pos', searched around the last one (the course may cross itself: figure 8)
static float CurveGrip(float c) {
    const float r = 1.0f / std::max(c, 1e-4f);
    const float l = std::log(r);
    if (r <= 15.0f) return cfg.gripTight;
    if (r >= 150.0f) return cfg.gripFast;
    if (r <= 45.0f) return cfg.gripTight + (1.0f - cfg.gripTight) * (l - std::log(15.0f)) / (std::log(45.0f) - std::log(15.0f));
    return 1.0f + (cfg.gripFast - 1.0f) * (l - std::log(45.0f)) / (std::log(150.0f) - std::log(45.0f));
}

static int NearestIndexIn(const Course& c, const CVector2D& pos, int hint, int window) {
    const int n = static_cast<int>(c.p.size());
    int best = hint;
    float bestD = 1e18f;
    for (int k = -window; k <= window; k++) {
        int i = hint + k;
        if (c.closed) i = ((i % n) + n) % n;
        else if (i < 0 || i >= n) continue;
        const float d = (c.p[i] - pos).MagnitudeSqr();
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}
// Where 'pos' is on the course between its points: offset (> 0 left), direction and curvature interpolated (the points'
// own values step every 0.5 m, and the heading feedback shook the wheel every frame on those steps)
struct OnCourse { int i; float frac, offset, head, curv; };
static OnCourse ProjectIn(const Course& c, const CVector2D& pos, int hint) {
    const int n = static_cast<int>(c.p.size());
    auto at = [&](int k) { return c.closed ? ((k % n) + n) % n : std::clamp(k, 0, n - 1); };
    OnCourse r{ hint, 0.0f, 0.0f, c.head[hint], c.curv[hint] };
    for (int a : { at(hint - 1), hint }) {
        const int b = at(a + 1);
        if (a == b) continue;
        const CVector2D ab = c.p[b] - c.p[a];
        const float len2 = ab.MagnitudeSqr();
        if (len2 < 1e-6f) continue;
        const float t = CVector2D::Dot(pos - c.p[a], ab) / len2;
        if (t < 0.0f || t > 1.0f) continue;
        r.i = a;
        r.frac = t;
        r.head = c.head[a] + WrapPi(c.head[b] - c.head[a]) * t;
        r.curv = c.curv[a] + (c.curv[b] - c.curv[a]) * t;
        break;
    }
    r.offset = CVector2D::Dot(pos - c.p[hint], CVector2D(-std::sin(r.head), std::cos(r.head)));
    return r;
}

static int NearestIndex(const CVector2D& pos, int hint, int window) { return NearestIndexIn(course, pos, hint, window); }
static OnCourse Project(const CVector2D& pos, int hint) { return ProjectIn(course, pos, hint); }

static int Advance(int i, float meters) {
    const int n = static_cast<int>(course.p.size());
    const int k = static_cast<int>(meters / 0.5f);
    return course.closed ? (((i + k) % n) + n) % n : std::clamp(i + k, 0, n - 1);
}

// --- Run state -------------------------------------------------------------------------------------------------------

enum eState { WAIT_GAME, NEXT_CAR, WAIT_MODEL, START_RUN, RUNNING, BETWEEN, DONE };
static eState state = WAIT_GAME;
static unsigned int stateTime = 0;
static int carIdx = -1, testIdx = 0, speedIdx = 0, runCount = 0;
static float clockT = 0.0f;            // s of running, every lane together
static const TestDef* curTest = nullptr;
static int curModel = 0;
static std::ofstream trace;            // (Trace)
// (Trace, steady state: a file per generation still running, its rows kept here and handed to Disk 1 MB at a time)
struct TraceBuf { std::string data; bool started = false; };
static std::map<int, TraceBuf> traceGen;
static void CloseTrace(int g);
static float traceNext[kMaxRunners];
static bool courseWritten = false;
static float firstHome = -1.0f;        // the run's first car at the end (s; -1: none yet)
static int camPick = -1;               // (UpdateCamera) -1: the leader
static int camOn = 0;                  // the car the camera is on


static unsigned int Now() { return CTimer::m_snTimeInMilliseconds; }

// (the summary is kept here and handed to Disk once a second, Perf::Frame: written on the game's thread, a write waited
// up to 16 s while the antivirus looked at the trace files)
static std::string logBuf, logPath;
static void Log(const std::string& s) {
    if (logPath.empty()) return;
    logBuf += s;
    logBuf += "\n";
}

// --- Disk: files written on a thread of their own --------------------------------------------------------------------
// (opening and closing a file, with the antivirus looking at it, can take tens of ms, and the game's one thread waited:
// the stutters when a generation ended, its history, state and trace files). A whole file goes to .tmp first, then over
// the old one: a state file is never half-written when the game is closed
namespace Disk {
struct Job { std::string path, data; bool append, close; };
static std::mutex mtx;
static std::condition_variable cv;
static std::deque<Job> jobs;
static bool started = false;
// (an appended file stays open until Close: opened and closed for every MB, the antivirus read the whole growing file
// again each time; 50 MB a generation)
static std::map<std::string, FILE*> files;
static void Loop() {
    for (;;) {
        Job j;
        {
            std::unique_lock<std::mutex> l(mtx);
            cv.wait(l, [] { return !jobs.empty(); });
            j = std::move(jobs.front());
            jobs.pop_front();
        }
        auto it = files.find(j.path);
        if (j.close || !j.append) {
            if (it != files.end()) { std::fclose(it->second); files.erase(it); }
            if (j.close) continue;
        }
        else {
            FILE* f = it != files.end() ? it->second : std::fopen(j.path.c_str(), "ab");
            if (!f) continue;
            files[j.path] = f;
            std::fwrite(j.data.data(), 1, j.data.size(), f);
            std::fflush(f);
            continue;
        }
        const std::string to = j.path + ".tmp";
        if (FILE* f = std::fopen(to.c_str(), "wb")) {
            std::fwrite(j.data.data(), 1, j.data.size(), f);
            std::fclose(f);
            MoveFileExA(to.c_str(), j.path.c_str(), MOVEFILE_REPLACE_EXISTING);
        }
    }
}
static void Write(const std::string& path, std::string data, bool append = false) {
    {
        std::lock_guard<std::mutex> l(mtx);
        if (!started) { started = true; std::thread(Loop).detach(); }
        jobs.push_back({ path, std::move(data), append, false });
    }
    cv.notify_one();
}
static void Close(const std::string& path) {
    {
        std::lock_guard<std::mutex> l(mtx);
        jobs.push_back({ path, std::string(), true, true });
    }
    cv.notify_one();
}
static int Pending() { std::lock_guard<std::mutex> l(mtx); return static_cast<int>(jobs.size()); }
} // namespace Disk

// --- Perf: where a frame's time goes (the summary every 10 s, "perf:"; a frame over 150 ms on its own line) ----------
// GTA runs on one thread: the CPU meter shows the machine's cores together, so a game stuck on one core reads ~10%
namespace Perf {
enum { UPDATE, CONTROL, DRAW, REPLACE, ENDBATCH, TRACE, HUD, N };
static const char* const kNames[N] = { "update", "controle", "desenho", "troca de cerebro", "fim de geracao", "trace", "hud" };
static double cur[N] = {}, sum[N] = {}, peak[N] = {}, shown[N] = {};
static double last = 0.0, windowStart = 0.0, frameSum = 0.0, frameMax = 0.0;
static int frames = 0;
static float fps = 0.0f, frameMs = 0.0f, modMs = 0.0f;
static double Now() {
    LARGE_INTEGER c, f;
    QueryPerformanceCounter(&c);
    QueryPerformanceFrequency(&f);
    return 1000.0 * static_cast<double>(c.QuadPart) / static_cast<double>(f.QuadPart);
}
struct Scope { int i; double t0; explicit Scope(int k) : i(k), t0(Now()) {} ~Scope() { cur[i] += Now() - t0; } };
static void Frame(bool running) {
    const double t = Now();
    if (last <= 0.0) windowStart = t;
    else {
        const double f = t - last;
        frames++;
        frameSum += f;
        frameMax = std::max(frameMax, f);
        if (f > 150.0 && running) {
            std::string s = Format("perf: a frame of %.0f ms:", f);
            for (int i = 0; i < N; i++) if (cur[i] > 1.0) s += Format(" %s %.0f", kNames[i], cur[i]);
            Log(s);
        }
    }
    for (int i = 0; i < N; i++) { sum[i] += cur[i]; peak[i] = std::max(peak[i], cur[i]); cur[i] = 0.0; }
    last = t;
    static double lastLog = 0.0;
    if (t - lastLog > 1000.0 && !logBuf.empty()) { lastLog = t; Disk::Write(logPath, std::move(logBuf), true); logBuf.clear(); }
    if (t - windowStart >= 10000.0 && frames > 0) {
        fps = static_cast<float>(1000.0 * frames / (t - windowStart));
        frameMs = static_cast<float>(frameSum / frames);
        std::string s = Format("perf: %.0f fps, frame %.1f ms (max %.0f); the mod (ms a frame):", fps, frameMs, frameMax);
        for (int i = 0; i < N; i++) {
            shown[i] = sum[i] / frames;
            s += Format(" %s %.2f (max %.0f)", kNames[i], shown[i], peak[i]);
            sum[i] = peak[i] = 0.0;
        }
        // (troca, fim de geracao and trace are inside update; hud inside desenho)
        modMs = static_cast<float>(shown[UPDATE] + shown[CONTROL] + shown[DRAW]);
        Log(s + Format("; in all %.1f of %.1f; disk queue %d", modMs, frameMs, Disk::Pending()));
        frames = 0;
        frameSum = frameMax = 0.0;
        windowStart = t;
    }
}
} // namespace Perf

// The road's obstacles on this car's lane, once (static objects, the same on every lane)
static void SpawnObstacles() {
    const Course& rd = R->road;
    const int n = static_cast<int>(rd.p.size());
    if (!R->obstacles.empty() || rd.obst.empty() || n < 2) return;
    for (int m : { 19401, 19402, 19403, 19404 })
        if (!Command<COMMAND_HAS_MODEL_LOADED>(m)) { Command<COMMAND_REQUEST_MODEL>(m); Command<COMMAND_LOAD_ALL_MODELS_NOW>(); }
    for (const auto& o : rd.obst) {
        const int i = std::clamp(o.i, 0, n - 1);
        const float h = rd.head[i];
        const CVector2D q = rd.p[i] + CVector2D(-std::sin(h), std::cos(h)) * (o.lat * rd.halfWidth);
        int handle = -1;
        // (a full object pool: the game's create command writes through a null object and crashes; 500 lanes of
        // obstacles came to 23,000 objects: the crash at 20:17)
        if (CPools::ms_pObjectPool->GetNoOfFreeSpaces() < 300) { Log("obstacles: the object pool is nearly full, the rest left out"); break; }
        Command<COMMAND_CREATE_OBJECT>(o.model, q.x, q.y, R->baseZ, &handle);
        if (handle < 0) continue;
        Command<COMMAND_SET_OBJECT_HEADING>(handle, h * 180.0f / 3.14159265f - 90.0f);
        R->obstacles.push_back(handle);
    }
}
static void RemoveObstacles() {
    for (int h : R->obstacles) Command<COMMAND_DELETE_OBJECT>(h);
    R->obstacles.clear();
}

static void RemoveCar() {
    if (carHandle >= 0 && car && CPools::ms_pVehiclePool->IsObjectValid(car)) {
        CPlayerPed* player = FindPlayerPed();
        if (player && player->m_pVehicle == car)
            Command<COMMAND_WARP_CHAR_FROM_CAR_TO_COORD>(CPools::GetPedRef(player), cfg.origin.x - 30.0f, cfg.origin.y - 30.0f, cfg.origin.z + 1.0f);
        if (driverHandle >= 0) Command<COMMAND_DELETE_CHAR>(driverHandle);
        Command<COMMAND_DELETE_CAR>(carHandle);
    }
    carHandle = driverHandle = -1;
    car = nullptr;
}

static void OpenRunDir() {
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    CreateDirectoryA((baseDir + "proving").c_str(), nullptr);
    runDir = baseDir + "proving\\" + stamp + "\\";
    CreateDirectoryA(runDir.c_str(), nullptr);
    if (!logPath.empty() && !logBuf.empty()) Disk::Write(logPath, std::move(logBuf), true);
    logBuf.clear();
    if (!logPath.empty()) Disk::Close(logPath);
    logPath = runDir + "summary.txt";
    Disk::Write(logPath, std::string()); // (a new, empty one)
    std::ofstream latest(baseDir + "proving\\latest.txt", std::ios::trunc);
    latest << stamp << "\n";
    CopyFileA(iniPath.c_str(), (runDir + "proving.ini").c_str(), FALSE);
}

static void Restart() {
    forRunners { RemoveCar(); RemoveObstacles(); if (csv.is_open()) csv.close(); }
    R = runners;
    if (trace.is_open()) trace.close();
    while (!traceGen.empty()) CloseTrace(traceGen.begin()->first);
    courseWritten = false;
    if (camOn) Command<COMMAND_RESTORE_CAMERA_JUMPCUT>();
    camOn = 0;
    camPick = -1;
    ReadSettings();
    OpenRunDir();
    GA::Start();
    carIdx = -1;
    runCount = 0;
    state = cfg.pause ? DONE : NEXT_CAR;
    stateTime = Now();
    Log(Format("Proving ground: origin (%.1f, %.1f, %.1f) heading %.0f, driver %s, steer %s, %d cars x %d tests%s",
        cfg.origin.x, cfg.origin.y, cfg.origin.z, cfg.headingDeg, cfg.driver.c_str(), cfg.mode.c_str(),
        static_cast<int>(cfg.cars.size()), static_cast<int>(cfg.tests.size()), numRunners > 1 ? Format(
            ", A/B: B %.0f m to the right, steer %s bandwidth %.2f/%.2f, grip factor %.2f/%.2f, brake g %.2f/%.2f", laneShiftB,
            CfgOf(1).mode.c_str(), CfgOf(0).bandwidth, CfgOf(1).bandwidth, CfgOf(0).gripFactor,
            CfgOf(1).gripFactor, CfgOf(0).speedBrakeG, CfgOf(1).speedBrakeG).c_str() : ""));
}

static bool IniChanged() {
    WIN32_FILE_ATTRIBUTE_DATA a;
    if (!GetFileAttributesExA(iniPath.c_str(), GetFileExInfoStandard, &a)) return false;
    if (CompareFileTime(&a.ftLastWriteTime, &iniStamp) != 0) { iniStamp = a.ftLastWriteTime; return true; }
    return false;
}

// --- Control (called right before the car's physics) -------------------------------------------------------------------

// --- Learning by result ------------------------------------------------------------------------------------------------
// While it drives each car tries small changes and keeps the ones that paid off, judged the way the score judges a run
// (speed and the line; no rule against sliding: sideways may well be faster). In every curve one number for that kind
// of curve (CurveBand) goes up or down by a step: its grip (the speed it plans) or the gas its TC lets through. At the
// curve's end its result, speed / the most the tyres allow at that radius - 0.67 x mean offset (m) (1% of speed ~ 0.015
// m of offset: the score's 1 s per 100 x mean offset over a ~150 s run) - 0.5 if it went 2 m off, against its own
// average for that kind: better -> kept, worse -> put back. Every braking the same for its ABS limit (deceleration / the
// AI grip). The step is 0.03 + 0.25 x LearnRate.
static std::minstd_rand learnRng(12345);
static float LearnU() { return std::uniform_real_distribution<float>(0.0f, 1.0f)(learnRng); }

static float* TrialValue(const Trial& t) {
    if (t.which == 0) return &R->learned[t.band];
    if (t.which == 1) return &R->tcBand[t.band];
    return &R->absLearn;
}

static void EndTrial(Trial& t, float& base) {
    float* v = TrialValue(t);
    if (t.t >= 0.5f && t.n > 0) {
        const float r = t.sum / t.n - 0.67f * t.off / t.n - (t.maxOff > 2.0f ? 0.5f : 0.0f);
        R->trials++;
        if (!std::isnan(base) && r < base) *v = t.old; // worse than usual: back
        else R->kept++;
        base = std::isnan(base) ? r : 0.7f * base + 0.3f * r;
    }
    else *v = t.old; // too short to tell
    t = Trial();
}

// What a frame costs in a trial: the offset from the centre line (m), or with a corridor the room closer than
// EdgeComfort to an edge (+1 past it); and > 2 when the car is out (the trial's -0.5)
static float LinePenalty() {
    if (course.halfWidth <= 0.0f) return std::abs(offsetNow);
    return std::max(edgeComfortGA - R->marginNow, 0.0f) + (R->marginNow < 0.0f ? 1.0f : 0.0f);
}
static float LineOut() { return course.halfWidth <= 0.0f ? std::abs(offsetNow) : R->marginNow < 0.0f ? 3.0f : 0.0f; }

static void LearnByResult(float speed, float dt, float brakeAsked) {
    const float step = 0.03f + 0.25f * cfg.learnRate;
    const float G = aiGripG * 9.81f;
    // a curve: a trial per curve (a new one when the kind changes); its grip only: the ABS and the TC keep the values
    // its genes give all through the run (the user: the GA changes them between generations, the best brain wins)
    const float c = std::abs(course.curv[nearIdx]);
    const int band = c > 1.0f / 400.0f ? CurveBand(c) : -1;
    Trial& ct = R->curveTrial;
    if (band != ct.band) {
        if (ct.band >= 0) EndTrial(ct, R->bandBase[ct.band]);
        if (band >= 0) {
            ct.band = band;
            ct.which = 0;
            float* v = TrialValue(ct);
            ct.old = *v;
            const float lo = ct.which == 0 ? 0.6f : 0.5f, hi = ct.which == 0 ? 1.6f : 2.5f;
            *v = std::clamp(*v + (LearnU() < 0.5f ? -step : step), lo, hi);
        }
    }
    if (ct.band >= 0) {
        ct.sum += std::min(speed / std::sqrt(G / c), 2.0f);
        ct.off += LinePenalty();
        ct.maxOff = std::max(ct.maxOff, LineOut());
        ct.t += dt;
        ct.n++;
    }
}

static float HandlingLockRad(CVehicle* v) { return v->m_pHandlingData->m_fSteeringLock * 3.14159265f / 180.0f; }

// What the neural driver sees (also logged for every car, for a human's lap to teach it: tools/nn_clone.py)
static void NeuralInputs(CVehicle* v, float* in) {
    const Course& rd = R->road;
    const CVector2D pos = v->GetPosition().To2D(), fwd = v->GetForward().To2D();
    const float carHead = std::atan2(fwd.y, fwd.x), half = rd.halfWidth > 0.0f ? rd.halfWidth : 2.0f;
    const int n = static_cast<int>(rd.p.size());
    for (int r = 0; r < 11; r++) {
        const float a = carHead + (-90.0f + 18.0f * r) * 3.14159265f / 180.0f;
        const CVector2D dir(std::cos(a), std::sin(a));
        float dist = 50.0f;
        int hint = std::clamp(nearIdx, 0, std::max(n - 1, 0));
        for (float t = 2.0f; t <= 50.0f && n > 1; t += 2.0f) {
            const CVector2D pt = pos + dir * t;
            hint = NearestIndexIn(rd, pt, hint, 10);
            if (std::abs(ProjectIn(rd, pt, hint).offset) > half) { dist = t; break; }
        }
        in[r] = dist / 50.0f;
    }
    const float lock = HandlingLockRad(v);
    in[11] = CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS / 80.0f;
    in[12] = CVector::Dot(v->m_vecMoveSpeed, v->GetRight()) * GAME_SPEED_TO_MPS / 10.0f;
    in[13] = v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS / 2.0f;
    in[14] = v->m_fSteerAngle / lock;
    for (int k = 0; k < 3; k++) {
        const float ahead = k == 0 ? 20.0f : k == 1 ? 50.0f : 100.0f;
        const int j = n > 0 ? std::clamp(nearIdx + static_cast<int>(ahead / 0.5f), 0, n - 1) : 0;
        in[15 + k] = n > 0 ? std::clamp(rd.curv[j] * 20.0f, -1.0f, 1.0f) : 0.0f;
    }
}

// What neural2 sees (also logged for its teachers and itself: tools/nn2_train.py)
static void Neural2InputsTo(CVehicle* v, float* in);
// (into a buffer with room to spare, then the kNn2In of them: a count off by one wrote past R->nnIn into the runner's
// obstacles, Gemini's review of the 85 inputs; the count is checked and logged)
static void Neural2Inputs(CVehicle* v, float* in) {
    float buf[kNn2In + 32];
    Neural2InputsTo(v, buf);
    std::memcpy(in, buf, sizeof(float) * kNn2In);
}
static int nn2Count = 0; // inputs written (Neural2InputsTo), for the check
static void Neural2InputsTo(CVehicle* v, float* in) {
    const Course& rd = R->road;
    const int n = static_cast<int>(rd.p.size());
    const CVector2D pos = v->GetPosition().To2D(), fwd = v->GetForward().To2D(), rt(fwd.y, -fwd.x);
    const float carHead = std::atan2(fwd.y, fwd.x), half = rd.halfWidth > 0.0f ? rd.halfWidth : 2.0f;
    const int ni = std::clamp(nearIdx, 0, std::max(n - 1, 0));
    int k = 0;
    // 19 rays to the road's edges, every 10 degrees over the front 180
    for (int r = 0; r < 19; r++) {
        const float a = carHead + (-90.0f + 10.0f * r) * 3.14159265f / 180.0f;
        const CVector2D dir(std::cos(a), std::sin(a));
        float dist = 60.0f;
        int hint = ni;
        // (every 4 m, then the 2 m before the first point out: the same answer as every 2 m, half the work; 160
        // neural2 cars x 19 rays x 30 points took 5 ms a frame)
        for (float t = 4.0f; t <= 60.0f && n > 1; t += 4.0f) {
            const CVector2D pt = pos + dir * t;
            hint = NearestIndexIn(rd, pt, hint, 14);
            if (std::abs(ProjectIn(rd, pt, hint).offset) > half) {
                const CVector2D pb = pos + dir * (t - 2.0f);
                dist = std::abs(ProjectIn(rd, pb, NearestIndexIn(rd, pb, hint, 8)).offset) > half ? t - 2.0f : t;
                break;
            }
        }
        in[k++] = dist / 60.0f;
    }
    // the road and its own line: offset, heading error; the room each side (the body's corners)
    const OnCourse oc = n > 1 ? ProjectIn(rd, pos, ni) : OnCourse{ 0, 0, 0, carHead, 0 };
    in[k++] = oc.offset / half;
    in[k++] = offsetNow / half;
    in[k++] = WrapPi(carHead - oc.head);
    in[k++] = headErrNow;
    float latMax = -99.0f, latMin = 99.0f;
    for (float fy : { R->bodyFront, R->bodyRear })
        for (float sx : { -1.0f, 1.0f }) {
            const CVector2D corner = pos + fwd * fy + rt * (sx * R->bodyHalfW);
            const float lat = n > 1 ? ProjectIn(rd, corner, NearestIndexIn(rd, corner, ni, 16)).offset : 0.0f;
            latMax = std::max(latMax, lat);
            latMin = std::min(latMin, lat);
        }
    in[k++] = (half - latMax) / 2.0f;
    in[k++] = (half + latMin) / 2.0f;
    // the road's curvature ahead, the next curve, the next obstacle, the progress
    auto at = [&](float m) { return std::clamp(ni + static_cast<int>(m / 0.5f), 0, std::max(n - 1, 0)); };
    for (float m : { 0.0f, 10.0f, 25.0f, 50.0f, 100.0f, 200.0f }) in[k++] = n ? std::clamp(rd.curv[at(m)] * 20.0f, -1.0f, 1.0f) : 0.0f;
    float curveD = 1.0f, curveK = 0.0f;
    for (float m = 0.0f; m <= 300.0f && n; m += 2.0f)
        if (std::abs(rd.curv[at(m)]) > 1.0f / 150.0f) { curveD = m / 300.0f; curveK = std::clamp(rd.curv[at(m)] * 20.0f, -1.0f, 1.0f); break; }
    in[k++] = curveD;
    in[k++] = curveK;
    float obD = 1.0f, obLat = 0.0f, obKind = 0.0f;
    for (const auto& o : rd.obst)
        if (o.i >= ni && n) { obD = std::min((rd.s[std::min(o.i, n - 1)] - rd.s[ni]) / 100.0f, 1.0f); obLat = o.lat; obKind = o.model == 19402 ? -1.0f : 1.0f; break; }
    in[k++] = obD;
    in[k++] = obLat;
    in[k++] = obKind;
    in[k++] = n ? rd.s[ni] / std::max(rd.length, 1.0f) : 0.0f;
    // the car's motion
    const CVector vel = v->m_vecMoveSpeed * GAME_SPEED_TO_MPS, turn = v->m_vecTurnSpeed * GAME_SPEED_TO_MPS;
    const float vF = CVector::Dot(vel, v->GetForward()), vR = CVector::Dot(vel, v->GetRight());
    in[k++] = vF / 80.0f;
    in[k++] = vR / 10.0f;
    in[k++] = vel.z / 10.0f;
    in[k++] = turn.z / 2.0f;
    in[k++] = CVector::Dot(turn, v->GetForward()) / 2.0f;   // roll rate
    in[k++] = CVector::Dot(turn, v->GetRight()) / 2.0f;     // pitch rate
    in[k++] = std::clamp(R->aLongNow / 20.0f, -2.0f, 2.0f);
    in[k++] = std::clamp(R->latGNow / 2.0f, -2.0f, 2.0f);
    in[k++] = std::atan2(vR, std::max(std::abs(vF), 0.5f));
    in[k++] = std::atan2(vR - v->m_vecTurnSpeed.z * rearAxleY * GAME_SPEED_TO_MPS, std::max(std::abs(vF), 0.5f));
    in[k++] = v->GetRight().z;                              // roll
    in[k++] = v->GetForward().z;                            // pitch
    // the wheel it asked for last frame, what its teacher asks for now, how far it is from the planner's speed
    // (steerOut and targetKmh are last frame's until Control writes this frame's, teacherSteer is this frame's (set
    // just before, Neural2Snapshot): in the trace these are the numbers the net saw, nothing of this frame's command;
    // the old gasOut / brakeOut here were that command)
    in[k++] = steerOut / HandlingLockRad(v);
    in[k++] = R->teacherSteer / HandlingLockRad(v);
    in[k++] = std::clamp((targetKmh / 3.6f - CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS) / 50.0f, -2.0f, 2.0f);
    // each wheel: its state (rolling, spinning, skidding, locked), its suspension, on the ground or not
    CAutomobile* a = static_cast<CAutomobile*>(v);
    int contact = 0;
    for (int w = 0; w < 4; w++) in[k++] = static_cast<float>(a->m_wheelState[w]) / 3.0f;
    for (int w = 0; w < 4; w++) { const float c = a->m_fWheelsSuspensionCompressionPrev[w]; in[k++] = c; contact += c < 1.0f; }
    in[k++] = contact / 4.0f;
    in[k++] = contact == 0 ? 1.0f : 0.0f;
    // the grip in use, the crosswind
    in[k++] = std::clamp(std::abs(R->latGNow) / std::max(aiGripG, 0.1f), 0.0f, 2.0f);
    // 22 more (Gemini's list from the plugin-sdk headers, checked; the car's constants left out: one model, they never
    // change): each wheel's slip (its rim speed against the ground's), its damper speed, the ground's tilt under it
    const float vFGame = CVector::Dot(v->m_vecMoveSpeed, v->GetForward());
    for (int w = 0; w < 4; w++)
        in[k++] = std::clamp((a->m_fWheelSpeed[w] * 0.5f * v->m_fWheelScale - vFGame) * GAME_SPEED_TO_MPS / 20.0f, -1.0f, 1.0f);
    // (the damper per frame step: the same at any frame rate)
    for (int w = 0; w < 4; w++)
        in[k++] = std::clamp((a->m_fWheelsSuspensionCompressionPrev[w] - a->m_fWheelsSuspensionCompression[w]) * 5.0f / std::max(CTimer::ms_fTimeStep, 0.2f), -1.0f, 1.0f);
    // (not the ground's tilt under each wheel: on the flat platform the car's own roll, in already; nor the wheelspin
    // and skid the game hears: only the cars with sound, ~2 of 500, have them)
    // the gearbox: the gear and how far into it (an rpm), the wheelspin and the skid the game hears, drive wheels down
    const cTransmission& tr = v->m_pHandlingData->m_transmissionData;
    const int gear = std::clamp<int>(v->m_nCurrentGear, 0, std::clamp<int>(tr.m_nNumberOfGears, 1, 5));
    in[k++] = static_cast<float>(gear) / std::max<int>(tr.m_nNumberOfGears, 1);
    in[k++] = std::clamp(vFGame / std::max(std::abs(tr.m_aGears[gear].m_fMaxVelocity), 0.05f), -1.0f, 1.0f);
    in[k++] = a->m_nNumDriveWheelsOnGroundLastFrame / 4.0f;
    // what the tyres' friction took off the car this frame, sideways and in yaw
    in[k++] = std::clamp(CVector::Dot(v->m_vecFrictionMoveSpeed, v->GetRight()) * GAME_SPEED_TO_MPS / 2.0f, -1.0f, 1.0f);
    in[k++] = std::clamp(v->m_vecFrictionTurnSpeed.z * GAME_SPEED_TO_MPS / 0.5f, -1.0f, 1.0f);
    // its line: the racing line's offset 20, 50 and 100 m on (of the room), the line it asked for last frame
    for (float m : { 20.0f, 50.0f, 100.0f }) {
        const int at = n ? Advance(ni, m) : 0;
        in[k++] = at < static_cast<int>(raceOff.size()) && R->lineRoom > 0.0f ? raceOff[at] / R->lineRoom : 0.0f;
    }
    // (the crosswind 0.15 s on, m/s2 / 3, in place of the line it asked for last frame: the net could not see the gusts)
    in[k++] = course.wind.empty() ? 0.0f : course.wind[Advance(ni, std::max(std::abs(vFGame) * GAME_SPEED_TO_MPS, 5.0f) * 0.15f)] / 3.0f;
    if (k != nn2Count) { nn2Count = k; if (k != kNn2In) Log(Format("neural2: %d inputs written for %d: check Neural2InputsTo", k, kNn2In)); }
    while (k < kNn2In) in[k++] = 0.0f;
}

// neural2's teacher: what pursuit with this car's genes would steer here (DAgger's label, one of the inputs, and the
// wheel its residual is added to). Worked out for every car the trace keeps neural2's sensors for (pursuit, rpp and
// blend too), so their rows carry the same label: the residual they drove over pursuit is what tools/nn2_train.py learns
static float PursuitSteer(CVehicle* v, float vv, int i) {
    const float lock = HandlingLockRad(v);
    const float ld = std::clamp(cfg.lookTime * vv, cfg.lookMin, cfg.lookMax) * (cfg.mode == "neural2" ? std::exp(0.5f * R->nnLook) : 1.0f);
    const CVector2D pos = v->GetPosition().To2D(), fwd = v->GetForward().To2D(), left(-fwd.y, fwd.x);
    const int at = Advance(i, ld);
    CVector2D aim = course.p[at];
    if (R->nnLine != 0.0f && R->lineRoom > 0.0f) {
        // (the room is the lane's from the middle: the racing line's offset there already used part of it)
        const float base = at < static_cast<int>(raceOff.size()) ? raceOff[at] : 0.0f;
        const float lim = at < static_cast<int>(raceRoom.size()) ? std::min(raceRoom[at], R->lineRoom) : R->lineRoom;
        const float shift = std::clamp(base + R->nnLine * R->lineRoom, -lim, lim) - base;
        aim += CVector2D(-std::sin(course.head[at]), std::cos(course.head[at])) * shift;
    }
    const CVector2D rel = aim - pos;
    return std::clamp(std::atan2(2.0f * wheelBase * std::sin(std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd))), ld) * cfg.gain, -lock, lock);
}
// the teacher, then the sensors of this frame (the teacher is one of them), kept for the net and for Trace
static void Neural2Snapshot(CVehicle* v, float vv, int i) {
    R->teacherSteer = PursuitSteer(v, vv, i);
    // the recovery pursuit lacks (the user: it lost the car at high speed and did not get it back): the rear axle's
    // slip angle past 0.08 rad steered against, and the rotation past what the line asks for at this speed (yaw rate
    // - v x its curvature) taken off as the wheel that makes it (wheelbase x the excess / v): the damping acts before
    // the slide grows. Steering > 0 is to the left; the rear going right (> 0) means the car turns left too much.
    // CounterSteer: their strength (the GA's)
    {
        const float lock = HandlingLockRad(v);
        const float vFwd = CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS;
        const float vRear = (CVector::Dot(v->m_vecMoveSpeed, v->GetRight()) - v->m_vecTurnSpeed.z * rearAxleY) * GAME_SPEED_TO_MPS;
        const float beta = vFwd > 5.0f ? std::atan2(vRear, vFwd) : 0.0f;
        const float betaEx = beta > 0.0f ? std::max(beta - 0.08f, 0.0f) : std::min(beta + 0.08f, 0.0f);
        const float yaw = v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS;
        const float want = vFwd * (course.curv.empty() ? 0.0f : course.curv[std::clamp(i, 0, static_cast<int>(course.curv.size()) - 1)]);
        float yawEx = yaw - want;
        yawEx = yawEx > 0.0f ? std::max(yawEx - 0.05f, 0.0f) : std::min(yawEx + 0.05f, 0.0f);
        // (only the rotation: the slip angle of the rear counted too, and a crosswind gust pushing the whole car sideways
        // read as a slide, so it steered with the wind, out of the road at 450 km/h: 426-439 of 500 spun, g149-151)
        (void)betaEx;
        const float cs = cfg.counterSteer * (-yawEx * wheelBase / std::max(vFwd, 5.0f));
        // the crosswind fed forward: the wheel whose sideways pull (v^2 x steer / wheelbase) cancels the gust's push, from
        // 0.15 s before it (the gusts are the course's: a driver leans into them; pushed first, 152 of 500 went off at
        // 450 km/h, g156). (> 0 pushes to the left: the wheel to the right, < 0)
        const float windAhead = course.wind.empty() ? 0.0f : course.wind[Advance(i, std::max(vFwd, 5.0f) * 0.15f)];
        const float ff = -wheelBase * windAhead / std::max(vFwd * vFwd, 25.0f);
        R->teacherSteer = std::clamp(R->teacherSteer + cs + ff, -lock, lock);
    }
    Neural2Inputs(v, R->nnIn);
}

static float Neural2Steer(CVehicle* v, float vv, int i) {
    float h[kNn2Hid];
    // (every frame: its outputs held a frame in between, for speed, made it worse at once, g65: 19 of 340 home)
    const float lock = HandlingLockRad(v);
    Neural2Snapshot(v, vv, i);
    const float* in = R->nnIn;
    const std::vector<float>& w = cfg.nn;
    if (static_cast<int>(w.size()) < kNN2) { R->nnPedal = 0.0f; return R->teacherSteer; }
    for (int j = 0; j < kNn2Hid; j++) {
        const float* row = &w[j * (kNn2In + 1)];
        float sum = row[kNn2In];
        for (int q = 0; q < kNn2In; q++) sum += row[q] * in[q];
        h[j] = std::tanh(sum);
    }
    float h2[kNn2Hid2];
    for (int j = 0; j < kNn2Hid2; j++) {
        const float* row = &w[kNn2L2 + j * (kNn2Hid + 1)];
        float sum = row[kNn2Hid];
        for (int q = 0; q < kNn2Hid; q++) sum += row[q] * h[q];
        h2[j] = std::tanh(sum);
    }
    float o[kNn2Out];
    for (int m = 0; m < kNn2Out; m++) {
        const float* row = &w[kNn2L3 + m * (kNn2Hid2 + 1)];
        float sum = row[kNn2Hid2];
        for (int j = 0; j < kNn2Hid2; j++) sum += row[j] * h2[j];
        o[m] = std::tanh(sum);
    }
    // residuals on the teacher (zero output weights: the teacher itself); the third: its line, across the lane from
    // the racing line, up to the room (the user: the AI itself in charge of its line; next frame's teacher aims there)
    R->nnPedal = o[1];
    R->nnLine = o[2];
    // its traction control, its ABS, its handbrake (0: as the car's genes set them; the user wanted them the net's too)
    R->nnTc = o[3];
    R->nnAbs = o[4];
    R->nnHb = o[5];
    // its lookahead, and the grip it takes for each kind of curve (the multipliers by radius the HUD showed: before
    // learned in the run, LearnByResult; now the net's, every frame, x e^(0.4 out), 0: the planner's own)
    R->nnLook = o[6];
    R->nnAgg = o[7];
    // (o[7..11] were the grip x for each kind of curve, through the planner's speed: 37 of 500 home in g115 and g116,
    // and the user found speed by the curve's distance senseless; kept in the net, unused)
    std::memcpy(R->nnAct, h, sizeof(float) * kNn2Hid);
    std::memcpy(R->nnAct + kNn2Hid, h2, sizeof(float) * kNn2Hid2);
    std::memcpy(R->nnAct + kNn2Hid + kNn2Hid2, o, sizeof(float) * kNn2Out);
    R->nnO0 = o[0];
    R->nnO1 = o[1];
    R->nnHeld = true;
    return std::clamp(R->teacherSteer + kNn2SteerSpan * lock * o[0], -lock, lock);
}

static float NeuralSteer(CVehicle* v) {
    float in[kNnIn], h[kNnHid];
    NeuralInputs(v, in);
    const std::vector<float>& w = cfg.nn;
    if (static_cast<int>(w.size()) < kNN) { R->nnPedal = 0.0f; return 0.0f; }
    for (int j = 0; j < kNnHid; j++) {
        const float* row = &w[j * (kNnIn + 1)];
        float sum = row[kNnIn];
        for (int i = 0; i < kNnIn; i++) sum += row[i] * in[i];
        h[j] = std::tanh(sum);
    }
    float o[2];
    for (int k = 0; k < 2; k++) {
        const float* row = &w[kNnHid * (kNnIn + 1) + k * (kNnHid + 1)];
        float sum = row[kNnHid];
        for (int j = 0; j < kNnHid; j++) sum += row[j] * h[j];
        o[k] = std::tanh(sum);
    }
    R->nnPedal = o[1];
    return o[0] * HandlingLockRad(v);
}

static float ClosedLoopSteer(CVehicle* v, float speed) {
    const CVector2D pos = v->GetPosition().To2D();
    const CVector2D fwd = v->GetForward().To2D();
    const CVector2D left(-fwd.y, fwd.x);
    const float L = wheelBase;
    const float carHead = std::atan2(fwd.y, fwd.x);
    const int i = nearIdx;
    const CVector2D tdir(std::cos(course.head[i]), std::sin(course.head[i]));
    const float e = CVector2D::Dot(pos - course.p[i], CVector2D(-tdir.y, tdir.x)); // > 0: left of the course
    const float psi = WrapPi(carHead - course.head[i]);                              // > 0: pointing left of it
    offsetNow = e;
    headErrNow = psi;
    curvNow = course.curv[i];
    const float vv = std::max(speed, 3.0f);
    float d = 0.0f;
    if (cfg.mode == "stanley") {
        // at the front axle
        const CVector2D front = pos + fwd * (0.5f * L);
        const int fi = NearestIndex(front, i, 20);
        // (between the course's points, as the kinematic mode: their own directions step every 0.5 m)
        const OnCourse oc = Project(front, fi);
        const float ef = oc.offset;
        const float pf = WrapPi(carHead - oc.head);
        d = -pf - std::atan(cfg.stanleyK * ef / (vv + cfg.stanleySoft));
        if (cfg.feedForward) d += std::atan(L * course.curv[Advance(oc.i, cfg.preview * vv + oc.frac * 0.5f)]);
        lookPoint = course.p[fi];
    }
    else if (cfg.mode == "kinematic" || cfg.mode == "gt4") {
        // Below the grip GTA's tyres cancel the sideways speed at the contact every frame (CVehicle::ProcessWheel): no
        // slip, the car turns at exactly v tan(steer) / L with no lag (measured: L from the yaw = the wheelbase). So the
        // wheel for the course's own curvature is known (feedforward), and only the error is fed back. At the rear axle
        // (the point that moves along the car's heading in that model) in the course frame:
        //   e' = v sin(psi),  psi' = v (k - kCourse)
        // k = kCourse - wn^2 / v^2 e - 2 zeta wn / v sin(psi): the error decays as a critically damped spring of wn rad/s
        // at every speed
        const CVector2D rear = pos + fwd * rearAxleY;
        const int ri = NearestIndex(rear, i, 40);
        const OnCourse oc = Project(rear, ri);
        const float er = oc.offset;
        const float pr = WrapPi(carHead - oc.head);
        const int ahead = Advance(oc.i, cfg.preview * vv + oc.frac * 0.5f);
        const float kRef = cfg.feedForward ? (cfg.preview > 0.0f ? course.curv[ahead] : oc.curv) : 0.0f;
        const float wn = cfg.bandwidth;
        const float k = kRef - wn * wn / (vv * vv) * er - 2.0f * cfg.damping * wn / vv * std::sin(pr);
        d = std::atan(L * k);
        lookPoint = course.p[ahead];
    }
    else if (cfg.mode == "mod") {
        // The mod's own predictive steering (LaneControllerSteer) on the course as its lane line
        CVector2D ctrl[90];
        int m = 0;
        for (float a = -30.0f; a <= 200.0f && m < 90; a += 5.0f) {
            int k = Advance(i, a);
            if (!course.closed && a > 0.0f && k == static_cast<int>(course.p.size()) - 1 && m > 0 && (course.p[k] - ctrl[m - 1]).Magnitude() < 1.0f) break;
            ctrl[m++] = course.p[k];
        }
        LaneLine line;
        SplineInto(ctrl, m, line);
        ExtendedData& x = vehExData.Get(v);
        x.bRacer = cfg.racer;
        x.groundGrip = 1.0f;
        int why = -1;
        const float r = line.n >= 3 ? LaneControllerSteer(v, x, speed, 0.0f, nullptr, why, nullptr, &line) : NAN;
        if (std::isfinite(r)) d = r;
        else {
            abortWhy = Format("mod controller off (why %d)", why); // logged once per run in the summary
            const float ld = std::clamp(cfg.lookTime * vv, cfg.lookMin, cfg.lookMax);
            const CVector2D tp = course.p[Advance(i, ld)];
            const CVector2D rel = tp - pos;
            d = std::atan2(2.0f * L * std::sin(std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd))), ld);
        }
        lookPoint = course.p[Advance(i, 0.5f * vv)];
    }
    else if (cfg.mode == "neural") {
        d = NeuralSteer(v);
        lookPoint = course.p[Advance(i, 0.5f * vv)];
    }
    else if (cfg.mode == "neural2") {
        d = Neural2Steer(v, vv, i);
        lookPoint = course.p[Advance(i, 0.5f * vv)];
    }
    // (neural2's teachers: their rows in the trace carry what the net would have seen, and pursuit's wheel, before
    // their own command of this frame is written)
    // (only for the trace, neural2's lessons: 63 sensors a frame for ~350 cars cost frames with Trace = 0)
    if (GA::traceOn && (cfg.mode == "pursuit" || cfg.mode == "rpp" || cfg.mode == "blend")) Neural2Snapshot(v, vv, i);
    else if (cfg.mode == "mpc") {
        // Predictive (sampling MPC): 15 wheel angles around the curvature's feedforward, each one driven ahead
        // MpcHorizon s with GTA's own turn (v tan(wheel) / L at the rear axle, no slip below the grip); the one whose
        // path keeps closest to its line, with its heading at the horizon (MpcWeight) and the least change of the
        // wheel, wins
        const CVector2D rear = pos + fwd * rearAxleY;
        const int ri = NearestIndex(rear, i, 40);
        const OnCourse oc0 = Project(rear, ri);
        const float dff = std::atan(L * course.curv[Advance(oc0.i, 0.1f * vv + oc0.frac * 0.5f)]);
        const float lock = HandlingLockRad(v), span = std::clamp(3.0f / vv, 0.02f, 0.3f), dts = 0.05f;
        const int steps = std::clamp(static_cast<int>(cfg.mpcHorizon / dts), 2, 60);
        const int window = static_cast<int>(vv * dts / 0.5f) + 6;
        float bestCost = 1e30f;
        d = dff;
        for (int c = -7; c <= 7; c++) {
            const float dc = std::clamp(dff + span * c / 7.0f, -lock, lock);
            CVector2D x = rear;
            float h = carHead, cost = 0.0f;
            int hint = ri;
            OnCourse oc = oc0;
            for (int k = 0; k < steps; k++) {
                x += CVector2D(std::cos(h), std::sin(h)) * (vv * dts);
                h += vv * std::tan(dc) / L * dts;
                hint = NearestIndex(x, hint, window);
                oc = Project(x, hint);
                cost += oc.offset * oc.offset;
            }
            const float he = WrapPi(h - oc.head) * vv * 0.5f;           // (as m across, half a second on)
            cost = cost / steps + cfg.mpcWeight * he * he + 20.0f * (dc - steerOut) * (dc - steerOut);
            if (cost < bestCost) { bestCost = cost; d = dc; lookPoint = x; }
        }
    }
    else if (cfg.mode == "rpp" || cfg.mode == "blend" || cfg.mode == "gap") {
        // Three relatives of pure pursuit (the one that won so far: they start from its genes)
        const float ld0 = std::clamp(cfg.lookTime * vv, cfg.lookMin, cfg.lookMax);
        auto pursuitTo = [&](float ld) {
            const CVector2D tp = course.p[Advance(i, ld)], rel = tp - pos;
            lookPoint = tp;
            return std::atan2(2.0f * L * std::sin(std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd))), ld);
        };
        if (cfg.mode == "rpp") {
            // Regulated Pure Pursuit (ROS 2 Nav2): the look-ahead shortened where the line bends within it (RppCurv)
            float kMax = 0.0f;
            for (float a = 0.0f; a <= ld0; a += 2.0f) kMax = std::max(kMax, std::abs(course.curv[Advance(i, a)]));
            d = pursuitTo(std::max(cfg.lookMin, ld0 / (1.0f + cfg.rppCurv * kMax * ld0))) * cfg.gain; // (never under LookaheadMin: S4)
        }
        else if (cfg.mode == "blend") {
            // pursuit and the kinematic law (feedforward of the curvature, the error as a damped spring) together
            const float dp = pursuitTo(ld0) * cfg.gain;
            const CVector2D rear = pos + fwd * rearAxleY;
            const OnCourse oc = Project(rear, NearestIndex(rear, i, 40));
            const int ahead = Advance(oc.i, cfg.preview * vv + oc.frac * 0.5f);
            const float wn = cfg.bandwidth;
            const float k = course.curv[ahead] - wn * wn / (vv * vv) * oc.offset - 2.0f * cfg.damping * wn / vv * std::sin(WrapPi(carHead - oc.head));
            d = cfg.blendW * dp + (1.0f - cfg.blendW) * std::atan(L * k);
        }
        else {
            // Follow-the-Gap (F1TENTH): rays every 5 degrees over +-60, each as far as the road goes before an edge;
            // the deepest, pulled towards its own line ahead by GapBias, is where it steers (the whole width is its)
            const CVector2D tp = course.p[Advance(i, ld0)], rel = tp - pos;
            const float aLine = std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd));
            const Course& rd = R->road;
            const float half = rd.halfWidth > 0.0f ? rd.halfWidth : 2.0f;
            float bestScore = -1e9f, bestA = aLine, bestD = ld0;
            for (float deg = -60.0f; deg <= 60.0f; deg += 5.0f) {
                const float a = deg * 3.14159265f / 180.0f, ha = carHead + a;
                const CVector2D dir(std::cos(ha), std::sin(ha));
                float dist = 60.0f;
                int hint = nearIdx;
                for (float t = 2.0f; t <= 60.0f; t += 2.0f) {
                    const CVector2D pt = pos + dir * t;
                    hint = NearestIndexIn(rd, pt, hint, 10);
                    if (std::abs(ProjectIn(rd, pt, hint).offset) > half - R->bodyHalfW - 0.1f) { dist = t; break; } // (the whole body inside: half of it let the car 0.5 m out, S1 of Gemini's 8th review)
                }
                const float score = dist / 60.0f - cfg.gapBias * std::abs(a - aLine);
                if (score > bestScore) { bestScore = score; bestA = a; bestD = dist; }
            }
            const float ld = std::max(std::min(bestD, 1.5f * ld0), cfg.lookMin);
            d = std::atan2(2.0f * L * std::sin(bestA), ld);
            lookPoint = pos + CVector2D(std::cos(carHead + bestA), std::sin(carHead + bestA)) * ld;
        }
    }
    else { // pure pursuit
        const float ld = std::clamp(cfg.lookTime * vv, cfg.lookMin, cfg.lookMax);
        const CVector2D tp = course.p[Advance(i, ld)];
        const CVector2D rel = tp - pos;
        const float alpha = std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd));
        d = std::atan2(2.0f * L * std::sin(alpha), ld);
        lookPoint = tp;
    }
    // (Gain only on the pursuit-based modes: kinematic's and stanley's wheel holds the curvature's exact feedforward)
    return cfg.mode == "pursuit" || cfg.mode == "mod" ? d * cfg.gain : d;   // (neural: its own output)
}

static float OpenLoopSteer(const TestDef& d) {
    const float t = runT - d.steerStart;
    offsetNow = headErrNow = curvNow = 0.0f;
    if (d.steer == "none" || t < 0.0f) return 0.0f;
    if (d.steer == "step" || d.steer == "hold") return d.steerAmp;
    if (d.steer == "ramp") return std::min(d.steerRate * t, std::abs(d.steerAmp)) * (d.steerAmp < 0.0f ? -1.0f : 1.0f);
    if (d.steer == "sine") return d.steerAmp * std::sin(2.0f * 3.14159265f * d.freq * t);
    if (d.steer == "sweep") {
        // frequency rising linearly from Freq to FreqEnd over the run
        const float T = std::max(d.duration - d.steerStart, 1.0f);
        const float ph = 2.0f * 3.14159265f * (d.freq * t + 0.5f * (d.freqEnd - d.freq) * t * t / T);
        return d.steerAmp * std::sin(ph);
    }
    return 0.0f;
}

static void Control(CVehicle* v) {
    const TestDef& d = *curTest;
    const float dt = std::max(CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS, 0.001f);
    const float speed = CVector::Dot(v->m_vecMoveSpeed, v->GetForward()) * GAME_SPEED_TO_MPS;
    const bool openLoop = d.steer != "none";
    // the search around the last point no wider than the car can go (the spiral's tight loops are 7-11 m apart and ~44 m
    // of course apart: with +-60 m a car off the line snapped onto the next loop and went round and round)
    nearIdx = NearestIndex(v->GetPosition().To2D(), nearIdx, std::clamp(static_cast<int>(std::abs(speed) * 0.5f / 0.5f), 30, 120));
    if (runT < countdownS) {
        // the countdown (a human drives car A): everyone waits on the brakes
        v->m_fSteerAngle = 0.0f;
        v->m_fGasPedal = 0.0f;
        v->m_fBreakPedal = 1.0f;
        gasOut = steerOut = targetKmh = 0.0f;
        brakeOut = 1.0f;
        return;
    }
    if (openLoop) {
        // offset from the drawn line, for the record only
        ClosedLoopSteer(v, speed);
    }
    float raw = openLoop ? OpenLoopSteer(d) : ClosedLoopSteer(v, speed);
    steerCmd = raw;
    // limits
    float lim = cfg.maxSteer > 0.0f ? cfg.maxSteer : HandlingLockRad(v);
    if (cfg.gripClampG > 0.0f && speed > 5.0f && !openLoop)
        lim = std::min(lim, std::atan(wheelBase * cfg.gripClampG * 9.81f / (speed * speed)));
    raw = std::clamp(raw, -lim, lim);
    if (cfg.steerRate > 0.0f && !openLoop) raw = std::clamp(raw, steerOut - cfg.steerRate * dt, steerOut + cfg.steerRate * dt);
    if (cfg.smoothing > 0.0f && !openLoop) raw = steerOut + (raw - steerOut) * std::min(dt / cfg.smoothing, 1.0f);
    steerOut = raw;
    float applied = raw;
    playerSteerFactor = 1.0f;
    if (v->m_nStatus == STATUS_PLAYER) {
        const float vf = CVector::Dot(v->m_vecMoveSpeed, v->GetForward());
        if (vf > 0.01f) {
            CColPoint cp{};
            cp.m_nSurfaceTypeA = SURFACE_WHEELBASE;
            cp.m_nSurfaceTypeB = SURFACE_TARMAC;
            const float adhesive = reinterpret_cast<SurfaceInfos_c*>(0xB79538)->GetAdhesiveLimit(&cp);
            const float traction = v->m_pHandlingData->m_fTractionMultiplier / 250.0f * 0.25f;
            const float a = std::min(adhesive * traction * 16.0f / (vf * vf), 1.0f);
            playerSteerFactor = std::min(std::asin(a) / HandlingLockRad(v), 1.0f);
            const float vr = CVector::Dot(v->m_vecMoveSpeed, v->GetRight());
            const bool counter = (raw < 0.0f && vr > 0.05f) || (raw > 0.0f && vr < -0.05f);
            if (!counter && playerSteerFactor > 0.01f)
                applied = std::clamp(raw / playerSteerFactor, -HandlingLockRad(v), HandlingLockRad(v));
        }
    }

    // pedals
    float vt = d.speeds[speedIdx] / 3.6f;
    const float gripG = d.gripG > 0.0f ? d.gripG : (d.gripG == 0.0f ? cfg.gripFactor * aiGripG : cfg.speedGripG), brakeG = d.brakeG > 0.0f ? d.brakeG : cfg.speedBrakeG;
    // (neural2's profile once the car is made: the road can be built before, with no grip known yet)
    // (once a run: an empty profile was worked out again every frame, 11,000 points x 500 cars: 12.7 ms a frame)
    if (cfg.mode == "neural2" && R->profile.empty() && !R->profileTried && aiGripG > 0.0f && !openLoop) { R->profileTried = true; SpeedProfile(*curTest); }
    if (gripG > 0.0f && !openLoop && R->profile.empty()) {
        // slower for the curves ahead: v2 = G g / k there, braking at BrakeG g to reach it
        // (up to 2 km: with no top speed set, 450+ km/h needs 700+ m to brake for the U-turn)
        const float reach = std::clamp(speed * speed / (2.0f * brakeG * 9.81f) + 60.0f, 60.0f, 2000.0f);
        for (float a = 0.0f; a < reach; a += 2.0f) {
            const int k = Advance(nearIdx, a);
            if (!course.closed && k >= static_cast<int>(course.p.size()) - 1) break;
            if (course.vcap[k] < 1e8f) vt = std::min(vt, std::sqrt(course.vcap[k] * course.vcap[k] + 2.0f * brakeG * 9.81f * a));
            const float c = std::abs(course.curv[k]);
            if (c < 1e-4f) continue;
            // no speed per section: each curve's from its radius, the grip for that kind of curve and what the car
            // learned of it so far
            const float vc = std::sqrt(gripG * CurveGrip(c) * R->learned[CurveBand(c)] * 9.81f / c);
            vt = std::min(vt, std::sqrt(vc * vc + 2.0f * brakeG * 9.81f * a));
        }
    }
    // mode gt4: its profile (a little ahead: the speed it has to be at in 0.3 s)
    // (neural2: x its aggressiveness, e^(0.25 out), 0.78..1.28: it braked early and came into the curves slow, the user)
    if (!openLoop && !R->profile.empty())
        vt = std::min(vt, R->profile[Advance(nearIdx, std::max(speed, 5.0f) * 0.3f)] * (cfg.mode == "neural2" ? std::exp(0.25f * R->nnAgg) : 1.0f));
    // a sudden brake: only once the car is in it
    if (!openLoop && !course.vsud.empty()) vt = std::min(vt, course.vsud[nearIdx]);
    targetKmh = vt * 3.6f;
    R->absOn = R->tcOn = false;
    R->windNow = 0.0f;
    if (!openLoop && !course.wind.empty() && course.wind[nearIdx] != 0.0f) {
        // crosswind: the car pushed sideways (across the course) before its physics frame
        R->windNow = course.wind[nearIdx];
        const float h = course.head[nearIdx];
        v->m_vecMoveSpeed += CVector(-std::sin(h), std::cos(h), 0.0f) * (R->windNow * dt / GAME_SPEED_TO_MPS);
    }
    float gas = 0.0f, brake = 0.0f;
    bool hb = false;
    const float tp = runT - d.pedalStart;
    if (d.pedal == "full" && tp >= 0.0f) gas = 1.0f;
    else if (d.pedal == "coast" && tp >= 0.0f) gas = 0.0f;
    else if (d.pedal == "brake" && tp >= 0.0f && !brakeDone && !(d.brakeTo > 0.0f && speed * 3.6f <= d.brakeTo)) brake = d.brakeLevel;
    else if (d.pedal == "lift" && tp >= 0.0f) gas = 0.0f;
    else {
        if (d.pedal == "brake" && tp >= 0.0f && d.brakeTo > 0.0f) { brakeDone = true; vt = d.brakeTo / 3.6f; } // then held there
        targetKmh = vt * 3.6f;
        const float e = vt - speed;
        if (e > -0.5f) gas = std::clamp(cfg.cruiseGas + cfg.speedKp * e, 0.0f, 1.0f);
        if (e < -1.0f) brake = std::clamp(cfg.brakeKp * (-e - 0.5f), 0.0f, 1.0f);
    }
    // mode neural: its own pedal (the speed plan is not its)
    R->teacherPedal = gas - brake;
    if (!openLoop && cfg.mode == "neural") {
        gas = std::max(R->nnPedal, 0.0f);
        brake = std::max(-R->nnPedal, 0.0f);
        // (below 18 km/h a launch: its teachers had nothing slower, and it never left the start)
        if (speed < 5.0f) { gas = std::max(gas, 0.8f); brake = 0.0f; }
    }
    // mode neural2: the planner's speed x (1 + kNn2SpeedSpan x its second output), through the same controller (then
    // the ABS and the TC): the pedal itself was its output before, and a P-controller's output (CruiseGas + Kp x the
    // speed error, each car's genes) never fit (R2 0.75, falling) nor launched. targetKmh stays the planner's: the
    // trace logs it and the net reads it (n51)
    // (now its second output is on the pedal itself: up to half of it more gas or more brake over the teacher's; the
    // planner's speed for the curves ahead only gives the teacher's pedal: the user, "a velocidade baseada na
    // distancia da curva nao faz muito sentido")
    if (!openLoop && cfg.mode == "neural2" && !cfg.nn.empty() && d.pedal == "hold") {
        const float e = vt - speed;
        const float g0 = e > -0.5f ? std::clamp(cfg.cruiseGas + cfg.speedKp * e, 0.0f, 1.0f) : 0.0f;
        const float b0 = e < -1.0f ? std::clamp(cfg.brakeKp * (-e - 0.5f), 0.0f, 1.0f) : 0.0f;
        const float pedal = std::clamp(g0 - b0 + kNn2SpeedSpan * std::clamp(R->nnPedal, -1.0f, 1.0f), -1.0f, 1.0f);
        gas = std::max(pedal, 0.0f);
        brake = std::max(-pedal, 0.0f);
    }
    const float brakeAsked = brake;
    if (!openLoop && cfg.abs && brake > 0.0f) {
        // No ABS in GTA: a wheel whose brake (brakeDecel x pedal x 2 x bias) passes its adhesion (adhesive x tractionMult
        // / 1000 x 2 x traction bias) locks at once (ProcessWheel: WHEEL_STATE_FIXED) and loses its sideways grip; the
        // Infernus locked its front wheels from ~70% pedal and ran wide in every braked turn. The pedal is held under
        // that point, and in a curve under what the curve leaves of the grip (friction circle)
        const tHandlingData* h = v->m_pHandlingData;
        const float adh = aiAdhesive * h->m_fTractionMultiplier / 1000.0f;
        const float bf = 2.0f * h->m_fBrakeBias, br = 2.0f - bf;
        const float tf = 2.0f * h->m_fTractionBias, tr = 2.0f - tf;
        const float lockF = bf > 0.01f ? adh * tf / (h->m_fBrakeDeceleration * bf) : 1.0f;
        const float lockR = br > 0.01f ? adh * tr / (h->m_fBrakeDeceleration * br) : 1.0f;
        // (the numbers are the car's genes: AbsMargin, AbsCircle, AbsDrop, AbsRecover)
        float pedalMax = cfg.absMargin * std::min(lockF, lockR);
        const float G = aiGripG * 9.81f;
        const float aLat = std::abs(speed * v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS);
        const float circle = std::sqrt(std::max(1.0f - (aLat / G) * (aLat / G), 0.04f));
        pedalMax *= 1.0f - cfg.absCircle * (1.0f - circle);
        // a wheel locked anyway: off a little more until it turns again
        CAutomobile* a = static_cast<CAutomobile*>(v);
        bool locked = false;
        for (int w = 0; w < 4; w++) if (a->m_wheelState[w] == 3) locked = true;
        absScale = locked ? std::max(absScale - cfg.absDrop, 0.3f) : std::min(absScale + cfg.absRecover, 1.0f);
        // (x what the car learned of its ABS: LearnByResult)
        const float allowed = pedalMax * absScale * R->absLearn * (cfg.mode == "neural2" ? std::exp(0.5f * R->nnAbs) : 1.0f);
        R->absOn = brake > allowed;
        brake = std::min(brake, allowed);
    }
    // (a direct TC for neural2, the rear slip past its target cutting the gas, came too late at 400 km/h: 23 of 500
    // home in g118, spins in the fast straight; the TC that keeps the gas to the grip the curve leaves is back, the
    // net still setting how much it takes, tcNet)
    if (!openLoop && cfg.tractionControl) {
        // The tyres share one grip between turning and driving (ProcessWheel: the thrust and the sideways pull are scaled
        // down together past the adhesion): gas only with the grip the curve leaves, none while sliding
        const float G = (gripG > 0.0f ? gripG : cfg.gripFactor * aiGripG) * 9.81f;
        const float aLat = std::abs(speed * v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS);
        // (the numbers are the car's genes: TcCircle, TcMinGas, TcSlip, TcDrop, TcRecover)
        const float room = 1.0f - cfg.tcCircle * (1.0f - std::sqrt(std::max(G * G - aLat * aLat, 0.0f)) / G);
        const float gasAsked = gas;
        // (x the gas the car learned its tyres take in this kind of curve: LearnByResult)
        const float cNow = std::abs(course.curv[nearIdx]);
        const float tcNet = cfg.mode == "neural2" ? std::exp(0.7f * R->nnTc) : 1.0f; // (> 1: it lets more slip and gas)
        gas = std::min(gas, std::clamp(room * tcNet * (cNow > 1.0f / 400.0f ? R->tcBand[CurveBand(cNow)] : 1.0f), cfg.tcMinGas, 1.0f));
        // sliding: at the rear axle (at the car's centre a tight turn at full lock reads ~0.25 rad with the tyres holding:
        // the gas went off and the car stopped in the 7 m U-turn), and not at walking pace
        const float vRightRear = CVector::Dot(v->m_vecMoveSpeed, v->GetRight()) - v->m_vecTurnSpeed.z * rearAxleY;
        const float slip = std::abs(std::atan2(vRightRear, std::max(std::abs(CVector::Dot(v->m_vecMoveSpeed, v->GetForward())), 0.02f)));
        const bool sliding = slip > cfg.tcSlip * tcNet && speed > 3.0f;
        if (sliding) gas = 0.0f;
        // wheels spinning (4WD out of a tight turn: all four at full gas): the gas comes off until they grip again
        CAutomobile* a = static_cast<CAutomobile*>(v);
        bool spinning = false;
        for (int w = 0; w < 4; w++) if (a->m_wheelState[w] == 1) spinning = true;
        tcScale = spinning ? std::max(tcScale - cfg.tcDrop, 0.2f) : std::min(tcScale + cfg.tcRecover, 1.0f);
        gas *= tcScale;
        R->tcOn = gas < gasAsked - 0.05f;
    }
    // (gt4 does not learn on the way: the same lap every time)
    // (not neural2: the speed multipliers by kind of curve changing in the run, the user did not like them; its net's
    // speed output does that)
    if (!openLoop && cfg.learnRate > 0.0f && speed > 5.0f && R->profile.empty() && cfg.mode != "neural" && cfg.mode != "neural2")
        LearnByResult(speed, dt, brakeAsked);
    if (d.handbrakeAt && tp >= 0.0f) hb = true;
    // (pulled a share of the frames, its output's (0..1): past 0.5 it was all or nothing, and the net's outputs start
    // at 0 and move by tenths, so it never pulled it and could not learn what it does)
    // (only under 80 km/h, the hairpins', and the share the square of the output: at the start of the learning +-0.1
    // pulled it 1-10% of the frames at 400 km/h and 426 of 500 spun, g149)
    if (cfg.mode == "neural2" && speed > 5.0f && speed < 22.0f && R->nnHb > 0.0f &&
        static_cast<float>((CTimer::m_FrameCounter + R->index) % 20) < 20.0f * R->nnHb * R->nnHb) hb = true;
    gasOut = gas;
    brakeOut = brake;
    handbrakeOut = hb;
    v->m_fSteerAngle = applied;
    v->m_fGasPedal = gas;
    v->m_fBreakPedal = brake;
    v->bIsHandbrakeOn = hb;
    if (cfg.carTraction > 0.0f) *reinterpret_cast<float*>(reinterpret_cast<char*>(v) + 0x8A0) = cfg.carTraction;
}

// CAutomobile::ProcessAI -> CCarCtrl::SteerAICarWithPhysics (0x6B52A8)
static unsigned int steerAiOriginal = 0x437C20;
static void __cdecl SteerAiHook(CVehicle* v) {
    if (enabled && state == RUNNING && curTest) for (int k = 0; k < numRunners; k++) if (v == CarOf(k)) {
        R = &runners[k];
        if (R->finished) { v->m_fSteerAngle = 0.0f; v->m_fGasPedal = 0.0f; v->m_fBreakPedal = 1.0f; return; }
        if (k == 0 && (cfg.driver == "player" || cfg.driver == "human")) break; // (the player's car: after its inputs / the human)
        Perf::Scope perf(Perf::CONTROL);
        Control(v); // ai, cjai, and car B
        return;
    }
    reinterpret_cast<void(__cdecl*)(CVehicle*)>(steerAiOriginal)(v);
    if (cfgAiAbs) ApplyAiAbs(v, v->m_fBreakPedal);
}
// CAutomobile::ProcessControlInputs (virtual): the pad first, then ours
static unsigned int inputsOriginal = 0x6AD690;
static void __fastcall InputsHook(CVehicle* v, void*, unsigned char pad) {
    reinterpret_cast<void(__thiscall*)(CVehicle*, unsigned char)>(inputsOriginal)(v, pad);
    if (enabled && state == RUNNING && curTest && v == CarOf(0) && CfgOf(0).driver == "player" && !runners[0].finished) {
        R = &runners[0];
        Control(v);
    }
}

static void InstallHooks() {
    const unsigned int site = 0x6B52A8;
    if (*reinterpret_cast<unsigned char*>(site) == 0xE8) {
        steerAiOriginal = site + 5 + *reinterpret_cast<int*>(site + 1);
        patch::RedirectCall(site, SteerAiHook, true);
    }
    // the vtable slots holding CAutomobile::ProcessControlInputs (CAutomobile's and its subclasses' that keep it)
    int slots = 0;
    for (unsigned int a = 0x858000; a < 0x8A0000; a += 4) {
        if (*reinterpret_cast<unsigned int*>(a) == 0x6AD690) {
            patch::SetUInt(a, reinterpret_cast<unsigned int>(&InputsHook), true);
            slots++;
        }
    }
    lg << "Proving ground: SteerAICarWithPhysics call -> 0x" << std::hex << steerAiOriginal << std::dec << ", "
       << slots << " vtable slots of ProcessControlInputs\n";
}

// --- Recording -------------------------------------------------------------------------------------------------------

static bool writeCsv = true;   // the GA's runs: [GA] Csv (a 5 km run of 10 cars writes ~10 MB)
static float offLimitGA = 2.0f; // m off the course that counts as out (stats.outTime)
static void StartCsv() {
    if (csv.is_open()) csv.close();
    if (!writeCsv) return;
    const std::string name = Format("%02d_%s_%d_%.0fkmh%s.csv", runCount, curTest->name.c_str(), curModel, curTest->speeds[speedIdx],
        numRunners > 1 ? (std::string("_") + R->tag).c_str() : "");
    csv.open(runDir + name, std::ios::trunc);
    csv << "t,dt,x,y,z,heading,speed_kmh,v_fwd,v_lat,yaw_rate,a_long,a_lat,steer_cmd,steer,gas,brake,handbrake,gear,"
           "offset,head_err,course_curv,target_kmh,ws_fl,ws_rl,ws_fr,ws_rr,susp_fl,susp_rl,susp_fr,susp_rr,contact,s,surface,player_factor,status\n";
}

static void Record(CVehicle* v, float dt) {
    const CVector vel = v->m_vecMoveSpeed * GAME_SPEED_TO_MPS;
    const CVector fwd = v->GetForward(), right = v->GetRight();
    float aLong = 0.0f, aLat = 0.0f;
    if (havePrevVel && dt > 0.0005f) {
        const CVector acc = (vel - prevVel) / dt;
        aLong = CVector::Dot(acc, fwd);
        aLat = -CVector::Dot(acc, right); // > 0: to the left
    }
    prevVel = vel;
    havePrevVel = true;
    const float vF = CVector::Dot(vel, fwd), vL = -CVector::Dot(vel, right);
    const float yaw = v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS;
    const float speed = vel.Magnitude();
    CAutomobile* a = v->m_nVehicleSubClass == VEHICLE_AUTOMOBILE || v->m_nVehicleSubClass == VEHICLE_MTRUCK || v->m_nVehicleSubClass == VEHICLE_QUAD
        ? static_cast<CAutomobile*>(v) : nullptr;
    int ws[4] = {};
    float su[4] = { 1, 1, 1, 1 };
    int contact = 0, surface = -1;
    if (a) surface = a->m_wheelColPoint[0].m_nSurfaceTypeB;
    if (a) for (int w = 0; w < 4; w++) {
        ws[w] = static_cast<int>(a->m_wheelState[w]);
        su[w] = a->m_fWheelsSuspensionCompressionPrev[w];
        if (su[w] < 1.0f) contact++;
    }
    const CVector p = v->GetPosition();
    R->latGNow = aLat / 9.81f;
    R->aLongNow = aLong;
    R->slipNow = std::atan2(vL, std::max(std::abs(vF), 0.5f));
    R->speedNow = speed * 3.6f;
    R->skidFNow = ws[0] || ws[2];
    R->skidRNow = ws[1] || ws[3];
    if (!course.s.empty()) R->sMax = std::max(R->sMax, course.s[nearIdx]);
    if (R->sMax > R->progressS + 5.0f) { R->progressS = R->sMax; R->progressT = runT; }
    // the sectors: the time it reached each part's start (the report: where the time is lost)
    if (R->secT.size() != course.sections.size()) R->secT.assign(course.sections.size(), -1.0f);
    for (size_t q = 0; q < course.sections.size(); q++)
        // (the parts' starts are the road's metres, its progress its own line's, a little shorter: by the share of the lap)
        if (R->secT[q] < 0.0f && R->sMax >= course.sections[q].first * course.length / std::max(R->road.length, 1.0f)) R->secT[q] = q == 0 ? 0.0f : runT;
    if (course.halfWidth > 0.0f && !course.p.empty()) {
        // The road is a corridor between two edges: what has to stay inside is the whole body, not its centre. Each of
        // its four corners on the course (in a curve the body sweeps wider than its centre's path, and the room left
        // shrinks); the room = the edge's distance from the corner nearest to it
        // (F1's track limits, the user: a wheel or three outside is fine, all four outside is not. The inner edge of
        // each tyre on the road; the room = how far the one nearest the middle is from the edge it is going over;
        // < 0: all four past the same edge. It was the body's corner nearest an edge before)
        const CVector2D c0 = v->GetPosition().To2D(), f = v->GetForward().To2D(), rt(f.y, -f.x);
        const float inner = std::max(R->bodyHalfW - 0.4f, 0.3f);
        float lo = 1e9f, hi = -1e9f;
        for (float fy : { rearAxleY + wheelBase, rearAxleY })
            for (float sx : { -1.0f, 1.0f }) {
                const CVector2D tyre = c0 + f * fy + rt * (sx * inner);
                const OnCourse oc = ProjectIn(R->road, tyre, NearestIndexIn(R->road, tyre, nearIdx, 16));
                lo = std::min(lo, oc.offset);
                hi = std::max(hi, oc.offset);
            }
        R->marginNow = lo > 0.0f ? course.halfWidth - lo : hi < 0.0f ? course.halfWidth + hi : course.halfWidth;
        if (runT > 2.0f) {
            stats.deficit += std::max(edgeComfortGA - R->marginNow, 0.0f) * dt;
            stats.minMargin = std::min<double>(stats.minMargin, R->marginNow);
            if (R->marginNow < 0.0f) stats.outTime += dt;
        }
    }
    else if (std::abs(offsetNow) > offLimitGA) stats.outTime += dt;
    if (csv.is_open()) csv << Format("%.3f,%.4f,%.2f,%.2f,%.2f,%.4f,%.2f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%d,%d,%.3f,%.4f,%.5f,%.1f,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%d,%.4f,%d\n",
        runT, dt, p.x, p.y, p.z, std::atan2(fwd.y, fwd.x), speed * 3.6f, vF, vL, yaw, aLong, aLat, steerCmd, v->m_fSteerAngle, gasOut, brakeOut,
        handbrakeOut ? 1 : 0, static_cast<int>(v->m_nCurrentGear), offsetNow, headErrNow, curvNow, targetKmh,
        ws[0], ws[1], ws[2], ws[3], su[0], su[1], su[2], su[3], contact, course.s.empty() ? 0.0f : course.s[nearIdx], surface, playerSteerFactor, static_cast<int>(v->m_nStatus));

    // stats (after the first 2 s: the flying start settles)
    if (runT > 2.0f) {
        stats.n++;
        stats.time += dt;
        stats.absE += std::abs(offsetNow) * dt;
        stats.sqE += offsetNow * offsetNow * dt;
        stats.maxE = std::max<double>(stats.maxE, std::abs(offsetNow));
        stats.speedErr += (speed * 3.6f - targetKmh) * dt;
        stats.maxLatG = std::max<double>(stats.maxLatG, std::abs(aLat) / 9.81f);
        stats.steerAbs += std::abs(v->m_fSteerAngle) * dt;
        const float dSteer = v->m_fSteerAngle - stats.lastSteerDelta;
        stats.lastSteerDelta = v->m_fSteerAngle;
        if (std::abs(dSteer) > 0.002f) {
            const float sgn = dSteer > 0.0f ? 1.0f : -1.0f;
            if (stats.lastSign != 0.0f && sgn != stats.lastSign) stats.reversals++;
            stats.lastSign = sgn;
        }
        if (ws[0] || ws[2]) stats.skidFront += dt;
        if (ws[1] || ws[3]) stats.skidRear += dt;
        if (std::abs(v->m_fSteerAngle) > 0.01f && speed > 5.0f && !ws[0] && !ws[1] && !ws[2] && !ws[3]) {
            stats.gainSum += yaw / (speed * std::tan(v->m_fSteerAngle));
            stats.gainN++;
        }
    }
    stats.maxSpeed = std::max(stats.maxSpeed, speed * 3.6f);
    if (stats.t100 == 0.0f && speed * 3.6f >= 100.0f && curTest->pedal == "full") stats.t100 = runT - curTest->pedalStart;
}

static void EndRun(const char* why) {
    if (csv.is_open()) csv.close();
    const double T = std::max(stats.time, 0.001);
    std::string line = Format("[%02d%s] %-12s model %3d %4.0f km/h  course %4.0f m  %-9s  offset mean %.2f rms %.2f max %.2f m | speed err %+.1f km/h max %.0f | "
        "lat g max %.2f | steer mean %.3f rad, reversals %.1f/s | skid front %.0f%% rear %.0f%% | 1/L %.3f (L %.2f, model %.2f)",
        runCount, numRunners > 1 ? R->tag : "", curTest->name.c_str(), curModel, curTest->speeds[speedIdx], course.length, why, stats.absE / T, std::sqrt(stats.sqE / T), stats.maxE,
        stats.speedErr / T, stats.maxSpeed, stats.maxLatG, stats.steerAbs / T, stats.reversals / T, 100.0 * stats.skidFront / T,
        100.0 * stats.skidRear / T, stats.gainN ? stats.gainSum / stats.gainN : 0.0, stats.gainN ? stats.gainN / stats.gainSum : 0.0, wheelBase);
    if (stats.t100 > 0.0f) line += Format(" | 0-100 %.2f s", stats.t100);
    if (curTest->pedal == "brake") line += Format(" | stop %.1f m", stats.stopDist);
    line += Format(" | %.1f s, out %.1f s, reached %.0f m", runT, stats.outTime, R->sMax);
    if (!abortWhy.empty()) line += " | " + abortWhy;
    Log(line);
    R->result = why;
    R->endTime = runT;
    if (GA::on) GA::Score(R->index, why, runT, stats.absE / T, std::sqrt(stats.sqE / T), stats.maxE, stats.outTime, R->sMax, course.length,
        stats.deficit, stats.minMargin);
    if (GA::on) {
        // each part's time (-1: not driven through)
        std::vector<float> sec;
        const bool home = !strcmp(why, "end");
        sec.assign(R->secT.size(), -1.0f);
        for (size_t q = 0; q < R->secT.size(); q++) {
            const float a = R->secT[q], b = q + 1 < R->secT.size() ? R->secT[q + 1] : (home ? runT : -1.0f);
            if (a >= 0.0f && b >= 0.0f) sec[q] = b - a;
        }
        GA::SetSectors(R->index, sec);
    }
}

// --- Genetic algorithm -----------------------------------------------------------------------------------------------
// [GA] Enabled = 1: every car on the course is one individual of a population that drives the tests at the same time; a
// generation is one pass over the tests. The genome is the steering algorithm (Mode) and the controller's numbers (the
// genes below, inside their ranges). After each generation the next one is bred from the best: the Elite best go on
// unchanged (they drive again: GTA's physics is not exactly repeatable), the rest are children of two parents picked by
// tournament, each gene from either parent (uniform crossover), then moved by a Gaussian of Sigma x its range with the
// MutationRate's chance (the mode with ModeMutation's), and Immigrants new random ones keep the population varied.
// Score (lower is better): a car that reaches the end: time + OffPenalty x s further than OffLimit from the course +
// RmsPenalty x its rms offset; one that does not: 1000 + 0.5 per m it did not reach + OffPenalty x s out.
// Files (scripts\proving): ga_state.txt (the population: a restart or an ini change goes on from it while the genes are
// the same; Resume = 0 starts over), ga_history.csv (every individual of every generation), ga_best.ini (the best ever
// as [Steer] / [Speed] sections).
namespace GA {
// (uses: the modes that read the gene, empty = all: only those genes mutate)
struct GeneDef { const char* sec; const char* key; float Settings::* field; float lo, hi; const char* uses; };
static const GeneDef kGenes[] = {
    { "Steer", "Bandwidth", &Settings::bandwidth, 0.5f, 4.0f, "kinematic gt4 blend" },
    { "Steer", "Damping", &Settings::damping, 0.4f, 2.0f, "kinematic gt4 blend" },
    { "Steer", "Preview", &Settings::preview, 0.0f, 0.3f, "kinematic stanley gt4 blend" },
    // (LookaheadTime and Gain from 0.2 and 0.7: generations 2-4 converged on those lower ends)
    // (neural2 too, its teacher is pursuit: frozen at the old brains' 0.003 s it aimed 4.3 m ahead at 450 km/h and spun
    // in the crosswind, g149-155; at least 0.3 s, 37 m there)
    { "Steer", "LookaheadTime", &Settings::lookTime, 0.3f, 1.5f, "pursuit mod rpp blend gap neural2" }, // (mod: its fallback)
    { "Steer", "LookaheadMin", &Settings::lookMin, 1.5f, 15.0f, "pursuit mod rpp blend gap neural2" }, // (from 3: 3.2 m strayed 3x less than 5.3 m in the U-turn)
    { "Steer", "StanleyK", &Settings::stanleyK, 0.3f, 5.0f, "stanley" },
    { "Steer", "Gain", &Settings::gain, 0.5f, 1.3f, "pursuit mod rpp blend" },
    { "Steer", "Smoothing", &Settings::smoothing, 0.0f, 0.12f, "" },
    // (0.55 while the teacher aimed 4.3 m ahead and the gusts threw it out; 0.70 with those fixed: the GA goes as close
    // to the grip as pays, the user's item 5)
    { "Speed", "GripFactor", &Settings::gripFactor, 0.3f, 0.70f, "" }, // curve speeds: share of the AI grip formula
    { "Speed", "BrakeG", &Settings::speedBrakeG, 0.5f, 1.6f, "" },    // braking planned for curves and limits
    { "Speed", "Kp", &Settings::speedKp, 0.1f, 1.0f, "" },
    { "Speed", "BrakeKp", &Settings::brakeKp, 0.2f, 2.5f, "" },   // (to 2.5: 10 of 40 at 1.5, the 1st among them)
    { "Steer", "LineBias", &Settings::lineBias, -1.0f, 1.0f, "kinematic stanley pursuit mod rpp blend gap gt4 mpc" },  // its own line: to one side of the road
    { "Steer", "LineCut", &Settings::lineCut, -1.0f, 1.0f, "kinematic stanley pursuit mod rpp blend gap gt4 mpc" },    // ... into the inside of the curves (< 0: wide)
    { "Steer", "LineRace", &Settings::lineRace, 0.0f, 1.0f, "kinematic stanley pursuit mod rpp blend gap gt4 mpc" },   // ... the racing line's share (1: all of it)
    { "Steer", "MpcHorizon", &Settings::mpcHorizon, 0.3f, 2.0f, "mpc" }, // mpc: how far ahead it simulates (s)
    { "Steer", "MpcWeight", &Settings::mpcWeight, 0.0f, 5.0f, "mpc" },   // ... the weight of the heading there
    { "Speed", "GtAccel", &Settings::gtAccel, 0.2f, 1.2f, "gt4 neural2" },
    { "Steer", "CounterSteer", &Settings::counterSteer, 0.0f, 2.0f, "neural2" },  // neural2: countersteer and yaw damping      // gt4: its profile's acceleration (g); neural2: x the grip (<= 1)
    { "Steer", "RppCurv", &Settings::rppCurv, 0.0f, 10.0f, "rpp" },     // rpp: a bend ahead shortens the look-ahead
    { "Steer", "BlendW", &Settings::blendW, 0.0f, 1.0f, "blend" },      // blend: the share of pursuit
    { "Steer", "GapBias", &Settings::gapBias, 0.0f, 3.0f, "gap" },      // gap: the pull of its own line
    { "Speed", "GripTight", &Settings::gripTight, 0.35f, 1.6f, "" }, // the grip in tight curves (<= 15 m) x this (from 0.35: 24 of 60 at 0.6)
    { "Speed", "GripFast", &Settings::gripFast, 0.6f, 1.6f, "" },   // ... in fast ones (>= 150 m)
    { "Speed", "LearnRate", &Settings::learnRate, 0.0f, 0.4f, "kinematic stanley pursuit mod rpp blend gap gt4 mpc" }, // how fast it learns curves, ABS and TC itself
    { "Speed", "CruiseGas", &Settings::cruiseGas, 0.0f, 0.4f, "" },  // the speed controller's base gas
    { "Speed", "AbsMargin", &Settings::absMargin, 0.5f, 1.2f, "" },  // ABS: share of the lock point
    { "Speed", "AbsDrop", &Settings::absDrop, 0.02f, 0.4f, "" },
    { "Speed", "AbsRecover", &Settings::absRecover, 0.01f, 0.2f, "" },
    { "Speed", "AbsCircle", &Settings::absCircle, 0.0f, 1.0f, "" },
    // (up to no TC at all: driving sideways may be faster, and the score decides)
    { "Speed", "TcSlip", &Settings::tcSlip, 0.03f, 1.0f, "" },       // TC: slip (rad) before the gas goes
    { "Speed", "TcMinGas", &Settings::tcMinGas, 0.0f, 1.0f, "" },
    { "Speed", "TcDrop", &Settings::tcDrop, 0.0f, 0.3f, "" },
    { "Speed", "TcRecover", &Settings::tcRecover, 0.01f, 0.1f, "" },
    { "Speed", "TcCircle", &Settings::tcCircle, 0.0f, 1.0f, "" },
};
constexpr int kNumGenes = sizeof(kGenes) / sizeof(kGenes[0]);
// label: its pilot profile (elites keep it); sum / runs: its scores so far (an elite drives again every generation: GTA's
// physics gave the same genome 156.8 and 158.2, more than between 1st and 3rd; its average is what ranks it)
struct Genome { int mode = 0; float g[kNumGenes] = {}; std::string label; double sum = 0.0; int runs = 0; float nn[kNN2] = {};
                int archiveId = -1; }; // archiveId: steady state, the archive entry it is (a re-run of an elite)
struct Result { double score = 0.0, time = 0.0, mean = 0.0, rms = 0.0, maxE = 0.0, out = 0.0, reached = 0.0, deficit = 0.0, minMargin = 1e9; std::string why; int runs = 0;
                std::vector<float> sectors; }; // (each part's time, s; -1: not driven through)

static int elite = 2, tournament = 3, immigrants = 1, maxGenerations = 0;
static int minPerMode = 0; // islands: every algorithm at least this many cars, bred among its own
static bool steady = false;       // SteadyState: no generations to wait for (Replace)
static float reevalChance = 0.1f; // ... the share of new runs that are an elite again (its average against the noise)
static float bestLapTime = -1.0f; // ... the best time home so far (the cutoff's reference)
static int runIds = 0;            // ... runs started
static float nnSigma = 0.05f;  // mode neural: the mutation of a weight
// neural2's evolution strategies ([GA] Nn2*: see AssignNn2 / StepTheta)
static bool nn2Es = false;
static float nn2Sigma = 0.02f, nn2Alpha = 0.03f, nn2Noise = 0.01f;
static int nn2Cars = 20;
static int nn2Min = 0;   // Nn2Min: at least this many neural2 brains in every generation (the user: 25, so it evolves)
static float mutRate = 0.3f, sigma0 = 0.15f, sigmaDecay = 0.96f, sigmaMin = 0.03f, modeMutation = 0.1f;
static float meanPenalty = 100.0f, offPenalty = 5.0f, rmsPenalty = 0.0f, edgePenalty = 1.0f;
// DevWeight: the time made longer by the mean deviation from the car's own line, in proportion (0.3: 0.1 m -> +3%): the
// user, "o desvio deve ser levado em consideracao sempre, de maneira proporcional"; the line itself is free (any line in the lane)
static float devWeight = 0.3f, loadedDev = -1.0f;
// [GA] ScoreVersion: raised when the course changes (its times are not comparable): the record, history and report
// start over (the brains are kept)
static int scoreVersion = 1, loadedVer = 1;
// steady state: parents from the best ParentTop of their algorithm's archive; sigma wider when stuck (SigmaMax)
static float parentTop = 0.2f, sigmaMax = 0.15f;
static int stall = 0;
static double stallRef = 1e18;
static std::vector<double> history10; // steady state: each generation's mean of its 10 best runs
// [GA] Trace = 0: no trace files (only the analysis tools read them: the viewer, Gemini, nn2_train); TraceEvery: s
// between a car's rows (0: 0.25, or 0.5 above 100 cars)
static float traceEvery = 0.0f;
// CutoffFactor: once the first car is home, a car still running at that time x this is stopped (CUT) and scored by
// the time it was heading for (its time x the course / how far it got) + CutPenalty (0: no cutoff)
static float cutoffFactor = 1.3f, cutPenalty = 20.0f;
static bool resume = true;
static unsigned int seed = 0;
static std::vector<std::string> modes{ "kinematic", "stanley", "pursuit", "mod" };
static Settings base;
static Genome pop[kMaxRunners];
static Result res[kMaxRunners];
// steady state: the archive of evaluated brains (Replace)
struct Entry { Genome g; double sum = 0.0; int runs = 0; int id = 0; Result last; double avg() const { return sum / std::max(runs, 1); } };
static std::vector<Entry> archive;
static int nextId = 1;
static int generation = 0;
static float sigma = 0.15f;
static Genome best;
static double bestScore = 1e18;
static int bestGen = -1;
static std::vector<double> history;  // the best score of each generation (HUD)
static Result bestRes;               // the best ever's run
static std::mt19937 rng;

static float U() { return std::uniform_real_distribution<float>(0.0f, 1.0f)(rng); }
static float N01() { return std::normal_distribution<float>(0.0f, 1.0f)(rng); }
static int Pick(int n) { return std::uniform_int_distribution<int>(0, std::max(n - 1, 0))(rng); }
static int ModeIndex(const std::string& m) {
    for (size_t i = 0; i < modes.size(); i++) if (modes[i] == m) return static_cast<int>(i);
    // (an algorithm taken out of Modes: pursuit if there)
    for (size_t i = 0; i < modes.size(); i++) if (modes[i] == "pursuit") return static_cast<int>(i);
    return 0;
}
static bool Uses(int gene, int mode) {
    const std::string u = kGenes[gene].uses;
    if (u.empty()) return true;
    const std::string m = modes[std::clamp(mode, 0, static_cast<int>(modes.size()) - 1)];
    for (const std::string& w : ParseWords(u)) if (w == m) return true;
    return false;
}
static std::string StatePath() { return baseDir + "proving\\ga_state.txt"; }

static void Read(CIniReader& ini, const Settings& s) {
    on = ini.ReadInteger("GA", "Enabled", 0) != 0;
    elite = ini.ReadInteger("GA", "Elite", elite);
    minPerMode = ini.ReadInteger("GA", "MinPerMode", minPerMode);
    steady = ini.ReadInteger("GA", "SteadyState", 0) != 0;
    reevalChance = ini.ReadFloat("GA", "ReevalChance", reevalChance);
    nnSigma = ini.ReadFloat("GA", "NnSigma", nnSigma);
    nn2Es = ini.ReadInteger("GA", "Nn2Es", 0) != 0;
    nn2Sigma = ini.ReadFloat("GA", "Nn2Sigma", nn2Sigma);
    nn2Alpha = ini.ReadFloat("GA", "Nn2Alpha", nn2Alpha);
    nn2Noise = ini.ReadFloat("GA", "Nn2Noise", nn2Noise);
    nn2Cars = ini.ReadInteger("GA", "Nn2Cars", nn2Cars);
    nn2Min = ini.ReadInteger("GA", "Nn2Min", nn2Min);
    tournament = std::max(ini.ReadInteger("GA", "Tournament", tournament), 1);
    immigrants = ini.ReadInteger("GA", "Immigrants", immigrants);
    maxGenerations = ini.ReadInteger("GA", "Generations", maxGenerations);
    mutRate = ini.ReadFloat("GA", "MutationRate", mutRate);
    sigma0 = ini.ReadFloat("GA", "Sigma", sigma0);
    sigmaDecay = ini.ReadFloat("GA", "SigmaDecay", sigmaDecay);
    sigmaMin = ini.ReadFloat("GA", "SigmaMin", sigmaMin);
    modeMutation = ini.ReadFloat("GA", "ModeMutation", modeMutation);
    offLimitGA = ini.ReadFloat("GA", "OffLimit", offLimitGA);
    meanPenalty = ini.ReadFloat("GA", "MeanPenalty", meanPenalty);
    devWeight = ini.ReadFloat("GA", "DevWeight", devWeight);
    scoreVersion = ini.ReadInteger("GA", "ScoreVersion", 1);
    traceOn = ini.ReadInteger("GA", "Trace", 1) != 0;
    traceEvery = ini.ReadFloat("GA", "TraceEvery", 0.0f);
    parentTop = std::clamp(ini.ReadFloat("GA", "ParentTop", parentTop), 0.01f, 1.0f);
    sigmaMax = ini.ReadFloat("GA", "SigmaMax", sigmaMax);
    offPenalty = ini.ReadFloat("GA", "OffPenalty", offPenalty);
    cutoffFactor = ini.ReadFloat("GA", "CutoffFactor", cutoffFactor);
    cutPenalty = ini.ReadFloat("GA", "CutPenalty", cutPenalty);
    edgePenalty = ini.ReadFloat("GA", "EdgePenalty", edgePenalty);
    edgeComfortGA = ini.ReadFloat("GA", "EdgeComfort", edgeComfortGA);
    rmsPenalty = ini.ReadFloat("GA", "RmsPenalty", rmsPenalty);
    resume = ini.ReadInteger("GA", "Resume", 1) != 0;
    seed = static_cast<unsigned int>(ini.ReadInteger("GA", "Seed", 0));
    auto m = ParseWords(ini.ReadString("GA", "Modes", ""));
    if (!m.empty()) modes = m;
    writeCsv = !on || ini.ReadInteger("GA", "Csv", 0) != 0;
    base = s;
}

static Genome FromSettings(const Settings& s) {
    Genome g;
    g.mode = ModeIndex(s.mode);
    for (int i = 0; i < kNumGenes; i++) g.g[i] = std::clamp(s.*kGenes[i].field, kGenes[i].lo, kGenes[i].hi);
    return g;
}
static Settings ToSettings(const Genome& g) {
    Settings s = base;
    s.mode = modes[std::clamp(g.mode, 0, static_cast<int>(modes.size()) - 1)];
    for (int i = 0; i < kNumGenes; i++) s.*kGenes[i].field = g.g[i];
    return s;
}
static Genome Random() {
    Genome g;
    g.mode = Pick(static_cast<int>(modes.size()));
    for (int i = 0; i < kNumGenes; i++) g.g[i] = kGenes[i].lo + U() * (kGenes[i].hi - kGenes[i].lo);
    return g;
}
static std::string GenesText(const Genome& g, const char* sep) {
    std::string s;
    for (int i = 0; i < kNumGenes; i++) s += Format("%s%.4g", i ? sep : "", g.g[i]);
    return s;
}
static std::string ShortText(const Genome& g) {
    std::string s = modes[g.mode];
    for (int i = 0; i < kNumGenes; i++) s += Format(" %s %.3g", kGenes[i].key, g.g[i]);
    return s;
}

static bool IsNeural(const Genome& g) { return modes[std::clamp(g.mode, 0, static_cast<int>(modes.size()) - 1)] == "neural"; }
// a neural genome's first weights: tools/nn_clone.py's (a human's lap) if there, else small random ones
// Neural2's evolution strategies (OpenAI-ES, Salimans et al. 2017): one central network theta
// (scripts\proving\nn2_central.txt: tools/nn2_train.py writes it, the game reloads it when it changes); its cars drive
// theta + sigma eps and theta - sigma eps in pairs (and one theta itself); after the generation, with Nn2Es = 1, theta
// moves by Nn2Alpha / (N sigma) x sum of each pair's eps weighted by its rank (centred ranks of the score)
static bool IsNeural2(const Genome& g) { return modes[std::clamp(g.mode, 0, static_cast<int>(modes.size()) - 1)] == "neural2"; }
static std::vector<float> nn2Theta;
static FILETIME nn2Stamp{};
static std::vector<float> nn2Eps[kMaxRunners];     // the perturbation each car drove (empty: none)
static std::string Nn2Path() { return baseDir + "proving\\nn2_central.txt"; }
// a theta with fewer outputs (the same layers): the new output rows added at 0, what it learned kept
static bool PadTheta(std::vector<float>& w) {
    const int sz = static_cast<int>(w.size());
    if (sz < kNN2 && sz >= kNn2L3 + (kNn2Hid2 + 1) && (kNN2 - sz) % (kNn2Hid2 + 1) == 0) { w.resize(kNN2, 0.0f); return true; }
    // (more outputs: the last ones dropped, the rest kept)
    if (sz > kNN2 && (sz - kNN2) % (kNn2Hid2 + 1) == 0) { w.resize(kNN2); return true; }
    return false;
}
static void LoadTheta() {
    WIN32_FILE_ATTRIBUTE_DATA a;
    if (!GetFileAttributesExA(Nn2Path().c_str(), GetFileExInfoStandard, &a) || CompareFileTime(&a.ftLastWriteTime, &nn2Stamp) == 0) return;
    std::ifstream f(Nn2Path());
    std::vector<float> w;
    float x;
    while (f >> x) w.push_back(x);
    // (fewer outputs: the same layers, the new output rows added at 0: what it learned kept)
    const int sz = static_cast<int>(w.size());
    if (PadTheta(w))
        Log(Format("neural2: theta of %d outputs made %d (what it learned kept)", (sz - kNn2L3) / (kNn2Hid2 + 1), kNn2Out));
    if (static_cast<int>(w.size()) != kNN2) return;
    nn2Theta = w;
    nn2Stamp = a.ftLastWriteTime;
    Log(Format("neural2: theta loaded (%d weights)", kNN2));
}
static void SaveTheta();
// No theta of this size (a new network: more inputs, the line output): hidden weights small and random, the outputs 0
// (the teacher itself until ES moves it); the ASI needs no tool for it
static void InitTheta() {
    if (static_cast<int>(nn2Theta.size()) == kNN2) return;
    nn2Theta.assign(kNN2, 0.0f);
    std::mt19937 g(12345);
    std::normal_distribution<float> nd1(0.0f, 1.0f / std::sqrt(static_cast<float>(kNn2In))), nd2(0.0f, 1.0f / std::sqrt(static_cast<float>(kNn2Hid)));
    for (int w = 0; w < kNn2L2; w++) nn2Theta[w] = nd1(g);
    for (int w = kNn2L2; w < kNn2L3; w++) nn2Theta[w] = nd2(g);
    SaveTheta();
    Log(Format("neural2: a new theta (%d inputs, %d and %d hidden, %d outputs: %d weights), outputs 0: its teacher", kNn2In, kNn2Hid, kNn2Hid2, kNn2Out, kNN2));
}
static void SaveTheta() {
    {
        std::ofstream f(Nn2Path(), std::ios::trunc);
        for (float x : nn2Theta) f << x << " ";
        f << "\n";
    }
    WIN32_FILE_ATTRIBUTE_DATA a;
    if (GetFileAttributesExA(Nn2Path().c_str(), GetFileExInfoStandard, &a)) nn2Stamp = a.ftLastWriteTime;
}
// Theta and its perturbations onto the neural2 genomes (pairs: +eps, -eps; the first is theta itself)
static void AssignNn2(Genome* g, int n) {
    LoadTheta();
    InitTheta();
    const float sg = nn2Es ? nn2Sigma : nn2Noise;
    int j = 0;
    std::vector<float> eps(kNN2);
    for (int k = 0; k < n; k++) {
        nn2Eps[k].clear();
        if (!IsNeural2(g[k])) continue;
        if (static_cast<int>(nn2Theta.size()) != kNN2) { for (float& w : g[k].nn) w = 0.0f; continue; } // (no theta yet: the teacher steers)
        if (j > 0 && j % 2 == 1) for (float& e : eps) e = N01();
        const float sign = j == 0 ? 0.0f : j % 2 == 1 ? 1.0f : -1.0f;
        nn2Eps[k].resize(kNN2);
        for (int w = 0; w < kNN2; w++) { nn2Eps[k][w] = sign * eps[w]; g[k].nn[w] = nn2Theta[w] + sg * nn2Eps[k][w]; }
        if (j == 0) nn2Eps[k].clear();          // (theta itself: shown, not used for the step)
        j++;
    }
}
// After a generation: theta's step from how its perturbations scored
static void StepTheta(int n) {
    if (!nn2Es || static_cast<int>(nn2Theta.size()) != kNN2) return;
    std::vector<int> ks;
    for (int k = 0; k < n; k++) if (IsNeural2(pop[k]) && !nn2Eps[k].empty()) ks.push_back(k);
    if (ks.size() < 2) return;
    std::vector<int> order = ks;
    std::sort(order.begin(), order.end(), [](int a, int b) { return res[a].score > res[b].score; }); // worst first
    std::vector<float> step(kNN2, 0.0f);
    const int m = static_cast<int>(order.size());
    for (int r = 0; r < m; r++) {
        const float u = static_cast<float>(r) / (m - 1) - 0.5f;   // centred rank: the best +0.5
        for (int w = 0; w < kNN2; w++) step[w] += u * nn2Eps[order[r]][w];
    }
    for (int w = 0; w < kNN2; w++) nn2Theta[w] += nn2Alpha / (m * nn2Sigma) * step[w] - 0.0005f * nn2Alpha * nn2Theta[w];
    SaveTheta();
    double best = 1e18;
    for (int k : ks) best = std::min(best, res[k].score);
    Log(Format("neural2: ES step from %d cars (best of them %.1f)", m, best));
}

static void SeedWeights(Genome& g) {
    std::ifstream f(baseDir + "proving\\nn_seed.txt");
    std::vector<float> w;
    float x;
    while (f >> x) w.push_back(x);
    for (int i = 0; i < kNN; i++)
        g.nn[i] = static_cast<int>(w.size()) == kNN ? w[i] + N01() * nnSigma : N01() * 0.3f;
}

static void Apply() {
    for (int k = 0; k < numRunners; k++) {
        Settings s = ToSettings(pop[k]);
        if (IsNeural(pop[k])) s.nn.assign(pop[k].nn, pop[k].nn + kNN);
        // (neural2 with no theta yet: no weights, it drives as its teacher)
        if (IsNeural2(pop[k]) && static_cast<int>(nn2Theta.size()) == kNN2) s.nn.assign(pop[k].nn, pop[k].nn + kNN2);
        s.driver = base.driver;
        CfgOf(k) = s;
    }
}

static void Save() {
    std::ostringstream f;
    f << "genes";
    for (const auto& gd : kGenes) f << " " << gd.key;
    f << "\ngeneration " << generation << "\nsigma " << sigma << "\n";
    if (bestGen >= 0) f << "best " << bestScore << " " << bestGen << " " << modes[best.mode] << " " << GenesText(best, " ") << "\n"
                        << "bestrun " << bestRes.time << " " << bestRes.mean << " " << bestRes.maxE << " " << bestRes.out << "\n";
    f << "history";
    for (double h : history) f << " " << h;
    f << "\nhistory10";
    for (double h : history10) f << " " << h;
    f << "\nscoring " << devWeight << " " << scoreVersion << "\n";
    // (steady state: the archive's brains, best first; they start the lanes on a resume)
    std::vector<const Genome*> out;
    if (steady && !archive.empty()) {
        std::vector<const Entry*> es;
        for (const auto& e : archive) es.push_back(&e);
        std::sort(es.begin(), es.end(), [](const Entry* a, const Entry* b) { return a->avg() < b->avg(); });
        for (const Entry* e : es) out.push_back(&e->g);
    }
    else for (int k = 0; k < numRunners; k++) out.push_back(&pop[k]);
    for (const Genome* gp : out)
    {
        const Genome& gk = *gp;
        f << "ind " << modes[gk.mode] << " " << GenesText(gk, " ") << (gk.label.empty() ? "" : " | " + gk.label) << "\n";
        if (IsNeural(gk)) {
            f << "nn";
            for (int i = 0; i < kNN; i++) f << " " << gk.nn[i];
            f << "\n";
        }
    }
    Disk::Write(StatePath(), f.str());
}

static bool Load() {
    std::ifstream f(StatePath());
    if (!f) return false;
    std::string line;
    std::vector<std::string> names;
    for (const auto& gd : kGenes) names.push_back(gd.key);
    int n = 0;
    while (std::getline(f, line)) {
        std::istringstream is(line);
        std::string key;
        is >> key;
        if (key == "genes") {
            // the saved genes by name: one this build has and the file not keeps the ini's value
            names.clear();
            std::string nm;
            while (is >> nm) names.push_back(nm);
        }
        else if (key == "generation") is >> generation;
        else if (key == "sigma") is >> sigma;
        else if (key == "history") { double h; while (is >> h) history.push_back(h); }
        else if (key == "history10") { double h; while (is >> h) history10.push_back(h); }
        else if (key == "scoring") { loadedVer = 1; is >> loadedDev; if (!(is >> loadedVer)) loadedVer = 1; }
        else if (key == "bestrun") is >> bestRes.time >> bestRes.mean >> bestRes.maxE >> bestRes.out;
        else if (key == "nn" && n > 0) for (int i = 0; i < kNN; i++) is >> pop[n - 1].nn[i];
        else if (key == "best" || key == "ind") {
            Genome g;
            std::string mode;
            double score = 0.0;
            int gen = 0;
            if (key == "best") is >> score >> gen;
            is >> mode;
            g = FromSettings(base);
            g.mode = ModeIndex(mode);
            for (const std::string& nm : names) {
                float v = 0.0f;
                is >> v;
                for (int i = 0; i < kNumGenes; i++) if (nm == kGenes[i].key) g.g[i] = std::clamp(v, kGenes[i].lo, kGenes[i].hi);
            }
            const bool ok = !is.fail();
            {
                // "| name": a pilot profile
                std::string rest;
                std::getline(is, rest);
                const size_t bar = rest.find('|');
                if (bar != std::string::npos) {
                    const size_t a = rest.find_first_not_of(" \t", bar + 1), b = rest.find_last_not_of(" \t\r");
                    if (a != std::string::npos) g.label = rest.substr(a, b - a + 1);
                }
            }
            if (!ok) continue;
            if (key == "best") { best = g; bestScore = score; bestGen = gen; }
            else if (n < numRunners) pop[n++] = g;
        }
    }
    if (std::abs(loadedDev - devWeight) > 1e-4f || loadedVer != scoreVersion) {
        // scored another way: the old best and history are not comparable (the brains are kept)
        Log(Format("GA: the score changed (DevWeight %.2f -> %.2f, version %d -> %d): best ever and history start over", loadedDev, devWeight,
            loadedVer, scoreVersion));
        if (loadedVer != scoreVersion) {
            // (the report's rows too: kept aside under the old version's name)
            const std::string rows = baseDir + "proving\\ga_generations.tsv";
            MoveFileExA(rows.c_str(), (baseDir + Format("proving\\ga_generations_v%d.tsv", loadedVer)).c_str(), MOVEFILE_REPLACE_EXISTING);
        }
        bestScore = 1e18;
        bestGen = -1;
        bestRes = Result();
        history.clear();
        history10.clear();
    }
    // a bigger population than the saved one: copies of the saved brains, a little mutated (not random: no abrupt
    // changes); none saved: random
    const int saved = n;
    for (; n < numRunners; n++) {
        if (saved == 0) { pop[n] = Random(); continue; }
        Genome g = pop[n % saved];
        g.label.clear();
        for (int i = 0; i < kNumGenes; i++)
            if (Uses(i, g.mode)) g.g[i] = std::clamp(g.g[i] + N01() * sigma0 * (kGenes[i].hi - kGenes[i].lo), kGenes[i].lo, kGenes[i].hi);
        pop[n] = g;
    }
    // (genes the file did not have, added since: the ini's value for everyone; the user wants no abrupt changes in the
    // brains, so not drawn at random)
    return true;
}

static void WriteBest() {
    std::ostringstream f;
    f << Format("; GA best ever: generation %d, score %.2f (lower is better)\n", bestGen, bestScore);
    for (const char* sec : { "Steer", "Speed" }) {
        f << "[" << sec << "]\n";
        if (!strcmp(sec, "Steer")) f << "Mode = " << modes[best.mode] << "\n";
        for (int i = 0; i < kNumGenes; i++) if (!strcmp(kGenes[i].sec, sec)) f << kGenes[i].key << " = " << Format("%.4g", best.g[i]) << "\n";
    }
    Disk::Write(baseDir + "proving\\ga_best.ini", f.str());
}

static void ResetBatches();
static void LoadRows();
static void Start() {
    if (!on) return;
    rng.seed(seed ? seed : GetTickCount());
    generation = 0;
    sigma = sigma0;
    history.clear();
    history10.clear();
    loadedDev = -1.0f;
    loadedVer = 1;
    stall = 0;
    stallRef = 1e18;
    ResetBatches();
    bestScore = 1e18;
    bestGen = -1;
    bestRes = Result();
    const bool resumed = resume && Load();
    LoadRows();
    if (!resumed) {
        // the ini's own controller, then one of each algorithm, then random ones
        pop[0] = FromSettings(base);
        for (int k = 1; k < numRunners; k++) {
            pop[k] = Random();
            if (k <= static_cast<int>(modes.size())) pop[k].mode = (k - 1) % static_cast<int>(modes.size());
        }
    }
    for (int k = 0; k < kMaxRunners; k++) res[k] = Result();
    AssignNn2(pop, numRunners);
    Apply();
    Log(Format("GA: %s, generation %d, population %d, elite %d, sigma %.3f, modes %d, best ever %s", resumed ? "resumed" : "new",
        generation, numRunners, elite, sigma, static_cast<int>(modes.size()), bestGen >= 0 ? Format("%.2f (gen %d)", bestScore, bestGen).c_str() : "-"));
}

static void Score(int k, const std::string& why, float time, double mean, double rms, double maxE, double out, float sMax, float length,
    double deficit, double minMargin) {
    Result& r = res[k];
    const bool done = why == "end";
    // the best is not the fastest: the one that is still fast and keeps inside the road. A corridor (Width): anywhere
    // with EdgeComfort m of room is free; EdgePenalty per m x s closer to an edge, OffPenalty per s with the body past
    // it (MeanPenalty x mean offset from the centre line: 0 with a corridor)
    // (CUT: too slow to wait for, scored by the time it was heading for, its outside time so far and CutPenalty)
    const bool cut = why == "CUT" && sMax > 50.0f;
    const double t = cut ? time * length / sMax : time;
    r.score += done || cut ? t * (1.0 + devWeight * mean) + meanPenalty * mean + edgePenalty * deficit + offPenalty * out + rmsPenalty * rms + (cut ? cutPenalty : 0.0)
                           : 400.0 + 600.0 * std::max(1.0 - sMax / std::max(length, 1.0f), 0.0) + 0.25 * std::min(time * length / std::max(sMax, 50.0f), 400.0f);
    // (not home: 400 + 600 x the share of the lap left + a quarter of the time it was heading for at its pace (at most
    // 100): further is always better, and at the same place the faster; 1000 + 0.5 x m left saw no speed. The time
    // outside the lane not counted there: crashing early scored better than driving on, outside. A CUT car, which drove
    // on, always better than one that crashed; any car home better than both)
    if (cut) r.score = std::min(r.score, 399.0);
    r.mean = std::max(r.mean, mean);
    r.deficit += deficit;
    r.minMargin = std::min(r.minMargin, minMargin);
    r.time += time;
    r.rms = std::max(r.rms, rms);
    r.maxE = std::max(r.maxE, maxE);
    r.out += out;
    r.reached += sMax;
    if (!done || r.why.empty()) r.why = why;
    r.runs++;
}

// mode >= 0: among that algorithm's only (its own children: S2 of Gemini's 4th review)
static int Tournament(const std::vector<double>& avg, int mode = -1) {
    std::vector<int> pool;
    for (int k = 0; k < numRunners; k++) if (mode < 0 || pop[k].mode == mode) pool.push_back(k);
    if (pool.empty()) return Pick(numRunners);
    int bestK = pool[Pick(static_cast<int>(pool.size()))];
    for (int t = 1; t < tournament; t++) {
        const int k = pool[Pick(static_cast<int>(pool.size()))];
        if (avg[k] < avg[bestK]) bestK = k;
    }
    return bestK;
}

static void ReportPop(int n);
static void NextGeneration() {
    const int n = numRunners;
    ReportPop(n);
    // ranked by the average of the generations each genome drove (a new one: this run)
    std::vector<double> avg(n);
    for (int k = 0; k < n; k++) {
        pop[k].sum += res[k].score;
        pop[k].runs++;
        if (pop[k].runs > 5) { pop[k].sum *= 5.0 / 6.0; pop[k].runs = 5; } // (the last ~5)
        avg[k] = pop[k].sum / pop[k].runs;
    }
    if (human && n > 0) avg[0] = 1e18; // (car A is the human: scored and logged, never chosen)
    std::vector<int> order(n);
    for (int k = 0; k < n; k++) order[k] = k;
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return avg[a] < avg[b]; });
    Result top = res[order[0]];
    top.score = avg[order[0]];
    history.push_back(top.score);
    if (top.score < bestScore) {
        bestScore = top.score;
        best = pop[order[0]];
        bestRes = top;
        bestGen = generation;
        WriteBest();
    }
    Log(Format("\n=== Generation %d (sigma %.3f): best %s %.2f (average of %d runs), best ever %.2f (gen %d)", generation, sigma,
        kTags[order[0]], top.score, pop[order[0]].runs, bestScore, bestGen));
    const std::string histPath = baseDir + "proving\\ga_history.csv";
    std::string header = "stamp,generation,rank,car,mode,score,result,time_s,mean_m,rms_m,max_m,out_s,reached_m,sigma";
    for (const auto& gd : kGenes) header += std::string(",") + gd.key;
    header += ",label,avg_score,runs,deficit_ms,min_margin_m";
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    {
        // other genes than the file's columns: that file kept as ga_history_<stamp>.csv (tools/ga_review.py reads all)
        std::ifstream old(histPath);
        std::string first;
        if (old && std::getline(old, first) && first != header) {
            old.close();
            MoveFileA(histPath.c_str(), (baseDir + "proving\\ga_history_" + stamp + ".csv").c_str());
        }
    }
    const bool fresh = GetFileAttributesA(histPath.c_str()) == INVALID_FILE_ATTRIBUTES;
    std::ofstream h(histPath, std::ios::app);
    if (fresh) h << header << "\n";
    for (int r = 0; r < n; r++) {
        const int k = order[r];
        const Result& x = res[k];
        Log(Format("  %2d. %s %-9s score %8.2f  %-7s %6.1f s  offset mean %.2f max %5.2f m  room min %.2f m, near the edge %.1f m.s  out %5.1f s  | %s",
            r + 1, kTags[k], human && k == 0 ? "HUMANO" : modes[pop[k].mode].c_str(), x.score, x.why.c_str(), x.time, x.mean, x.maxE, x.minMargin < 1e8 ? x.minMargin : 0.0,
            x.deficit, x.out, ShortText(pop[k]).c_str()));
        h << Format("%s,%d,%d,%s,%s,%.3f,%s,%.2f,%.3f,%.3f,%.3f,%.2f,%.0f,%.4f,", stamp, generation, r + 1, kTags[k],
            human && k == 0 ? "human" : modes[pop[k].mode].c_str(),
            x.score, x.why.c_str(), x.time, x.mean, x.rms, x.maxE, x.out, x.reached, sigma) << GenesText(pop[k], ",") << "," << pop[k].label
          << Format(",%.3f,%d,%.3f,%.3f", avg[k], pop[k].runs, x.deficit, x.minMargin < 1e8 ? x.minMargin : 0.0) << "\n";
    }

    // the next generation: elite first (lane A, CJ's car, is the best), then the islands, children, immigrants
    static Genome next[kMaxRunners]; // (static: ~6.5 KB a genome, 100+ of them blew the game's stack)
    const int e = std::clamp(elite, 0, n);
    const int imm = std::clamp(immigrants, 0, std::max(n - e - 1, 0));
    for (int i = 0; i < e; i++) next[i] = pop[order[i]];
    // Islands (MinPerMode): every algorithm keeps at least that many cars, its best that finished kept as it is and
    // the rest its own children, so a new or a weaker algorithm gets generations to be tuned instead of losing every
    // tournament against 50 tuned pursuits (the user wanted more than one viable AI). Without MinPerMode: the best
    // that finished of every other algorithm and one child of its own
    int kept = e;
    std::vector<int> slots;                                  // the algorithm of each car still to breed (-1: any)
    const int nm = static_cast<int>(modes.size());
    for (int m = 0; m < nm; m++) {
        int have = 0;
        for (int i = 0; i < e; i++) have += next[i].mode == m;
        const int quota = modes[m] == "neural2" ? std::max(minPerMode, nn2Cars) : minPerMode; // (neural2: its ES's pairs)
        int need = minPerMode > 0 ? std::max(quota - have, 0) : (have ? 0 : 2);
        if (need <= 0) continue;
        for (int r = e; r < n; r++)
            if (pop[order[r]].mode == m && res[order[r]].why == "end" && !(human && order[r] == 0)) { next[kept++] = pop[order[r]]; need--; break; }
        for (int k = 0; k < need; k++) slots.push_back(m);
    }
    while (kept + static_cast<int>(slots.size()) > n - imm && !slots.empty()) slots.pop_back();
    while (kept + static_cast<int>(slots.size()) < n - imm) slots.push_back(-1);
    for (int i = kept; i < n - imm; i++) {
        const int own = slots[i - kept];
        bool any = own < 0;
        for (int k = 0; k < n && !any; k++) any = pop[k].mode == own;
        if (!any) {
            // an algorithm with no car yet: the best overall's shared genes (speed, ABS, TC, line), its own at the ini's
            // values, a little mutated
            Genome c = pop[order[0]];
            const Genome ini = FromSettings(base);
            c.mode = own;
            c.label.clear();
            c.sum = 0.0;
            c.runs = 0;
            for (int g = 0; g < kNumGenes; g++) {
                if (Uses(g, own) && !Uses(g, pop[order[0]].mode)) c.g[g] = ini.g[g];
                if (Uses(g, own) && U() < mutRate)
                    c.g[g] = std::clamp(c.g[g] + N01() * sigma * (kGenes[g].hi - kGenes[g].lo), kGenes[g].lo, kGenes[g].hi);
            }
            if (IsNeural(c)) SeedWeights(c);
            next[i] = c;
            continue;
        }
        // (an island's child: one parent of its algorithm, the other of its algorithm too or, half the time, the best
        // overall: it keeps its algorithm and may take the shared genes, speed / ABS / TC, from the best)
        const Genome& a = pop[Tournament(avg, own)];
        const Genome& b = pop[Tournament(avg, own >= 0 && U() < 0.5f ? own : -1)];
        Genome c;
        c.mode = own >= 0 ? own : U() < 0.5f ? a.mode : b.mode;
        // each gene from the parent whose algorithm reads it (from the one that ignores it, it only rode along)
        for (int g = 0; g < kNumGenes; g++) {
            const bool ua = Uses(g, a.mode), ub = Uses(g, b.mode);
            c.g[g] = ua && !ub ? a.g[g] : ub && !ua ? b.g[g] : U() < 0.5f ? a.g[g] : b.g[g];
        }
        if (own < 0 && U() < modeMutation && modes.size() > 1) {
            // another algorithm: the genes it reads and the old one did not are drawn anew (they only rode along)
            const int old = c.mode;
            c.mode = (c.mode + 1 + Pick(static_cast<int>(modes.size()) - 1)) % static_cast<int>(modes.size());
            for (int g = 0; g < kNumGenes; g++)
                if (Uses(g, c.mode) && !Uses(g, old)) c.g[g] = kGenes[g].lo + U() * (kGenes[g].hi - kGenes[g].lo);
        }
        // a neural child's weights: each from a neural parent, then a little mutated
        if (IsNeural(c)) {
            const bool an = IsNeural(a), bn = IsNeural(b);
            if (!an && !bn) SeedWeights(c);
            else for (int w = 0; w < kNN; w++) c.nn[w] = an && bn ? (U() < 0.5f ? a.nn[w] : b.nn[w]) : an ? a.nn[w] : b.nn[w];
            for (int w = 0; w < kNN; w++) if (U() < mutRate) c.nn[w] += N01() * nnSigma;
        }
        // only the genes its algorithm reads (a third of the mutations went to genes the mode ignores)
        for (int g = 0; g < kNumGenes; g++)
            if (Uses(g, c.mode) && U() < mutRate)
                c.g[g] = std::clamp(c.g[g] + N01() * sigma * (kGenes[g].hi - kGenes[g].lo), kGenes[g].lo, kGenes[g].hi);
        next[i] = c;
    }
    for (int i = n - imm; i < n; i++) next[i] = Random();
    if (human && n > 1) std::swap(next[0], next[n - 1]); // (the best brain not on the human's car A)
    StepTheta(n);
    AssignNn2(next, n);
    for (int k = 0; k < n; k++) { pop[k] = next[k]; res[k] = Result(); }
    generation++;
    sigma = std::max(sigma * sigmaDecay, sigmaMin);
    Save();
    Apply();
}

// --- Steady state ([GA] SteadyState = 1) -----------------------------------------------------------------------------
// No waiting for the slowest car: the moment a car's run ends (home, out, cut...) it is scored into an archive of the
// best brains (as many as there are cars), and a brain bred from the archive starts at once on its lane. The islands
// hold by which algorithm the new brain gets (the one furthest under its quota), ReevalChance of the new runs are an
// elite again (its score the average of its runs), neural2's perturbations come in +- pairs and theta takes a step
// every Nn2Cars of them. Every 'cars' runs make a "generation" for the logs (history, trace, Gemini).
struct Done { Genome g; Result r; std::string id; };
// A generation = Runners brains in the order they were made (each runs once); it closes when all of its runs ended,
// while the next ones already drive (the user: exactly Runners runs a generation, its slow cars and its quick
// failures alike; by count of runs ended the quick failures filled it, by time it had 650-750)
static std::map<int, std::vector<Done>> batches;
static int birthGen = -1, birthCount = 0;
static void ResetBatches() { batches.clear(); birthGen = -1; birthCount = 0; }
static int NextBirth() {
    if (birthGen < 0) birthGen = generation;
    const int g = birthGen;
    if (++birthCount >= numRunners) { birthGen++; birthCount = 0; }
    return g;
}
// --- The report: scripts\proving\ga_report.html, written by the ASI when a generation closes ---------------------------
// The best car of each generation and how the GA and neural2 evolve, for the user to open in a browser with no tool
// running (the rows also in ga_generations.tsv, read back at the start: the report keeps every generation)
struct GenRow {
    int gen = 0, runs = 0, home = 0, nnRuns = 0, nnHome = 0, esSteps = 0;
    double score = 0.0, time = 0.0, mean = 0.0, top10 = 0.0, sigma = 0.0, nnBest = 0.0, nnReach = 0.0, nnOut = 0.0;
    std::string stamp, id, mode, genes;
};
static std::vector<GenRow> genRows;
static int esSteps = 0; // neural2's ES steps so far (StepThetaBatch)
static std::string RowsPath() { return baseDir + "proving\\ga_generations.tsv"; }
static void WriteReport();
static void LoadRows() {
    genRows.clear();
    std::ifstream f(RowsPath());
    std::string line;
    while (std::getline(f, line)) {
        std::vector<std::string> c;
        size_t a = 0;
        for (size_t b; (b = line.find('\t', a)) != std::string::npos; a = b + 1) c.push_back(line.substr(a, b - a));
        c.push_back(line.substr(a));
        if (c.size() < 19 || c[0] == "gen") continue;
        GenRow r;
        r.gen = std::atoi(c[0].c_str()); r.stamp = c[1]; r.runs = std::atoi(c[2].c_str()); r.home = std::atoi(c[3].c_str());
        r.id = c[4]; r.mode = c[5]; r.score = std::atof(c[6].c_str()); r.time = std::atof(c[7].c_str()); r.mean = std::atof(c[8].c_str());
        r.top10 = std::atof(c[9].c_str()); r.sigma = std::atof(c[10].c_str()); r.nnRuns = std::atoi(c[11].c_str());
        r.nnHome = std::atoi(c[12].c_str()); r.nnBest = std::atof(c[13].c_str()); r.nnReach = std::atof(c[14].c_str());
        r.esSteps = std::atoi(c[15].c_str()); r.nnOut = std::atof(c[16].c_str()); r.genes = c[18];
        genRows.push_back(r);
    }
    if (!genRows.empty()) { esSteps = genRows.back().esSteps; WriteReport(); }
}
static std::string Html(const std::string& t) {
    std::string o;
    for (char ch : t) o += ch == '<' ? "&lt;" : ch == '>' ? "&gt;" : ch == '&' ? "&amp;" : std::string(1, ch);
    return o;
}
// an SVG line chart: each series (value per generation, NAN = none) over the generations; lower is better if 'down'
static std::string Chart(const std::vector<std::pair<std::string, std::vector<double>>>& series, const std::vector<int>& gens,
    const char* const* colors, const char* unit, bool down) {
    const double W = 900.0, H = 260.0, L = 56.0, Rm = 16.0, T = 14.0, B = 30.0;
    double lo = 1e18, hi = -1e18;
    for (const auto& se : series) for (double v : se.second) if (!std::isnan(v)) { lo = std::min(lo, v); hi = std::max(hi, v); }
    std::string o = Format("<svg viewBox=\"0 0 %.0f %.0f\" class=\"chart\">", W, H);
    if (gens.empty() || lo > hi) return o + "<text x=\"20\" y=\"40\">ainda sem dados</text></svg>";
    if (hi - lo < 1e-6) { hi += 1.0; lo -= 1.0; }
    const double pad = 0.06 * (hi - lo);
    lo -= pad; hi += pad;
    const int n = static_cast<int>(gens.size());
    auto X = [&](int i) { return L + (n > 1 ? (W - L - Rm) * i / (n - 1) : (W - L - Rm) / 2); };
    auto Y = [&](double v) { return T + (H - T - B) * (hi - v) / (hi - lo); };
    for (int k = 0; k <= 4; k++) {
        const double v = lo + (hi - lo) * k / 4.0;
        o += Format("<line x1=\"%.0f\" x2=\"%.0f\" y1=\"%.1f\" y2=\"%.1f\" class=\"grid\"/><text x=\"%.0f\" y=\"%.1f\" class=\"ax\" text-anchor=\"end\">%.1f%s</text>",
            L, W - Rm, Y(v), Y(v), L - 6, Y(v) + 4, v, unit);
    }
    const int every = std::max(1, n / 12);
    for (int i = 0; i < n; i += every) o += Format("<text x=\"%.1f\" y=\"%.0f\" class=\"ax\" text-anchor=\"middle\">g%d</text>", X(i), H - 10, gens[i]);
    for (size_t si = 0; si < series.size(); si++) {
        std::string d;
        bool pen = false;
        for (int i = 0; i < n && i < static_cast<int>(series[si].second.size()); i++) {
            const double v = series[si].second[i];
            if (std::isnan(v)) { pen = false; continue; }
            d += Format("%s%.1f,%.1f ", pen ? "L" : "M", X(i), Y(v));
            pen = true;
            o += Format("<circle cx=\"%.1f\" cy=\"%.1f\" r=\"2.5\" fill=\"%s\"><title>g%d: %.2f%s</title></circle>", X(i), Y(v), colors[si], gens[i], v, unit);
        }
        // (Format is limited to 4096 characters: the long parts joined as strings)
        o += "<path d=\"" + d + "\" fill=\"none\" stroke=\"" + colors[si] + "\" stroke-width=\"2\"/>";
    }
    o += "</svg><div class=\"legend\">";
    for (size_t si = 0; si < series.size(); si++)
        o += Format("<span><i style=\"background:%s\"></i>%s</span>", colors[si], Html(series[si].first).c_str());
    return o + Format("<span class=\"dim\">%s</span></div>", down ? "mais baixo = melhor" : "mais alto = melhor");
}
static std::vector<std::string> secNames;          // (the course's parts, for the report)
static std::vector<float> secBest, secWin;          // ... the last generation's best of each and its winner's
static std::vector<std::string> secBestId;
static int secGen = -1;
static void WriteReport() {
    std::vector<int> gens;
    std::vector<double> bestS, top10, nnBest, nnRate, nnReach, nnOut;
    for (const auto& r : genRows) {
        gens.push_back(r.gen);
        bestS.push_back(r.score < 1000.0 ? r.score : NAN);
        top10.push_back(r.top10 < 1000.0 ? r.top10 : NAN);
        nnBest.push_back(r.nnHome > 0 ? r.nnBest : NAN);
        nnRate.push_back(r.nnRuns > 0 ? 100.0 * r.nnHome / r.nnRuns : NAN);
        nnReach.push_back(r.nnRuns > 0 ? r.nnReach / 1000.0 : NAN);
        nnOut.push_back(r.nnOut);
    }
    static const char* const kC1[] = { "#3d8bfd", "#f59f00" };
    static const char* const kC2[] = { "#b072ff" };
    static const char* const kC3[] = { "#2fb380", "#e8590c" };
    std::ostringstream h;
    h << "<!doctype html><html lang=\"pt-br\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
         "<meta http-equiv=\"refresh\" content=\"60\"><title>Proving Ground GA</title><style>"
         ":root{--bg:#f6f7f9;--fg:#1d2330;--card:#fff;--dim:#6b7280;--line:#e3e6eb}"
         "@media (prefers-color-scheme:dark){:root{--bg:#12151b;--fg:#e8eaee;--card:#1b2029;--dim:#8b93a1;--line:#2a313d}}"
         "body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,sans-serif}"
         "main{max-width:980px;margin:0 auto;padding:20px 16px}h1{font-size:22px;margin:0 0 4px}h2{font-size:16px;margin:26px 0 8px}"
         ".dim{color:var(--dim)}.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin:10px 0}"
         ".kpis{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}.kpi b{display:block;font-size:22px}"
         ".chart{width:100%;height:auto}.chart .grid{stroke:var(--line)}.chart .ax{fill:var(--dim);font-size:11px}"
         ".legend span{margin-right:16px;font-size:12px}.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:5px}"
         ".tw{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:12.5px}th,td{text-align:left;padding:5px 8px;border-bottom:1px solid var(--line);white-space:nowrap}"
         "th{color:var(--dim);font-weight:600}td.num{text-align:right;font-variant-numeric:tabular-nums}.genes{white-space:normal;color:var(--dim);font-size:11.5px;min-width:260px}"
         "</style></head><body><main>";
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%d/%m/%Y %H:%M", std::localtime(&now));
    h << "<h1>Proving Ground: algoritmo genetico</h1><div class=\"dim\">Gerado pelo RealTrafficFix.SA.asi em " << stamp
      << " ao fechar a geracao " << (genRows.empty() ? 0 : genRows.back().gen)
      << " (a pagina recarrega a cada minuto). " << Html(Format("pontos = tempo + %.0f x m.s com a roda mais de dentro alem da borda + %.0f x s com as 4 rodas fora (uma a tres "
             "rodas fora nao custa nada); quem nao chega: 1000 + 0,5 x m que faltaram", edgePenalty, offPenalty)) << "</div>";
    h << "<div class=\"kpis\">";
    if (bestGen >= 0)
        h << Format("<div class=\"card kpi\"><span class=\"dim\">recorde (g%d)</span><b>%.2f</b>%s, %.1f s, desvio %.3f m</div>", bestGen, bestScore,
            Html(modes[best.mode]).c_str(), bestRes.time, bestRes.mean);
    if (!genRows.empty()) {
        const GenRow& r = genRows.back();
        h << Format("<div class=\"card kpi\"><span class=\"dim\">ultima geracao (g%d)</span><b>%.2f</b>%s %s</div>", r.gen, r.score, Html(r.id).c_str(), Html(r.mode).c_str());
        h << Format("<div class=\"card kpi\"><span class=\"dim\">media dos 10 melhores</span><b>%.2f</b>%d de %d chegaram</div>", r.top10, r.home, r.runs);
        h << Format("<div class=\"card kpi\"><span class=\"dim\">neural2 na ultima</span><b>%d/%d</b>chegaram%s</div>", r.nnHome, r.nnRuns,
            r.nnHome > 0 ? Format(", melhor %.2f", r.nnBest).c_str() : "");
    }
    h << "</div>";
    h << "<h2>Evolucao do GA</h2><div class=\"card\">"
      << Chart({ { "melhor da geracao", bestS }, { "media dos 10 melhores", top10 } }, gens, kC1, "", true) << "</div>";
    h << "<h2>Evolucao da IA neural (neural2)</h2><div class=\"dim\">Uma rede (63 sensores, 24 neuronios) que corrige o volante do pursuit "
         "(ate 1/4 do curso) e a velocidade-alvo do planejador (ate 30%); aprende por estrategias evolutivas: cada passo junta as voltas de "
      << nn2Cars << " carros. Passos ate agora: " << esSteps << ".</div>";
    h << "<div class=\"card\">" << Chart({ { "neural2: melhor pontuacao", nnBest } }, gens, kC2, "", true) << "</div>";
    h << "<div class=\"card\">" << Chart({ { "% das voltas que chegaram", nnRate } }, gens, kC3, "%", false) << "</div>";
    h << "<div class=\"card\">" << Chart({ { "distancia media percorrida (km)", nnReach }, { "tamanho da correcao (RMS da camada de saida)", nnOut } },
        gens, kC3, "", false) << "</div>";
    if (!secNames.empty()) {
        // where the time is: each part's best this generation (any car) against the winner's
        double sumBest = 0.0, sumWin = 0.0;
        h << Format("<h2>Setores da geracao %d</h2><div class=\"dim\">O melhor de cada trecho (qualquer carro) contra o vencedor da "
                    "geracao: a soma dos melhores e o tempo possivel juntando os melhores trechos.</div><div class=\"card tw\"><table><tr>"
                    "<th>trecho</th><th>melhor</th><th>de</th><th>vencedor</th><th>perde</th></tr>", secGen);
        for (size_t q = 0; q < secNames.size(); q++) {
            const float b = q < secBest.size() ? secBest[q] : -1.0f, w = q < secWin.size() ? secWin[q] : -1.0f;
            if (b > 0.0f) sumBest += b;
            if (w > 0.0f) sumWin += w;
            h << "<tr><td>" << Html(secNames[q]) << "</td>"
              << (b > 0.0f ? Format("<td class=\"num\">%.2f s</td><td>%s</td>", b, Html(secBestId[q]).c_str()) : std::string("<td>-</td><td></td>"))
              << (w > 0.0f ? Format("<td class=\"num\">%.2f s</td>", w) : std::string("<td>-</td>"))
              << (b > 0.0f && w > 0.0f ? Format("<td class=\"num\">%+.2f s</td>", w - b) : std::string("<td></td>")) << "</tr>";
        }
        h << Format("<tr><th>soma</th><th class=\"num\">%.2f s</th><th></th><th class=\"num\">%.2f s</th><th class=\"num\">%+.2f s</th></tr></table></div>",
            sumBest, sumWin, sumWin - sumBest);
    }
    h << "<h2>Melhor carro de cada geracao</h2><div class=\"card tw\"><table><tr><th>geracao</th><th>quando</th><th>carro</th><th>algoritmo</th>"
         "<th>pontos</th><th>tempo</th><th>desvio</th><th>top 10</th><th>chegaram</th><th>neural2</th><th>sigma</th><th>genes</th></tr>";
    for (auto it = genRows.rbegin(); it != genRows.rend(); ++it) {
        const GenRow& r = *it;
        h << Format("<tr><td>g%d</td><td>%s</td><td>%s</td><td>%s</td><td class=\"num\">%.2f</td><td class=\"num\">%.1f s</td><td class=\"num\">%.3f m</td>"
                    "<td class=\"num\">%.2f</td><td class=\"num\">%d/%d</td><td class=\"num\">%d/%d%s</td><td class=\"num\">%.3f</td>",
            r.gen, Html(r.stamp).c_str(), Html(r.id).c_str(), Html(r.mode).c_str(), r.score, r.time, r.mean, r.top10, r.home, r.runs, r.nnHome, r.nnRuns,
            r.nnHome > 0 ? Format(" (%.1f)", r.nnBest).c_str() : "", r.sigma) << "<td class=\"genes\">" << Html(r.genes) << "</td></tr>";
    }
    h << "</table></div></main></body></html>\n";
    Disk::Write(baseDir + "proving\\ga_report.html", h.str());
}
// the closed generation g into the rows, the tsv and the report
static void ReportGeneration(int g, const std::vector<Done>& batch, double top10) {
    GenRow r;
    r.gen = g;
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%d/%m %H:%M", std::localtime(&now));
    r.stamp = stamp;
    r.runs = static_cast<int>(batch.size());
    r.top10 = top10;
    r.sigma = sigma;
    r.esSteps = esSteps;
    double reach = 0.0;
    for (const Done& d : batch) {
        r.home += d.r.why == "end";
        if (!IsNeural2(d.g)) continue;
        r.nnRuns++;
        reach += d.r.reached;
        if (d.r.why == "end" && (r.nnHome++ == 0 || d.r.score < r.nnBest)) r.nnBest = d.r.score;
    }
    r.nnReach = r.nnRuns ? reach / r.nnRuns : 0.0;
    // neural2's best theta kept (proving/nn2_best.txt): a generation of it with 80% of the laps home and its best
    // score; one under 50% home goes back to it (ES drifted it out of the lane twice, g58 and g65: unattended, it
    // must not keep getting worse)
    {
        static std::vector<float> bestTheta;
        static double bestNn = 1e18;
        static bool loaded = false;
        const std::string path = baseDir + "proving\\nn2_best.txt";
        if (!loaded) {
            loaded = true;
            std::ifstream f(path);
            double sc = 0.0;
            std::vector<float> w;
            float x;
            // (the best of a smaller net too: padded like theta, or a bad generation had nothing to go back to, g115)
            if (f >> sc) { while (f >> x) w.push_back(x); PadTheta(w); if (static_cast<int>(w.size()) == kNN2) { bestTheta = w; bestNn = sc; } }
        }
        const bool sized = static_cast<int>(nn2Theta.size()) == kNN2;
        if (sized && r.nnRuns >= 10) {
            const double home = static_cast<double>(r.nnHome) / r.nnRuns;
            if (home >= 0.8 && r.nnBest < bestNn) {
                bestNn = r.nnBest;
                bestTheta = nn2Theta;
                std::ostringstream o;
                o << bestNn;
                for (float w : bestTheta) o << " " << w;
                Disk::Write(path, o.str() + "\n");
                Log(Format("neural2: best theta kept (g%d: %d of %d home, best %.2f)", g, r.nnHome, r.nnRuns, r.nnBest));
            }
            else if (home < 0.5 && static_cast<int>(bestTheta.size()) == kNN2) {
                nn2Theta = bestTheta;
                SaveTheta();
                Log(Format("neural2: g%d only %d of %d home: back to the best theta (%.2f)", g, r.nnHome, r.nnRuns, bestNn));
            }
        }
    }
    if (static_cast<int>(nn2Theta.size()) == kNN2) {
        // the output layer's size: 0 = pursuit itself; how far the net has moved from its teacher
        const int out0 = kNn2L3;
        double sq = 0.0;
        for (int w = out0; w < kNN2; w++) sq += nn2Theta[w] * nn2Theta[w];
        r.nnOut = std::sqrt(sq / (kNN2 - out0));
    }
    if (!batch.empty()) {
        const Done& b = batch[0];
        r.id = b.id;
        r.mode = modes[b.g.mode];
        r.score = b.r.score;
        r.time = b.r.time;
        r.mean = b.r.mean;
        for (int i = 0; i < kNumGenes; i++) if (Uses(i, b.g.mode)) r.genes += Format("%s%s %.3g", r.genes.empty() ? "" : ", ", kGenes[i].key, b.g.g[i]);
    }
    genRows.push_back(r);
    {
        // the sectors of this generation: the best of each part and the winner's
        const Course& c0 = runners[0].road;
        secNames.clear();
        for (const auto& sc : c0.sections) secNames.push_back(sc.second);
        secBest.assign(secNames.size(), -1.0f);
        secBestId.assign(secNames.size(), std::string());
        secWin.assign(secNames.size(), -1.0f);
        for (const Done& d : batch)
            for (size_t q = 0; q < d.r.sectors.size() && q < secNames.size(); q++)
                if (d.r.sectors[q] > 0.0f && (secBest[q] < 0.0f || d.r.sectors[q] < secBest[q])) { secBest[q] = d.r.sectors[q]; secBestId[q] = d.id; }
        if (!batch.empty() && batch[0].r.why == "end")
            for (size_t q = 0; q < batch[0].r.sectors.size() && q < secNames.size(); q++) secWin[q] = batch[0].r.sectors[q];
        secGen = g;
    }
    const bool fresh = GetFileAttributesA(RowsPath().c_str()) == INVALID_FILE_ATTRIBUTES && genRows.size() == 1;
    std::string line = fresh ? "gen\tstamp\truns\thome\tid\tmode\tscore\ttime\tmean\ttop10\tsigma\tnn_runs\tnn_home\tnn_best\tnn_reach\tes_steps\tnn_out\t-\tgenes\n" : "";
    line += Format("%d\t%s\t%d\t%d\t%s\t%s\t%.3f\t%.2f\t%.4f\t%.3f\t%.4f\t%d\t%d\t%.3f\t%.0f\t%d\t%.5f\t-\t", r.gen, r.stamp.c_str(), r.runs, r.home,
        r.id.c_str(), r.mode.c_str(), r.score, r.time, r.mean, r.top10, r.sigma, r.nnRuns, r.nnHome, r.nnBest, r.nnReach, r.esSteps, r.nnOut) + r.genes + "\n";
    Disk::Write(RowsPath(), line, true);
    WriteReport();
}

// a generation of the old kind (all its cars at once): its runs into the report
static void ReportPop(int n) {
    std::vector<Done> b;
    for (int k = 0; k < n; k++) b.push_back({ pop[k], res[k], Format("%s", kTags[k]) });
    std::sort(b.begin(), b.end(), [](const Done& x, const Done& y) { return x.r.score < y.r.score; });
    double top10 = 0.0;
    const int n10 = std::min<int>(10, static_cast<int>(b.size()));
    for (int i = 0; i < n10; i++) top10 += b[i].r.score / std::max(n10, 1);
    ReportGeneration(generation, b, top10);
}

static void SetSectors(int k, const std::vector<float>& sec) { res[k].sectors = sec; }

struct EsDone { double score; std::vector<float> eps; };
static std::vector<EsDone> esBatch;
static std::vector<float> pendingEps;   // a pair's + waiting for its -
static Genome pendingGenes;             // ... and the genes it drove with: its - drives the same (the pair differs by eps alone)

static int ArchiveTournament(int mode) {
    std::vector<int> pool;
    for (int i = 0; i < static_cast<int>(archive.size()); i++) if (mode < 0 || archive[i].g.mode == mode) pool.push_back(i);
    if (pool.empty()) return -1;
    // only the best ParentTop of them (a tournament of 3 among all 500 picked, on average, a parent of the top quarter
    // of a pool full of failures: the best stayed at 138-140 s for 8 generations)
    std::sort(pool.begin(), pool.end(), [](int a, int b) { return archive[a].avg() < archive[b].avg(); });
    const size_t keep = std::max<size_t>(std::min<size_t>(pool.size(), 3), static_cast<size_t>(std::ceil(pool.size() * parentTop)));
    pool.resize(std::min(pool.size(), keep));
    int bestI = pool[Pick(static_cast<int>(pool.size()))];
    for (int t = 1; t < tournament; t++) {
        const int i = pool[Pick(static_cast<int>(pool.size()))];
        if (archive[i].avg() < archive[bestI].avg()) bestI = i;
    }
    return bestI;
}

static Genome Breed(const Genome& a, const Genome& b, int own) {
    Genome c;
    c.mode = own >= 0 ? own : U() < 0.5f ? a.mode : b.mode;
    for (int g = 0; g < kNumGenes; g++) {
        const bool ua = Uses(g, a.mode), ub = Uses(g, b.mode);
        c.g[g] = ua && !ub ? a.g[g] : ub && !ua ? b.g[g] : U() < 0.5f ? a.g[g] : b.g[g];
    }
    for (int g = 0; g < kNumGenes; g++)
        if (Uses(g, c.mode) && U() < mutRate)
            c.g[g] = std::clamp(c.g[g] + N01() * sigma * (kGenes[g].hi - kGenes[g].lo), kGenes[g].lo, kGenes[g].hi);
    return c;
}

static int forceMode = -1; // (Child: the algorithm the next brain must be, Nn2Min)
static Genome ChildOf(int k);
// The next brain for lane k; neural2 forced when the generation would end with fewer than Nn2Min of them
static Genome Child(int k) {
    static int nnGen = -1, nnBorn = 0;
    const int g = birthGen < 0 ? generation : birthGen;
    if (g != nnGen) { nnGen = g; nnBorn = 0; }
    int nn2Mode = -1;
    for (int m = 0; m < static_cast<int>(modes.size()); m++) if (modes[m] == "neural2") nn2Mode = m;
    const int left = numRunners - birthCount; // births left in this generation, this one too
    forceMode = nn2Mode >= 0 && nnBorn < nn2Min && left <= nn2Min - nnBorn ? nn2Mode : -1;
    Genome c = ChildOf(k);
    forceMode = -1;
    if (IsNeural2(c)) nnBorn++;
    return c;
}
static Genome ChildOf(int k) {
    if (archive.empty()) {
        Genome c = pop[k];
        for (int g = 0; g < kNumGenes; g++)
            if (Uses(g, c.mode) && U() < mutRate)
                c.g[g] = std::clamp(c.g[g] + N01() * sigma * (kGenes[g].hi - kGenes[g].lo), kGenes[g].lo, kGenes[g].hi);
        c.archiveId = -1;
        return c;
    }
    // the algorithm furthest under its quota (the islands)
    const int nm = static_cast<int>(modes.size());
    std::vector<int> count(nm, 0);
    for (int j = 0; j < numRunners; j++) if (j != k) count[std::clamp(pop[j].mode, 0, nm - 1)]++;
    int own = -1, lack = 0;
    for (int m = 0; m < nm; m++) {
        const int quota = modes[m] == "neural2" ? std::max(minPerMode, nn2Cars) : minPerMode;
        if (quota - count[m] > lack) { lack = quota - count[m]; own = m; }
    }
    if (forceMode >= 0) own = forceMode;
    std::vector<int> order(archive.size());
    for (int i = 0; i < static_cast<int>(order.size()); i++) order[i] = i;
    std::sort(order.begin(), order.end(), [](int a, int b) { return archive[a].avg() < archive[b].avg(); });
    if (U() < reevalChance) {
        // an elite again (of the algorithm under its quota, if one is): of the Elite best, the one run the fewest times,
        // so a lucky single run is checked before it breeds much (own was almost never -1: no elite had been re-run)
        int pickI = -1, seen = 0;
        for (int i : order) {
            if (own >= 0 && archive[i].g.mode != own) continue;
            if (pickI < 0 || archive[i].runs < archive[pickI].runs) pickI = i;
            if (++seen >= std::max(elite, 1)) break;
        }
        if (pickI >= 0) {
            Genome c = archive[pickI].g;
            c.archiveId = archive[pickI].id;
            return c;
        }
    }
    const int ia = ArchiveTournament(own);
    if (ia < 0) {
        // an algorithm with no brain in the archive: the best's shared genes, its own at the ini's values
        Genome c = archive[order[0]].g;
        const Genome ini = FromSettings(base);
        for (int g = 0; g < kNumGenes; g++) if (Uses(g, own) && !Uses(g, c.mode)) c.g[g] = ini.g[g];
        c.mode = own;
        c.label.clear();
        c.archiveId = -1;
        return c;
    }
    int ib = ArchiveTournament(own >= 0 && U() < 0.5f ? own : -1);
    if (ib < 0) ib = ia;
    Genome c = Breed(archive[ia].g, archive[ib].g, own);
    c.archiveId = -1;
    return c;
}

// neural2: theta and a perturbation of a +- pair onto a new brain on lane k
static void AssignNn2Lane(Genome& g, int k) {
    nn2Eps[k].clear();
    if (!IsNeural2(g)) return;
    LoadTheta();
    InitTheta();
    if (static_cast<int>(nn2Theta.size()) != kNN2) return;
    if (!pendingEps.empty()) {
        nn2Eps[k] = pendingEps;
        for (float& e : nn2Eps[k]) e = -e;
        pendingEps.clear();
        // (the same genes as its +: bred anew, the pair's difference was as much the genes' (Kp, BrakeG, the ABS...) as eps's)
        for (int q = 0; q < kNumGenes; q++) g.g[q] = pendingGenes.g[q];
    }
    else { nn2Eps[k].resize(kNN2); for (float& e : nn2Eps[k]) e = N01(); pendingEps = nn2Eps[k]; pendingGenes = g; }
    const float sg = nn2Es ? nn2Sigma : nn2Noise;
    for (int w = 0; w < kNN2; w++) g.nn[w] = nn2Theta[w] + sg * nn2Eps[k][w];
}

static void StepThetaBatch() {
    if (static_cast<int>(nn2Theta.size()) != kNN2 || esBatch.size() < 2) { esBatch.clear(); return; }
    std::sort(esBatch.begin(), esBatch.end(), [](const EsDone& a, const EsDone& b) { return a.score > b.score; }); // worst first
    const int m = static_cast<int>(esBatch.size());
    std::vector<float> step(kNN2, 0.0f);
    for (int r = 0; r < m; r++) {
        const float u = static_cast<float>(r) / (m - 1) - 0.5f;
        for (int w = 0; w < kNN2; w++) step[w] += u * esBatch[r].eps[w];
    }
    for (int w = 0; w < kNN2; w++) nn2Theta[w] += nn2Alpha / (m * nn2Sigma) * step[w] - 0.0005f * nn2Alpha * nn2Theta[w];
    SaveTheta();
    esSteps++;
    Log(Format("neural2: ES step from %d runs (best of them %.1f)", m, esBatch.back().score));
    esBatch.clear();
}

static void ApplyOne(int k) {
    Settings s = ToSettings(pop[k]);
    if (IsNeural(pop[k])) s.nn.assign(pop[k].nn, pop[k].nn + kNN);
    if (IsNeural2(pop[k]) && static_cast<int>(nn2Theta.size()) == kNN2) s.nn.assign(pop[k].nn, pop[k].nn + kNN2);
    s.driver = base.driver;
    CfgOf(k) = s;
}

// A "generation" of the steady state: its runs logged as one (history, the best ever, sigma, the state file)
static void EndBatch(int g) {
    Perf::Scope perf(Perf::ENDBATCH);
    std::vector<Done>& batch = batches[g];
    std::sort(batch.begin(), batch.end(), [](const Done& a, const Done& b) { return a.r.score < b.r.score; });
    int bestA = 0;
    for (int i = 1; i < static_cast<int>(archive.size()); i++) if (archive[i].avg() < archive[bestA].avg()) bestA = i;
    const Entry& top = archive[bestA];
    // the generation's own runs: its best and the mean of its 10 best (what the HUD's chart shows)
    double top10 = 0.0;
    const int n10 = std::min<int>(10, static_cast<int>(batch.size()));
    for (int i = 0; i < n10; i++) top10 += batch[i].r.score / std::max(n10, 1);
    history.push_back(batch.empty() ? 0.0 : batch[0].r.score);
    history10.push_back(top10);
    if (top.avg() < bestScore) {
        bestScore = top.avg();
        best = top.g;
        bestRes = top.last;
        bestGen = g;
        WriteBest();
    }
    Log(Format("\n=== Generation %d (steady state, %d runs): best run %s %.2f, archive best %.2f (%d runs), best ever %.2f (gen %d)", g,
        static_cast<int>(batch.size()), batch[0].id.c_str(), batch[0].r.score, top.avg(), top.runs, bestScore, bestGen));
    const std::string histPath = baseDir + "proving\\ga_history.csv";
    std::string header = "stamp,generation,rank,car,mode,score,result,time_s,mean_m,rms_m,max_m,out_s,reached_m,sigma";
    for (const auto& gd : kGenes) header += std::string(",") + gd.key;
    header += ",label,avg_score,runs,deficit_ms,min_margin_m";
    char stamp[32];
    std::time_t now = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&now));
    static bool histChecked = false; // (the file's header looked at once: the disk thread may still be appending)
    if (!histChecked) {
        std::ifstream old(histPath);
        std::string first;
        if (old && std::getline(old, first) && first != header) {
            old.close();
            MoveFileA(histPath.c_str(), (baseDir + "proving\\ga_history_" + stamp + ".csv").c_str());
        }
    }
    const bool fresh = !histChecked && GetFileAttributesA(histPath.c_str()) == INVALID_FILE_ATTRIBUTES;
    histChecked = true;
    std::ostringstream h;
    if (fresh) h << header << "\n";
    for (int r = 0; r < static_cast<int>(batch.size()); r++) {
        const Done& d = batch[r];
        const Result& x = d.r;
        if (r < 10)
            Log(Format("  %2d. %-7s %-9s score %8.2f  %-7s %6.1f s  room min %.2f m  out %5.1f s  | %s", r + 1, d.id.c_str(), modes[d.g.mode].c_str(),
                x.score, x.why.c_str(), x.time, x.minMargin < 1e8 ? x.minMargin : 0.0, x.out, ShortText(d.g).c_str()));
        h << Format("%s,%d,%d,%s,%s,%.3f,%s,%.2f,%.3f,%.3f,%.3f,%.2f,%.0f,%.4f,", stamp, g, r + 1, d.id.c_str(), modes[d.g.mode].c_str(),
            x.score, x.why.c_str(), x.time, x.mean, x.rms, x.maxE, x.out, x.reached, sigma) << GenesText(d.g, ",") << "," << d.g.label
          << Format(",%.3f,%d,%.3f,%.3f", x.score, 1, x.deficit, x.minMargin < 1e8 ? x.minMargin : 0.0) << "\n";
    }
    Disk::Write(histPath, h.str(), true);
    ReportGeneration(g, batch, top10); // (before the erase: batch is a reference into batches)
    batches.erase(g);
    generation = std::max(generation, g + 1);
    // sigma by progress: the archive's 10 best better than before -> narrower (refine); 3 generations without -> wider
    // (out of a local peak); it only shrank before, to 0.025, and the GA stood still
    std::vector<double> av;
    for (const auto& e : archive) av.push_back(e.avg());
    std::sort(av.begin(), av.end());
    double a10 = 0.0;
    const int na = std::min<int>(10, static_cast<int>(av.size()));
    for (int i = 0; i < na; i++) a10 += av[i] / std::max(na, 1);
    if (a10 < stallRef - 0.3) { stallRef = a10; stall = 0; sigma = std::max(sigma * 0.9f, sigmaMin); }
    else if (++stall >= 3) {
        sigma = std::min(sigma * 1.5f, sigmaMax);
        stall = 0;
        stallRef = a10;
        Log(Format("GA: no progress in 3 generations (archive top 10 %.2f): sigma up to %.3f", a10, sigma));
    }
    Save();
    CloseTrace(g);
}

// A run ended on lane k: its score into the archive, the next brain onto the lane
static void Replace(int k) {
    Genome g = pop[k];
    const Result r = res[k];
    res[k] = Result();
    const std::string id = Format("%s.%d", kTags[k], runners[k].runId);
    if (r.why == "end" && (bestLapTime < 0.0f || r.time < bestLapTime)) bestLapTime = static_cast<float>(r.time);
    bool found = false;
    if (g.archiveId >= 0)
        for (auto& e : archive) if (e.id == g.archiveId) { e.sum += r.score; e.runs++; e.last = r; found = true; break; }
    if (!found) {
        Entry e;
        e.g = g;
        e.id = nextId++;
        e.g.archiveId = e.id;
        e.sum = r.score;
        e.runs = 1;
        e.last = r;
        archive.push_back(e);
        if (static_cast<int>(archive.size()) > numRunners) {
            int worst = 0;
            for (int i = 1; i < static_cast<int>(archive.size()); i++) if (archive[i].avg() > archive[worst].avg()) worst = i;
            archive.erase(archive.begin() + worst);
        }
    }
    const int bg = runners[k].bornGen;
    batches[bg].push_back({ g, r, id });
    if (IsNeural2(g) && nn2Es && !nn2Eps[k].empty()) {
        esBatch.push_back({ r.score, nn2Eps[k] });
        if (static_cast<int>(esBatch.size()) >= nn2Cars) StepThetaBatch();
    }
    if (static_cast<int>(batches[bg].size()) >= numRunners) EndBatch(bg);
    Genome c = Child(k);
    AssignNn2Lane(c, k);
    pop[k] = c;
    ApplyOne(k);
}
} // namespace GA

static CVehicle* MakeCar(int model) {
    const CVector2D start = course.p.empty() ? CVector2D(cfg.origin.x, cfg.origin.y) : course.p[0];
    Command<COMMAND_CLEAR_AREA>(start.x, start.y, R->baseZ, 30.0f, 0);
    Command<COMMAND_CREATE_CAR>(model, start.x, start.y, R->baseZ + 1.0f, &carHandle);
    CVehicle* v = CPools::GetVehicle(carHandle);
    if (!v) return nullptr;
    Command<COMMAND_SET_CAR_PROOFS>(carHandle, 1, 1, 1, 1, 1);
    CPlayerPed* player = FindPlayerPed();
    const int ph = CPools::GetPedRef(player);
    if (R != runners) {
        Command<COMMAND_CREATE_RANDOM_CHAR_AS_DRIVER>(carHandle, &driverHandle);
        Command<COMMAND_SET_CAR_CRUISE_SPEED>(carHandle, 0.0f);
    }
    else if (cfg.driver == "player" || cfg.driver == "cjai" || cfg.driver == "human") {
        Command<COMMAND_WARP_CHAR_INTO_CAR>(ph, carHandle);
    }
    else {
        Command<COMMAND_CREATE_RANDOM_CHAR_AS_DRIVER>(carHandle, &driverHandle);
        Command<COMMAND_WARP_CHAR_INTO_CAR_AS_PASSENGER>(ph, carHandle, 0);
        Command<COMMAND_SET_CAR_CRUISE_SPEED>(carHandle, 0.0f);
    }
    v->m_nCreatedBy = MISSION_VEHICLE; // never removed; the mod's own driving leaves it alone (no forceDrive)
    CVector mn, mx;
    GetEntityDimensions(v, mn, mx);
    wheelBase = GetWheelBase(v, mn, mx);
    R->bodyFront = mx.y;
    R->bodyRear = mn.y;
    R->bodyHalfW = std::max(-mn.x, mx.x);
    rearAxleY = -0.5f * wheelBase;
    if (CColModel* cm = v->GetColModel()) if (cm->m_pColData && cm->m_pColData->m_nNumLines >= 4 && cm->m_pColData->m_pLines) {
        float rear = 1000.0f;
        for (int w = 0; w < 4; w++) rear = std::min(rear, cm->m_pColData->m_pLines[w].m_vecStart.y);
        rearAxleY = rear;
    }
    const tHandlingData* h = v->m_pHandlingData;
    {
        // Every wheel cancels up to adhesive x tractionMult / 1000 of game speed per frame step (x its traction bias, 2
        // per axle in all): 4 wheels -> 4 adh tm / 1000 x 50 x 50 m/s2 = 10 adh tm m/s2, whatever the load (AI cars:
        // the load, wet and off-road factors are the player's only)
        CColPoint cp{};
        cp.m_nSurfaceTypeA = SURFACE_WHEELBASE;
        cp.m_nSurfaceTypeB = SURFACE_TARMAC;
        const float adhesive = reinterpret_cast<SurfaceInfos_c*>(0xB79538)->GetAdhesiveLimit(&cp);
        aiGripG = 10.0f * adhesive * h->m_fTractionMultiplier / 9.81f;
        aiAdhesive = adhesive;
        if (R == runners) Log(Format("  tarmac adhesive limit %.3f -> AI grip %.2f g", adhesive, aiGripG));
    }
    if (R == runners) Log(Format("\nModel %d: mass %.0f kg, turn mass %.0f, traction mult %.2f loss %.2f bias %.2f, steering lock %.1f deg, "
        "brake %.4f bias %.2f, susp force %.2f bias %.2f, CoM (%.2f, %.2f, %.2f), drive %c, wheelbase %.2f m",
        model, h->m_fMass, h->m_fTurnMass, h->m_fTractionMultiplier, h->m_fTractionLoss, h->m_fTractionBias, h->m_fSteeringLock,
        h->m_fBrakeDeceleration, h->m_fBrakeBias, h->m_fSuspensionForceLevel, h->m_fSuspensionBiasBetweenFrontAndRear,
        h->m_vecCentreOfMass.x, h->m_vecCentreOfMass.y, h->m_vecCentreOfMass.z, h->m_transmissionData.m_nDriveType, wheelBase));
    return v;
}

static void PlaceCar(CVehicle* v, float kmh);
// The current lane (R) ready for a new run: its state cleared, the car at the start
static void ResetLane() {
    runT = 0.0f;
    traceNext[R->index] = 0.0f;
    R->runId = ++GA::runIds;
    R->nnHeld = false; // (a new brain: not the last one's outputs)
    R->nnLine = R->nnTc = R->nnAbs = R->nnHb = R->nnLook = R->nnAgg = 0.0f;
    R->profileTried = false;
    R->secT.clear();
    for (float& x : R->nnAct) x = 0.0f;
    R->nnPedal = R->teacherSteer = R->teacherPedal = 0.0f; // (the last run's, read on the first frame before)
    R->bornGen = GA::on && GA::steady ? GA::NextBirth() : GA::generation;
    nearIdx = 0;
    steerOut = steerCmd = 0.0f;
    stats = Stats();
    abortWhy.clear();
    trail.clear();
    havePrevVel = false;
    absScale = tcScale = 1.0f;
    R->finished = false;
    R->result.clear();
    R->endTime = R->sMax = R->slowTime = R->progressS = R->progressT = 0.0f;
    for (float& m : R->learned) m = 1.0f;
    for (float& m : R->tcBand) m = 1.0f;
    for (float& b : R->bandBase) b = NAN;
    R->absLearn = 1.0f;
    R->absBase = NAN;
    R->curveTrial = R->absTrial = Trial();
    R->trials = R->kept = 0;
    R->marginNow = 99.0f;
    PlaceCar(car, curTest->startSpeed >= 0.0f ? curTest->startSpeed : curTest->speeds[speedIdx]);
    brakeDone = false;
}

static void PlaceCar(CVehicle* v, float kmh) {
    const CVector2D p0 = course.p[0];
    const float head = course.head[0];
    bool found = false;
    float z = FindGroundZ({ p0.x, p0.y, R->baseZ + 3.0f }, &found);
    if (!found) z = R->baseZ;
    Command<COMMAND_SET_CAR_COORDINATES>(carHandle, p0.x, p0.y, z + 0.5f);
    Command<COMMAND_SET_CAR_HEADING>(carHandle, head * 180.0f / 3.14159265f - 90.0f);
    v->m_vecMoveSpeed = CVector(std::cos(head), std::sin(head), 0.0f) * (kmh / 3.6f / GAME_SPEED_TO_MPS);
    v->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f);
    v->m_fSteerAngle = 0.0f;
}

// --- Trace (tools/ga_review.py, the viewer) --------------------------------------------------------------------------
// Every car every 0.25 s in the course frame of its own lane (x along the test's heading, y to its left: all the lanes
// on top of each other), one file per generation in scripts\proving\ga_trace\ (with the GA; else in the session's
// folder), and the course itself once (course.csv, sections.csv)
static CVector2D ToLane(const CVector2D& world) {
    const TestDef& d = *curTest;
    const float a = (d.hasStart ? d.startHeading : cfg.headingDeg) * 3.14159265f / 180.0f;
    const CVector2D dir(std::cos(a), std::sin(a)), left(-std::sin(a), std::cos(a));
    const CVector2D o = (d.hasStart ? d.start : CVector2D(cfg.origin.x, cfg.origin.y)) + CVector2D(std::sin(a), -std::cos(a)) * R->laneShift;
    return CVector2D(CVector2D::Dot(world - o, dir), CVector2D::Dot(world - o, left));
}

static void TraceHeader(std::ostream& f) {
    f << "generation,car,mode,t,x,y,speed_kmh,target_kmh,offset,steer,gas,brake,s,lat_g,slip,abs,tc,margin";
    for (int i = 0; i < kNnIn; i++) f << ",in" << i;
    for (int i = 0; i < kNn2In; i++) f << ",n" << i;
    f << ",teacher_steer,teacher_pedal\n";
}
// steady state: the file of generation g (begun when its first brain drives, ended when its last run ended)
static std::string TracePath(int g) { return baseDir + "proving\\ga_trace\\" + Format("gen_%04d.csv", g); }
static void TraceAppend(int g, const std::string& row) {
    TraceBuf& b = traceGen[g];
    if (!b.started && b.data.empty()) {
        CreateDirectoryA((baseDir + "proving\\ga_trace").c_str(), nullptr);
        std::ostringstream h;
        TraceHeader(h);
        b.data = h.str();
    }
    b.data += row;
    if (b.data.size() > (1u << 20)) { Disk::Write(TracePath(g), std::move(b.data), b.started); b.started = true; b.data.clear(); }
}
static void CloseTrace(int g) {
    auto it = traceGen.find(g);
    if (it == traceGen.end()) return;
    if (!it->second.data.empty()) Disk::Write(TracePath(g), std::move(it->second.data), it->second.started);
    Disk::Close(TracePath(g));
    traceGen.erase(it);
}

static void OpenTrace() {
    const std::string dir = GA::on ? baseDir + "proving\\ga_trace\\" : runDir;
    CreateDirectoryA(dir.c_str(), nullptr);
    if (trace.is_open()) trace.close();
    if (!(GA::on && GA::steady)) {
        trace.open(dir + (GA::on ? Format("gen_%04d.csv", GA::generation) : std::string("trace.csv")), std::ios::trunc);
        TraceHeader(trace);
        for (float& t : traceNext) t = 0.0f;
    }
    if (courseWritten || !curTest) return;
    courseWritten = true;
    Runner* keep = R;
    R = runners;
    std::ofstream c(dir + "course.csv", std::ios::trunc);
    c << "x,y,s,curv,vcap_kmh,vsud_kmh,wind,half_width\n";
    for (size_t i = 0; i < R->road.p.size(); i += 2) {
        const CVector2D q = ToLane(R->road.p[i]);
        c << Format("%.2f,%.2f,%.1f,%.5f,%.0f,%.0f,%.1f,%.2f\n", q.x, q.y, R->road.s[i], R->road.curv[i],
            R->road.vcap[i] < 1e8f ? R->road.vcap[i] * 3.6f : -1.0f, R->road.vsud[i] < 1e8f ? R->road.vsud[i] * 3.6f : -1.0f, R->road.wind[i],
            R->road.halfWidth);
    }
    std::ofstream s(dir + "sections.csv", std::ios::trunc);
    s << "s,name\n";
    for (const auto& p : R->road.sections) s << Format("%.1f,%s\n", p.first, p.second.c_str());
    std::ofstream k(dir + "cones.csv", std::ios::trunc);
    k << "x,y\n";
    for (const auto& q : R->road.cones) { const CVector2D l = ToLane(q); k << Format("%.2f,%.2f\n", l.x, l.y); }
    R = keep;
}

static void Trace(CVehicle* v) {
    const bool perGen = GA::on && GA::steady;
    if ((!perGen && !trace.is_open()) || runT < traceNext[R->index] || (GA::on && !GA::traceOn)) return;
    Perf::Scope perf(Perf::TRACE);
    std::string row;
    traceNext[R->index] = runT + (GA::traceEvery > 0.0f ? GA::traceEvery : numRunners > 100 ? 0.5f : 0.25f); // (2 a second above 100 cars: the file grows with them)
    const CVector2D q = ToLane(v->GetPosition().To2D());
    row = Format("%d,%s,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.3f,%.2f,%.2f,%.1f,%.2f,%.3f,%d,%d,%.2f", perGen ? R->bornGen : GA::on ? GA::generation : runCount,
        GA::steady ? Format("%s.%d", R->tag, R->runId).c_str() : R->tag, human && R == runners ? "human" : cfg.mode.c_str(), runT, q.x, q.y, R->speedNow, targetKmh, offsetNow, v->m_fSteerAngle, gasOut, brakeOut, R->sMax,
        R->latGNow, R->slipNow, R->absOn ? 1 : 0, R->tcOn ? 1 : 0, course.halfWidth > 0.0f ? R->marginNow : 0.0f);
    float in[kNnIn];
    NeuralInputs(v, in);
    std::string tail;
    for (int i = 0; i < kNnIn; i++) tail += Format(",%.3f", in[i]);
    // (neural2's sensors only for its teachers and itself: they make the file 3x bigger)
    const bool n2 = cfg.mode == "neural2" || cfg.mode == "pursuit" || cfg.mode == "rpp" || cfg.mode == "blend";
    if (n2) {
        // (the inputs as the net saw them before this frame's command, not taken again now: n49..n51 would be the
        // command itself. teacher_steer is pursuit's wheel for every one of them, teacher_pedal the planner's pedal
        // before the ABS and the TC; the car's own wheel and pedals are the steer, gas, brake columns)
        for (int i = 0; i < kNn2In; i++) tail += Format(",%.3f", R->nnIn[i]);
        tail += Format(",%.4f,%.3f", R->teacherSteer, R->teacherPedal);
    }
    else tail += std::string(kNn2In + 2, ',');
    row += tail;
    row += "\n";
    if (perGen) TraceAppend(R->bornGen, row);
    else trace << row;
}

// --- Camera ----------------------------------------------------------------------------------------------------------
// Auto (Home): the standout of the newest generation on the road: of the cars of the youngest generation driving (past
// 300 m), the one heading for the best time; held until its run ends (and 2.5 s more: home or out), then the best of
// that same generation still driving, until it has none. PageUp / PageDown: those cars one by one, best first.
// (It was on the leader: it got home, the lane started a new brain, the camera jumped to the next leader.) Car A is
// CJ's: the game's own camera
static unsigned int camSince = 0, camEndAt = 0;
static int camRun = -1;        // the run followed (a lane's next brain is another run)
static int camGen = -1;        // its generation (the HUD)

// the lap time car k is heading for (s; 1e9: too early to tell, or not running)
static float PaceOf(int k) {
    Runner* keep = R;
    R = &runners[k];
    const float p = R->result.empty() && R->sMax > 300.0f && course.length > 1.0f ? runT * course.length / R->sMax : 1e9f;
    R = keep;
    return p;
}
// the cars of a generation driving (gen < 0: the youngest one), best pace first
static std::vector<int> CamCandidates(int gen = -1) {
    int young = gen;
    if (young < 0) for (int k = 0; k < numRunners; k++) if (runners[k].result.empty() && runners[k].sMax > 300.0f) young = std::max(young, runners[k].bornGen);
    std::vector<std::pair<float, int>> c;
    for (int k = 0; k < numRunners; k++)
        if (runners[k].bornGen == young) { const float pc = PaceOf(k); if (pc < 1e8f) c.push_back({ pc, k }); }
    std::sort(c.begin(), c.end());
    std::vector<int> o;
    for (const auto& e : c) o.push_back(e.second);
    return o;
}

// The order of the cars: the finished ones by time, then the ones running by how far they got, then the rest
static std::vector<int> Standings() {
    std::vector<int> o;
    for (int k = 0; k < numRunners; k++) o.push_back(k);
    auto rank = [](const Runner& r) { return r.result.empty() ? 1 : r.result == "end" ? 0 : 2; };
    std::stable_sort(o.begin(), o.end(), [&](int a, int b) {
        const Runner &x = runners[a], &y = runners[b];
        if (rank(x) != rank(y)) return rank(x) < rank(y);
        if (rank(x) == 0) return GA::on ? GA::res[x.index].score < GA::res[y.index].score : x.endTime < y.endTime;
        return x.sMax > y.sMax;
    });
    return o;
}

static void PointCamera(int k) {
    if (k <= 0 || !CarOf(k)) Command<COMMAND_RESTORE_CAMERA_JUMPCUT>();
    else Command<COMMAND_POINT_CAMERA_AT_CAR>(HandleOf(k), 18, 2);
    camOn = CarOf(k) ? k : 0;
}

static void FollowCar(int k, unsigned int now) {
    if (k != camOn) PointCamera(k);
    camRun = runners[k].runId;
    camGen = runners[k].bornGen;
    camSince = now;
    camEndAt = 0;
}

static void UpdateCamera(unsigned int now) {
    if (human) { if (camOn) PointCamera(0); return; } // (the human's own camera)
    static bool prevUp = false, prevDown = false, prevHome = false;
    const bool up = KeyPressed(VK_PRIOR) != 0, down = KeyPressed(VK_NEXT) != 0, home = KeyPressed(VK_HOME) != 0;
    const bool stepUp = up && !prevUp, stepDown = down && !prevDown, goHome = home && !prevHome;
    prevUp = up; prevDown = down; prevHome = home;
    if (goHome) { camPick = -1; camRun = -1; }
    if (stepUp || stepDown) {
        const std::vector<int> c = CamCandidates();
        if (!c.empty()) {
            const int n = static_cast<int>(c.size());
            int at = static_cast<int>(std::find(c.begin(), c.end(), camOn) - c.begin());
            at = at >= n ? 0 : (at + (stepDown ? 1 : n - 1)) % n;
            camPick = c[at];
            FollowCar(c[at], now);
        }
        return;
    }
    // the run followed: still on, or ended a moment ago
    const Runner& f = runners[std::clamp(camOn, 0, numRunners - 1)];
    bool keep = camRun >= 0 && f.runId == camRun;
    if (keep && !f.result.empty()) {
        if (!camEndAt) camEndAt = now;
        if (now - camEndAt > 2500) keep = false;
    }
    // picked by hand: held to the end of its run
    if (keep && camPick >= 0) return;
    camPick = -1;
    // auto: the first car, the one furthest on of those still driving (the user: "sempre mostre o primeiro carro"); a
    // new leader only 20 m ahead of the one followed (no jumping between two side by side); the leader home or out:
    // 2.5 s on it, then the next one first
    int lead = -1;
    for (int k = 0; k < numRunners; k++) if (runners[k].result.empty() && (lead < 0 || runners[k].sMax > runners[lead].sMax)) lead = k;
    if (lead < 0) return;
    const bool onRoad = keep && f.result.empty();
    if (!keep || (onRoad && lead != camOn && runners[lead].sMax > f.sMax + 20.0f)) FollowCar(lead, now);
}

// --- Per frame -------------------------------------------------------------------------------------------------------

void Init(const std::string& moduleDir) {
    baseDir = moduleDir;
    iniPath = moduleDir + "RealTrafficFix.proving.ini";
    if (GetFileAttributesA(iniPath.c_str()) == INVALID_FILE_ATTRIBUTES) {
        // (the AI's ABS goes through the same call)
        if (cfgAiAbs) InstallHooks();
        return;
    }
    enabled = true;
    IniChanged();
    ReadSettings();
    InstallHooks();
}

void Update() {
    if (!enabled) return;
    Perf::Frame(state == RUNNING);
    Perf::Scope perf(Perf::UPDATE);
    CPlayerPed* player = FindPlayerPed();
    if (!player) return;
    const unsigned int now = Now();

    // nothing else about: no traffic, no people, midday, dry
    Command<COMMAND_SET_PED_DENSITY_MULTIPLIER>(0.0f);
    Command<COMMAND_SET_CAR_DENSITY_MULTIPLIER>(0.0f);
    // the game's HUD and radar under ours
    Command<COMMAND_DISPLAY_HUD>(0);
    Command<COMMAND_DISPLAY_RADAR>(0);
    cfgAmbientSpawn = false;
    CCarCtrl::MaxNumberOfCarsInUse = 0;
    static unsigned int lastClear = 0;
    if (now - lastClear > 2000 && state != WAIT_GAME) {
        lastClear = now;
        Command<COMMAND_SET_TIME_OF_DAY>(13, 0);
        Command<COMMAND_FORCE_WEATHER_NOW>(0);
        for (int i = 0; i < CPools::ms_pVehiclePool->m_nSize; i++) {
            CVehicle* v = CPools::ms_pVehiclePool->GetAt(i);
            if (v && v->m_nCreatedBy == RANDOM_VEHICLE && v != player->m_pVehicle) spawnRemovals.push_back(v);
        }
        for (int i = 0; i < CPools::ms_pPedPool->m_nSize; i++) {
            CPed* p = CPools::ms_pPedPool->GetAt(i);
            if (p && p != player && !p->bInVehicle && p->m_nCreatedBy == 1) Command<COMMAND_DELETE_CHAR>(CPools::GetPedRef(p));
        }
    }

    if (GetTickCount() - lastIniCheck > 500) {
        lastIniCheck = GetTickCount();
        if (IniChanged() && state != WAIT_GAME) {
            Log("--- ini changed: starting over");
            Restart();
            return;
        }
    }

    switch (state) {
    case WAIT_GAME:
        if (now < 8000) return;
        Command<COMMAND_SET_MAX_WANTED_LEVEL>(0);
        Command<COMMAND_SET_EVERYONE_IGNORE_PLAYER>(0, 1);
        Command<COMMAND_SET_CHAR_PROOFS>(CPools::GetPedRef(player), 1, 1, 1, 1, 1);
        if (player->m_pVehicle) Command<COMMAND_WARP_CHAR_FROM_CAR_TO_COORD>(CPools::GetPedRef(player), cfg.origin.x - 30.0f, cfg.origin.y - 30.0f, cfg.origin.z + 1.0f);
        else Command<COMMAND_SET_CHAR_COORDINATES>(CPools::GetPedRef(player), cfg.origin.x - 30.0f, cfg.origin.y - 30.0f, cfg.origin.z + 1.0f);
        Command<COMMAND_LOAD_SCENE>(cfg.origin.x, cfg.origin.y, cfg.origin.z);
        Restart();
        {
            // The platform's model (tools/make_gridmap.py): defined, with its collision, and placed in the world?
            const int gridModel = 19400;
            CBaseModelInfo* mi = CModelInfo::GetModelInfo(gridModel);
            CColModel* col = mi ? mi->m_pColModel : nullptr;
            int placed = 0;
            CVector first;
            for (int i = 0; i < CPools::ms_pBuildingPool->m_nSize; i++) {
                CBuilding* b = CPools::ms_pBuildingPool->GetAt(i);
                if (b && b->m_nModelIndex == gridModel) { if (!placed) first = b->GetPosition(); placed++; }
            }
            int dummies = 0;
            for (int i = 0; i < CPools::ms_pDummyPool->m_nSize; i++) {
                CDummy* d = CPools::ms_pDummyPool->GetAt(i);
                if (d && d->m_nModelIndex == gridModel) dummies++;
            }
            Log(Format("Grid model %d: info %p, col model %p (data %p, triangles %d), %d buildings (first at %.0f, %.0f, %.0f), %d dummies",
                gridModel, mi, col, col ? col->m_pColData : nullptr, col && col->m_pColData ? col->m_pColData->m_nNumTriangles : -1,
                placed, first.x, first.y, first.z, dummies));
        }
        stateTime = now;
        return;

    case NEXT_CAR:
        forRunners { RemoveCar(); RemoveObstacles(); }
        R = runners;
        carIdx++;
        testIdx = 0;
        speedIdx = 0;
        if (carIdx >= static_cast<int>(cfg.cars.size()) || cfg.tests.empty()) {
            Log("\nDone.");
            state = DONE;
            if (cfg.exitWhenDone) ExitProcess(0);
            return;
        }
        curModel = cfg.cars[carIdx];
        Command<COMMAND_REQUEST_MODEL>(curModel);
        Command<COMMAND_LOAD_ALL_MODELS_NOW>();
        state = WAIT_MODEL;
        stateTime = now;
        return;

    case WAIT_MODEL:
        if (!Command<COMMAND_HAS_MODEL_LOADED>(curModel)) {
            if (now - stateTime > 10000) { Log(Format("model %d did not load", curModel)); state = NEXT_CAR; }
            return;
        }
        curTest = &cfg.defs[cfg.tests[testIdx]];
        forRunners {
            BuildCourse(*curTest);
            car = MakeCar(curModel);
            if (!car) { Log(Format("model %d: could not create the car", curModel)); state = NEXT_CAR; return; }
            SpawnObstacles();
        }
        R = runners;
        state = START_RUN;
        stateTime = now;
        return;

    case START_RUN: {
        curTest = &CfgOf(0).defs[CfgOf(0).tests[testIdx]];
        bool ready = true;
        forRunners {
        if (!car || !CPools::ms_pVehiclePool->IsObjectValid(car)) { state = NEXT_CAR; return; }
        BuildCourse(*curTest);
        // the collision where it starts must be there first (it fell through the ground teleported 300 m away)
        {
            const CVector2D p0 = course.p[0];
            CColStore::RequestCollision(CVector(p0.x, p0.y, R->baseZ), 0);
            CColStore::AddCollisionNeededAtPosn(CVector(p0.x, p0.y, R->baseZ));
            CColPoint cp;
            CEntity* hit = nullptr;
            const bool ground = CWorld::ProcessVerticalLine(CVector(p0.x, p0.y, R->baseZ + 5.0f), R->baseZ - 10.0f, cp, hit,
                true, false, false, false, true, false, nullptr);
            if (!ground) {
                if (now - stateTime > 8000) { Log("no ground at the start of this course: check Origin / Start"); state = NEXT_CAR; }
                else if (now - stateTime > 1500 && now - stateTime < 1600) Command<COMMAND_LOAD_SCENE>(p0.x, p0.y, R->baseZ);
                ready = false;
            }
        }
        }
        if (!ready || now - stateTime < 500) return; // (and let the cars settle after being made/moved)
        runCount++;
        firstHome = -1.0f;
        forRunners {
            ResetLane();
            StartCsv();
        }
        R = runners;
        if (!trace.is_open()) OpenTrace();
        state = RUNNING;
        stateTime = now;
        return;
    }

    case RUNNING: {
        const float dt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
        if (CTimer::m_UserPause || CTimer::m_CodePause) return;
        UpdateCamera(now);
        clockT += dt;
        bool allDone = true;
        forRunners {
        if (!car || !CPools::ms_pVehiclePool->IsObjectValid(car)) { Log("car lost"); state = NEXT_CAR; return; }
        // CJ at the wheel of a car driven by the AI's code: the game's STATUS_PHYSICS (AI grip, no player steering
        // limit, CCarCtrl::SteerAICarWithPhysics -> Control) before every physics frame
        // (and the other cars: far from the player the game would put them on its simple, rail-like movement)
        if (R != runners || cfg.driver == "cjai") car->m_nStatus = STATUS_PHYSICS;
        if (R->finished) {
            if (R->result != "end") { car->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f); car->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f); }
            if (GA::on && GA::steady && clockT - R->doneAt > 0.5f) {
                // steady state: this run into the archive, a new brain on the lane, from the start again
                Perf::Scope perf(Perf::REPLACE);
                GA::Replace(R->index);
                // (curTest points into car A's settings, which a new brain on lane A replaces: the old pointer read freed
                // memory, d.speeds empty -> null, and the game crashed; the crash logs of 15:31 and 15:36)
                curTest = &CfgOf(0).defs[CfgOf(0).tests[testIdx]];
                BuildCourse(*curTest);
                SpawnObstacles();
                car->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f);
                car->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f);
                // (a rolled car is not made again: PlaceCar's heading puts it back on its wheels; made again, with a new
                // driver, each took up to 140 ms, and the humps and the moose roll ~90 a generation: the stutters)
                ResetLane();
            }
            continue;
        }
        allDone = false;
        runT += dt;
        if (human && R == runners) {
            // the human: where on the road, and the wheel and pedals for the record; held still in the countdown
            const CVector2D pp = car->GetPosition().To2D();
            const float sp = CVector::Dot(car->m_vecMoveSpeed, car->GetForward()) * GAME_SPEED_TO_MPS;
            nearIdx = NearestIndex(pp, nearIdx, std::clamp(static_cast<int>(std::abs(sp)), 30, 120));
            offsetNow = Project(pp, nearIdx).offset;
            steerCmd = steerOut = car->m_fSteerAngle;
            gasOut = car->m_fGasPedal;
            brakeOut = car->m_fBreakPedal;
            targetKmh = 0.0f;
            if (runT < countdownS) { car->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f); car->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f); }
        }
        Record(car, dt);
        Trace(car);
        if (trail.empty() || (car->GetPosition() - trail.back()).Magnitude() > 1.0f) {
            trail.push_back(car->GetPosition());
            if (trail.size() > 1500) trail.pop_front();
        }
        const float speed = car->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
        if (curTest->pedal == "brake" && runT >= curTest->pedalStart && !brakeDone) stats.stopDist += speed * dt;
        const bool stopping = curTest->pedal == "brake" || curTest->pedal == "coast";
        R->slowTime = speed < 1.0f && runT > 3.0f && !stopping ? R->slowTime + dt : 0.0f;
        const char* end = nullptr;
        const bool openLoop = curTest->steer != "none";
        if (runT >= curTest->duration) end = "time";
        else if (!course.closed && !openLoop && nearIdx >= static_cast<int>(course.p.size()) - 4) end = "end";
        else if (!openLoop && std::abs(offsetNow) > cfg.abortOffset) end = "OFF";
        else if (car->GetUp().z < 0.3f) end = "ROLLED";
        else if ((car->GetPosition().To2D() - CVector2D(cfg.origin.x, cfg.origin.y)).Magnitude() > cfg.areaRadius) end = "AREA";
        else if (std::abs(CVector::Dot(car->m_vecMoveSpeed, car->GetRight())) * GAME_SPEED_TO_MPS > 6.0f &&
            std::abs(std::atan2(CVector::Dot(car->m_vecMoveSpeed, car->GetRight()), CVector::Dot(car->m_vecMoveSpeed, car->GetForward()))) > 1.2f)
            end = "SPUN";
        else if ((curTest->pedal == "brake" || curTest->pedal == "coast") && runT > curTest->pedalStart + 0.5f && speed < 0.3f) end = "stopped";
        else if (car->GetPosition().z < R->baseZ - 5.0f) end = "FELL";
        else if (R->slowTime > 8.0f) end = "STUCK";
        else if (!openLoop && runT - R->progressT > 15.0f) end = "NOPROG"; // moving, but not along the course (circling)
        // too slow to be worth waiting for: the first car home a while ago (steady state: the best time home so far)
        else if (GA::on && GA::cutoffFactor > 0.0f && (GA::steady ? GA::bestLapTime > 0.0f && runT > GA::bestLapTime * GA::cutoffFactor
                                                                   : firstHome > 0.0f && runT > firstHome * GA::cutoffFactor)) end = "CUT";
        if (end) {
            EndRun(end);
            R->finished = true;
            R->doneAt = clockT;
            if (!strcmp(end, "end") && firstHome < 0.0f) firstHome = runT;
        }
        }
        if (allDone && !(GA::on && GA::steady)) {
            speedIdx++;
            if (speedIdx >= static_cast<int>(curTest->speeds.size())) { speedIdx = 0; testIdx++; }
            if (testIdx >= static_cast<int>(CfgOf(0).tests.size())) {
                if (!GA::on) { state = NEXT_CAR; return; }
                // the GA: the next generation on the same tests with the same cars
                GA::NextGeneration();
                OpenTrace();
                testIdx = speedIdx = 0;
                if (GA::maxGenerations > 0 && GA::generation >= GA::maxGenerations) { Log("\nDone."); state = DONE; return; }
            }
            forRunners {
                // stop the car before the next run
                car->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f);
                car->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f);
                if (car->GetUp().z < 0.3f) { RemoveCar(); car = MakeCar(curModel); if (!car) { state = NEXT_CAR; return; } }
            }
            state = START_RUN;
            stateTime = now;
        }
        return;
    }
    default:
        return;
    }
}

// --- Drawing ---------------------------------------------------------------------------------------------------------

// 3D lines, drawn in batches (10 courses of 5 km: one draw call per line was far too slow)
static std::vector<RwIm3DVertex> lineBuf;
static void FlushLines() {
    if (lineBuf.empty()) return;
    if (RwIm3DTransform(lineBuf.data(), static_cast<RwUInt32>(lineBuf.size()), nullptr, 0)) {
        RwIm3DRenderPrimitive(rwPRIMTYPELINELIST);
        RwIm3DEnd();
    }
    lineBuf.clear();
}
static void Line3D(const CVector& a, const CVector& b, CRGBA c) {
    RwIm3DVertex v[2];
    RwIm3DVertexSetPos(&v[0], a.x, a.y, a.z);
    RwIm3DVertexSetPos(&v[1], b.x, b.y, b.z);
    RwIm3DVertexSetRGBA(&v[0], c.r, c.g, c.b, c.a);
    RwIm3DVertexSetRGBA(&v[1], c.r, c.g, c.b, c.a);
    lineBuf.push_back(v[0]);
    lineBuf.push_back(v[1]);
    if (lineBuf.size() >= 4000) FlushLines();
}

// filled quads (the road: asphalt, kerbs), batched as triangles
static std::vector<RwIm3DVertex> triBuf;
static void FlushTris() {
    if (triBuf.empty()) return;
    if (RwIm3DTransform(triBuf.data(), static_cast<RwUInt32>(triBuf.size()), nullptr, 0)) {
        RwIm3DRenderPrimitive(rwPRIMTYPETRILIST);
        RwIm3DEnd();
    }
    triBuf.clear();
}
static void Quad3D(const CVector& a, const CVector& b, const CVector& c, const CVector& d, CRGBA col) {
    for (const CVector* q : { &a, &b, &c, &a, &c, &d }) {
        RwIm3DVertex v;
        RwIm3DVertexSetPos(&v, q->x, q->y, q->z);
        RwIm3DVertexSetRGBA(&v, col.r, col.g, col.b, col.a);
        triBuf.push_back(v);
    }
    if (triBuf.size() >= 3000) FlushTris();
}

// one colour per car: its line on the ground, its trail, its row and dot in the HUD
static const CRGBA kColorTable[20] = {
    { 255, 214, 10, 255 }, { 255, 89, 94, 255 }, { 54, 162, 235, 255 }, { 76, 217, 100, 255 }, { 255, 146, 43, 255 },
    { 175, 110, 255, 255 }, { 0, 220, 220, 255 }, { 255, 105, 200, 255 }, { 190, 230, 60, 255 }, { 235, 235, 235, 255 },
    { 160, 120, 60, 255 }, { 120, 140, 255, 255 }, { 255, 180, 150, 255 }, { 0, 160, 120, 255 }, { 200, 60, 140, 255 },
    { 150, 200, 255, 255 }, { 255, 240, 140, 255 }, { 120, 90, 200, 255 }, { 255, 120, 70, 255 }, { 140, 140, 140, 255 } };
static CRGBA ColorOf(int k) {
    if (k < 20) return kColorTable[k];
    // 21-40: hues around the wheel, a little lighter
    const float h = std::fmod((k - 20) * 0.618034f, 1.0f) * 6.0f, x = 1.0f - std::abs(std::fmod(h, 2.0f) - 1.0f);
    float r = 0, g = 0, b = 0;
    switch (static_cast<int>(h)) { case 0: r = 1; g = x; break; case 1: r = x; g = 1; break; case 2: g = 1; b = x; break;
        case 3: g = x; b = 1; break; case 4: r = x; b = 1; break; default: r = 1; b = x; }
    auto c = [](float v) { return static_cast<unsigned char>(90 + 165 * v); };
    return CRGBA(c(r), c(g), c(b), 255);
}
static CRGBA Alpha(CRGBA c, int a) { c.a = static_cast<unsigned char>(a); return c; }

// HUD: positions in pixels of a 1080-line screen (u), x up to W
static float hudU = 1.0f, hudW = 1920.0f, hudH = 1080.0f;
static void Box(float x, float y, float w, float h, CRGBA c) {
    CSprite2d::DrawRect(CRect(x * hudU, y * hudU, (x + w) * hudU, (y + h) * hudU), c);
}
static void Text(float x, float y, const std::string& s, float size, CRGBA c, int align = 0, short font = FONT_SUBTITLES, bool edge = true) {
    CFont::SetFontStyle(font);
    CFont::SetProportional(true);
    CFont::SetBackground(false, false);
    CFont::SetScale(0.42f * size * hudU, 0.95f * size * hudU);
    CFont::SetColor(c);
    CFont::SetEdge(edge ? 1 : 0);
    CFont::SetDropShadowPosition(0);
    CFont::SetDropColor(CRGBA(0, 0, 0, 220));
    CFont::SetOrientation(align == 1 ? ALIGN_CENTER : align == 2 ? ALIGN_RIGHT : ALIGN_LEFT);
    CFont::SetWrapx(hudW * hudU);
    CFont::SetCentreSize(hudW * hudU);
    CFont::SetRightJustifyWrap(0.0f);
    CFont::PrintString(x * hudU, y * hudU, s.c_str());
}
// a bar from 0 to 1 (or centred: -1 to 1 from the middle)
static void Bar(float x, float y, float w, float h, float v, CRGBA c, bool centred = false) {
    Box(x, y, w, h, CRGBA(255, 255, 255, 35));
    v = std::clamp(v, centred ? -1.0f : 0.0f, 1.0f);
    if (centred) {
        const float m = x + w * 0.5f;
        if (v >= 0.0f) Box(m, y, w * 0.5f * v, h, c);
        else Box(m + w * 0.5f * v, y, -w * 0.5f * v, h, c);
        Box(m - 1.0f, y - 2.0f, 2.0f, h + 4.0f, CRGBA(255, 255, 255, 160));
    }
    else Box(x, y, w * v, h, c);
}
static void Chip(float x, float y, const char* label, bool lit, CRGBA c) {
    Box(x, y, 64.0f, 24.0f, lit ? c : CRGBA(255, 255, 255, 30));
    Text(x + 32.0f, y + 3.0f, label, 0.75f, lit ? CRGBA(20, 20, 20, 255) : CRGBA(255, 255, 255, 110), 1, FONT_SUBTITLES, !lit);
}
static const CRGBA kPanel(12, 14, 20, 185), kDim(255, 255, 255, 140), kWhite(255, 255, 255, 255);

static std::string StatusText(const Runner& r) {
    if (r.result.empty()) return "";
    if (r.result == "end") return Format("%.1f s", r.endTime);
    return r.result;
}

// F9: compact (0, the default), the full HUD of before (1), none (2)
static int hudMode = 0;

// The compact HUD: the GA's generations and how they evolve (top right), the car followed (bottom left), the course
// strip (bottom)
// The net of the car followed: its 80 inputs, its two layers and its 12 outputs (green > 0, red < 0, brighter the
// bigger), each output with what it does to the car now
static void DrawNetPanel() {
    R = &runners[std::clamp(camOn, 0, numRunners - 1)];
    if (cfg.mode != "neural2" || !car) return;
    const float x = 20.0f, w = 440.0f;
    float y = 20.0f;
    const float ph = 470.0f;
    Box(x, y, w, ph, kPanel);
    Box(x, y, 4.0f, ph, CRGBA(120, 200, 255, 255));
    Text(x + 14.0f, y + 6.0f, Format("REDE NEURAL  %d > %d > %d > %d", kNn2In, kNn2Hid, kNn2Hid2, kNn2Out), 0.8f, kWhite, 0, FONT_MENU);
    Text(x + w - 12.0f, y + 9.0f, Format("carro %s.%d", R->tag, R->runId), 0.55f, kDim, 2);
    y += 34.0f;
    auto cell = [](float cx, float cy, float cw, float chh, float v) {
        const float a = std::clamp(std::abs(v), 0.0f, 1.0f);
        Box(cx, cy, cw, chh, v >= 0.0f ? CRGBA(80, 230, 120, static_cast<unsigned char>(40 + 215 * a)) : CRGBA(255, 90, 90, static_cast<unsigned char>(40 + 215 * a)));
    };
    auto grid = [&](const char* title, const float* v, int n, int cols) {
        Text(x + 14.0f, y, title, 0.55f, kDim);
        y += 16.0f;
        const float cw = (w - 28.0f) / cols;
        for (int i = 0; i < n; i++) cell(x + 14.0f + cw * (i % cols) + 1.0f, y + 12.0f * (i / cols), cw - 2.0f, 10.0f, v[i]);
        y += 12.0f * ((n + cols - 1) / cols) + 6.0f;
    };
    grid("entradas: raios, pista, movimento, rodas, cambio, linha", R->nnIn, kNn2In, 20);
    grid("camada 1", R->nnAct, kNn2Hid, 24);
    grid("camada 2", R->nnAct + kNn2Hid, kNn2Hid2, 24);
    const float* o = R->nnAct + kNn2Hid + kNn2Hid2;
    const float lock = HandlingLockRad(car) * 180.0f / 3.14159265f;
    const std::string names[kNn2Out] = { "volante", "acelerador / freio", "linha", "controle de tracao", "ABS", "freio de mao", "olhar a frente", "agressividade" };
    std::string eff[kNn2Out];
    eff[0] = Format("%+.1f graus", o[0] * kNn2SteerSpan * lock);
    eff[1] = Format("pedal %+.2f", kNn2SpeedSpan * o[1]);
    eff[2] = Format("%+.2f m", o[2] * R->lineRoom);
    eff[3] = Format("x%.2f", std::exp(0.7f * o[3]));
    eff[4] = Format("x%.2f", std::exp(0.5f * o[4]));
    eff[5] = o[5] > 0.0f ? Format("%.0f%% abaixo de 80", 100.0f * std::min(o[5] * o[5], 1.0f)) : std::string("-");
    eff[6] = Format("x%.2f", std::exp(0.5f * o[6]));
    eff[7] = Format("x%.2f", std::exp(0.25f * o[7]));
    Text(x + 14.0f, y, "saidas (0 = o professor, pursuit)", 0.55f, kDim);
    y += 18.0f;
    for (int m = 0; m < kNn2Out; m++) {
        Text(x + 14.0f, y, names[m], 0.56f, kWhite);
        Bar(x + 170.0f, y + 4.0f, 160.0f, 9.0f, std::clamp(o[m], -1.0f, 1.0f), o[m] >= 0.0f ? CRGBA(80, 230, 120, 230) : CRGBA(255, 90, 90, 230), true);
        Text(x + w - 12.0f, y, eff[m], 0.56f, kWhite, 2);
        y += 17.0f;
    }
    R = runners;
}

static void DrawHudCompact() {
    const float W = hudW, H = hudH;
    DrawNetPanel();
    const CRGBA kGood(120, 230, 120, 255), kBad(255, 110, 110, 255), kGold(255, 214, 10, 255);
    if (GA::on) {
        const float gw = 420.0f, x = W - 20.0f - gw;
        float y = 20.0f;
        const int nb = static_cast<int>(GA::batches.size());
        const float ph = 300.0f + 18.0f * std::min(nb, 3);
        Box(x, y, gw, ph, kPanel);
        Box(x, y, 4.0f, ph, CRGBA(175, 110, 255, 255));
        Text(x + 16.0f, y + 6.0f, Format("GERACAO %d", GA::generation), 0.9f, kWhite, 0, FONT_MENU);
        Text(x + gw - 12.0f, y + 8.0f, Perf::fps > 0.0f ? Format("%.0f fps  mod %.1f ms", Perf::fps, Perf::modMs) : std::string(""), 0.58f, kDim, 2);
        y += 34.0f;
        // each generation still running: born of Runners (light) and runs ended (solid); the oldest closes next
        int shown = 0;
        for (const auto& bg : GA::batches) {
            if (shown++ >= 3) break;
            const int g = bg.first;
            const int born = g < GA::birthGen ? numRunners : g == GA::birthGen ? GA::birthCount : 0;
            const int done = static_cast<int>(bg.second.size());
            const char* what = shown == 1 && nb > 1 ? "fechando" : g == GA::birthGen ? "nascendo" : "rodando";
            Text(x + 16.0f, y, Format("g%d %s", g, what), 0.6f, kWhite);
            Box(x + 150.0f, y + 5.0f, 170.0f, 9.0f, CRGBA(255, 255, 255, 30));
            Box(x + 150.0f, y + 5.0f, 170.0f * born / std::max(numRunners, 1), 9.0f, CRGBA(175, 110, 255, 90));
            Box(x + 150.0f, y + 5.0f, 170.0f * done / std::max(numRunners, 1), 9.0f, CRGBA(175, 110, 255, 255));
            Text(x + gw - 12.0f, y, Format("%d/%d", done, numRunners), 0.6f, kDim, 2);
            y += 18.0f;
        }
        y += 4.0f;
        // the last generation closed and the record
        const auto& h = GA::history;
        const auto& h10 = GA::history10;
        if (!h10.empty())
            Text(x + 16.0f, y, Format("ultima (g%d): melhor %.1f   media dos 10 melhores %.1f", GA::generation - 1, h.back(), h10.back()), 0.6f, kWhite);
        else Text(x + 16.0f, y, "nenhuma geracao fechada ainda", 0.6f, kDim);
        y += 18.0f;
        if (GA::bestGen >= 0)
            Text(x + 16.0f, y, Format("recorde %.1f (g%d, %s)   %.1f s, desvio %.2f m", GA::bestScore, GA::bestGen, GA::modes[GA::best.mode].c_str(),
                GA::bestRes.time, GA::bestRes.mean), 0.6f, kGold);
        y += 22.0f;
        // the evolution: the mean of each generation's 10 best (bars, taller = better) and its best (white tick)
        {
            const float bx = x + 16.0f, bw = gw - 32.0f, bh = 70.0f;
            Box(bx, y, bw, bh, CRGBA(255, 255, 255, 16));
            const int n = std::min<int>(static_cast<int>(h10.size()), 30);
            if (n > 0) {
                double lo = 1e18, hi = -1e18;
                for (int i = static_cast<int>(h10.size()) - n; i < static_cast<int>(h10.size()); i++) {
                    if (h10[i] < 1000.0) { lo = std::min(lo, h10[i]); hi = std::max(hi, h10[i]); }
                    if (i < static_cast<int>(h.size()) && h[i] < 1000.0) { lo = std::min(lo, h[i]); hi = std::max(hi, h[i]); }
                }
                if (hi <= lo) hi = lo + 1.0;
                const float step = bw / 30.0f;
                auto hgt = [&](double v) { return static_cast<float>(0.15 + 0.85 * (hi - std::min(v, hi)) / (hi - lo)) * bh; };
                for (int i = 0; i < n; i++) {
                    const int j = static_cast<int>(h10.size()) - n + i;
                    const float f = hgt(h10[j]);
                    Box(bx + step * i + 1.0f, y + bh - f, step - 2.0f, f, CRGBA(120, 200, 255, 200));
                    if (j < static_cast<int>(h.size()) && h[j] < 1000.0) Box(bx + step * i + 1.0f, y + bh - hgt(h[j]) - 1.0f, step - 2.0f, 3.0f, kWhite);
                }
                Text(bx + 4.0f, y + 2.0f, Format("%.1f", lo), 0.5f, kDim);
                Text(bx + 4.0f, y + bh - 16.0f, Format("%.1f", hi), 0.5f, kDim);
            }
            y += bh + 4.0f;
            if (h10.size() >= 6) {
                const double d = h10.back() - h10[h10.size() - 6];
                Text(x + 16.0f, y, Format("10 melhores: %+.1f s em 5 geracoes   %s", d, d < -0.2 ? "evoluindo" : d > 0.2 ? "piorou" : "parado"), 0.6f,
                    d < -0.2 ? kGood : d > 0.2 ? kBad : kDim);
            }
            else Text(x + 16.0f, y, "barras: media dos 10 melhores   traco: o melhor", 0.56f, kDim);
            y += 18.0f;
            Text(x + 16.0f, y, Format("sigma %.3f%s   arquivo %d", GA::sigma, GA::stall > 0 ? Format(" (sem melhora %d)", GA::stall).c_str() : "",
                static_cast<int>(GA::archive.size())), 0.56f, kDim);
            y += 18.0f;
            Text(x + 16.0f, y, Format("pontos = tempo + %.0f x m.s alem da borda + %.0f x s com 4 rodas fora", GA::edgePenalty, GA::offPenalty), 0.56f, kDim);
            y += 20.0f;
        }
        // the closing generation's 5 best so far
        if (!GA::batches.empty()) {
            std::vector<const GA::Done*> top;
            for (const auto& dn : GA::batches.begin()->second) top.push_back(&dn);
            const int nt = std::min<int>(5, static_cast<int>(top.size()));
            std::partial_sort(top.begin(), top.begin() + nt, top.end(), [](const GA::Done* a, const GA::Done* b) { return a->r.score < b->r.score; });
            for (int i = 0; i < nt; i++) {
                const GA::Done& d = *top[i];
                Text(x + 16.0f, y, Format("%d. %s %s", i + 1, d.id.c_str(), GA::modes[d.g.mode].c_str()), 0.56f, kWhite);
                Text(x + gw - 12.0f, y, d.r.why == "end" ? Format("%.1f  (%.1f s, %.2f m)", d.r.score, d.r.time, d.r.mean) : d.r.why, 0.56f,
                    d.r.why == "end" ? kGood : kBad, 2);
                y += 16.0f;
            }
        }
        Text(x + 16.0f, 20.0f + ph - 18.0f, "F9 HUD   PgUp/PgDn carros   Home auto", 0.5f, kDim);
    }

    // the car followed
    R = &runners[std::clamp(camOn, 0, numRunners - 1)];
    if (car) {
        const float pw = 470.0f, ph = 100.0f, x = 20.0f, y = H - 40.0f - ph;
        Box(x, y, pw, ph, kPanel);
        Box(x, y, pw, 3.0f, ColorOf(R->index));
        const std::string who = GA::on && GA::steady ? Format("%s.%d  %s  g%d", R->tag, R->runId, cfg.mode.c_str(), R->bornGen)
                                                     : Format("%s  %s", R->tag, cfg.mode.c_str());
        Text(x + 12.0f, y + 6.0f, who, 0.75f, ColorOf(R->index), 0, FONT_MENU);
        Text(x + pw - 12.0f, y + 8.0f, camPick >= 0 ? "camera: escolhida" : "camera: destaque da geracao nova", 0.52f, kDim, 2);
        Text(x + 100.0f, y + 30.0f, Format("%.0f", R->speedNow), 1.8f, kWhite, 2, FONT_PRICEDOWN);
        Text(x + 104.0f, y + 44.0f, "km/h", 0.6f, kDim);
        const float pace = PaceOf(R->index);
        if (!R->result.empty())
            Text(x + 160.0f, y + 32.0f, R->result == "end" ? Format("chegou: %.1f s", R->endTime) : R->result, 0.7f, R->result == "end" ? kGood : kBad);
        else if (pace < 1e8f)
            Text(x + 160.0f, y + 32.0f, GA::bestLapTime > 0.0f ? Format("ritmo %.1f s  (%+.1f do recorde)", pace, pace - GA::bestLapTime) : Format("ritmo %.1f s", pace), 0.7f,
                GA::bestLapTime > 0.0f && pace < GA::bestLapTime ? kGood : kWhite);
        else Text(x + 160.0f, y + 32.0f, Format("t %.0f s", runT), 0.7f, kDim);
        Text(x + pw - 12.0f, y + 32.0f, SectionAt(course, course.s.empty() ? 0.0f : course.s[nearIdx]), 0.6f, kDim, 2);
        // across the road (the bar is the lane, edge to edge), the pedals, ABS / TC
        const float bx = x + 160.0f;
        if (course.halfWidth > 0.0f)
            Bar(bx, y + 58.0f, 180.0f, 9.0f, -offsetNow / course.halfWidth,
                R->marginNow < 0.0f ? CRGBA(255, 90, 90, 230) : R->marginNow < edgeComfortGA ? CRGBA(255, 200, 60, 230) : CRGBA(120, 230, 120, 230), true);
        Text(bx + 186.0f, y + 54.0f, Format("desvio %.2f m", stats.time > 0.0 ? stats.absE / stats.time : 0.0), 0.52f, kDim);
        Bar(bx, y + 76.0f, 85.0f, 8.0f, gasOut, CRGBA(76, 217, 100, 230));
        Bar(bx + 95.0f, y + 76.0f, 85.0f, 8.0f, brakeOut, CRGBA(255, 89, 94, 230));
        Text(bx + 186.0f, y + 72.0f, Format("%s%s%s", R->absOn ? "ABS " : "", R->tcOn ? "TC " : "", R->skidFNow || R->skidRNow ? "DERRAPANDO" : ""), 0.52f, kGold);
        Text(x + 12.0f, y + 72.0f, Format("%.2f g", std::abs(R->latGNow)), 0.55f, kDim);
    }

    // the course strip: its parts and every car on it (the newest generation's brighter)
    R = runners;
    if (course.length > 1.0f) {
        const float x0 = 40.0f, x1 = W - 40.0f, y = H - 18.0f;
        Box(x0 - 12.0f, y - 12.0f, x1 - x0 + 24.0f, 24.0f, kPanel);
        Box(x0, y - 1.0f, x1 - x0, 2.0f, CRGBA(255, 255, 255, 90));
        for (const auto& sec : course.sections) {
            const float sx = x0 + (x1 - x0) * sec.first / course.length;
            Box(sx - 1.0f, y - 6.0f, 2.0f, 12.0f, CRGBA(255, 255, 255, 120));
        }
        int young = -1;
        for (int k = 0; k < numRunners; k++) young = std::max(young, runners[k].bornGen);
        for (int k = numRunners - 1; k >= 0; k--) {
            if (!runners[k].result.empty()) continue;
            const float sx = x0 + (x1 - x0) * std::clamp(runners[k].sMax / course.length, 0.0f, 1.0f);
            const bool hot = runners[k].bornGen == young;
            Box(sx - 3.0f, y - 3.0f, 6.0f, 6.0f, hot ? ColorOf(k) : CRGBA(255, 255, 255, 60));
        }
        if (camOn >= 0 && camOn < numRunners) {
            const float sx = x0 + (x1 - x0) * std::clamp(runners[camOn].sMax / course.length, 0.0f, 1.0f);
            Box(sx - 1.0f, y - 11.0f, 2.0f, 22.0f, kGold);
        }
    }
    R = runners;
}

static void DrawHud() {
    hudU = static_cast<float>(RsGlobal.maximumHeight) / 1080.0f;
    hudH = 1080.0f;
    hudW = static_cast<float>(RsGlobal.maximumWidth) / hudU;
    const float W = hudW, H = hudH;
    if (state != RUNNING || !curTest) {
        Box(24.0f, 24.0f, 620.0f, 56.0f, kPanel);
        Text(40.0f, 38.0f, state == DONE ? "PROVING GROUND - fim: edite o ini para rodar de novo" : "PROVING GROUND - preparando", 1.0f, kWhite);
        return;
    }
    static bool prevF9 = false;
    const bool f9 = KeyPressed(VK_F9) != 0;
    if (f9 && !prevF9) hudMode = (hudMode + 1) % 3;
    prevF9 = f9;
    if (hudMode == 2) { Text(W - 20.0f, H - 30.0f, "F9: HUD", 0.5f, kDim, 2); return; }
    if (hudMode == 0) { DrawHudCompact(); return; }
    const std::vector<int> order = Standings();
    if (human && runT < countdownS + 1.0f) {
        // the countdown
        const std::string c = runT < countdownS ? Format("%d", static_cast<int>(std::ceil(countdownS - runT))) : "VAI!";
        Text(W * 0.5f, 330.0f, c, 7.0f, runT < countdownS ? CRGBA(255, 214, 10, 255) : CRGBA(76, 217, 100, 255), 1, FONT_PRICEDOWN);
    }

    // header
    Box(24.0f, 24.0f, 700.0f, 70.0f, kPanel);
    Box(24.0f, 24.0f, 6.0f, 70.0f, ColorOf(0));
    Text(44.0f, 30.0f, "PROVING GROUND", 1.25f, kWhite, 0, FONT_MENU);
    std::string sub = Format("%s   modelo %d   t %.1f s", curTest->name.c_str(), curModel, runT);
    if (GA::on) sub = Format("Geracao %d   ", GA::generation) + sub;
    Text(44.0f, 62.0f, sub, 0.85f, kDim);

    // standings: the finished ones by score (what the GA ranks), then the ones running by how far they got
    // (no more than 24 rows: the rest in one line; the followed car always shown)
    const int rows = std::min(numRunners, 24);
    const float rowH = numRunners > 12 ? 24.0f : 30.0f, top = 104.0f, pw = 860.0f;
    const float ts = numRunners > 12 ? 0.74f : 0.85f;
    Box(24.0f, top, pw, 34.0f + rowH * rows + (numRunners > rows ? rowH : 0.0f), kPanel);
    const float cPos = 36.0f, cCar = 66.0f, cMode = 112.0f, cSpeed = 290.0f, cBar = 306.0f, cOff = 590.0f, cTime = 690.0f, cScore = 872.0f;
    Text(cCar, top + 8.0f, "carro", 0.7f, kDim);
    Text(cMode, top + 8.0f, "algoritmo", 0.7f, kDim);
    Text(cSpeed, top + 8.0f, "km/h", 0.7f, kDim, 2);
    Text(cBar, top + 8.0f, "progresso", 0.7f, kDim);
    Text(cOff, top + 8.0f, course.halfWidth > 0.0f ? "margem / min" : "desvio medio / max", 0.7f, kDim, 2);
    Text(cTime, top + 8.0f, "tempo", 0.7f, kDim, 2);
    Text(cScore, top + 8.0f, "pontos", 0.7f, kDim, 2);
    std::vector<int> shown;
    for (int i = 0; i < static_cast<int>(order.size()); i++) if (i < rows - 1 || order[i] == camOn) shown.push_back(i);
    if (static_cast<int>(shown.size()) < rows && rows < static_cast<int>(order.size())) shown.insert(shown.begin() + (rows - 1), rows - 1);
    shown.resize(std::min<size_t>(shown.size(), rows));
    if (numRunners <= rows) { shown.clear(); for (int i = 0; i < numRunners; i++) shown.push_back(i); }
    if (numRunners > rows) {
        int done = 0, out = 0;
        for (int k = 0; k < numRunners; k++) { done += runners[k].result == "end"; out += !runners[k].result.empty() && runners[k].result != "end"; }
        Text(cCar, top + 34.0f + rowH * rows + 4.0f, Format("+ %d carros   (chegaram %d, fora %d, rodando %d)", numRunners - rows, done, out,
            numRunners - done - out), ts, kDim);
    }
    for (int row = 0; row < static_cast<int>(shown.size()); row++) {
        const int i = shown[row];
        const int k = order[i];
        R = &runners[k];
        const float y = top + 34.0f + rowH * row;
        if (k == camOn) Box(24.0f, y, pw, rowH, CRGBA(255, 255, 255, 28));
        Text(cPos, y + 4.0f, Format("%d", i + 1), ts, kDim);
        Box(cCar, y + 5.0f, 16.0f, 16.0f, ColorOf(k));
        Text(cCar + 24.0f, y + 4.0f, R->tag, ts, kWhite);
        const std::string born = GA::on && GA::steady ? Format("  g%d", R->bornGen) : std::string();
        Text(cMode, y + 4.0f, human && k == 0 ? std::string("VOCE (humano)") : (GA::on && !GA::pop[k].label.empty() ? cfg.mode + " - " + GA::pop[k].label : cfg.mode) + born,
            ts, human && k == 0 ? CRGBA(255, 214, 10, 255) : kWhite);
        const bool running = R->result.empty(), arrived = R->result == "end";
        Text(cSpeed, y + 4.0f, running ? Format("%.0f", R->speedNow) : "-", ts, kWhite, 2);
        const float prog = course.length > 1.0f ? R->sMax / course.length : 0.0f;
        Bar(cBar, y + 9.0f, 120.0f, 9.0f, prog, Alpha(ColorOf(k), 220));
        Text(cBar + 128.0f, y + 4.0f, Format("%.0f%%", prog * 100.0f), 0.68f, kDim);
        // the mean offset so far (what the GA scores) and the most
        const float mean = stats.time > 0.0 ? static_cast<float>(stats.absE / stats.time) : 0.0f;
        const CRGBA meanC = mean > 0.3f ? CRGBA(255, 90, 90, 255) : mean > 0.15f ? CRGBA(255, 200, 60, 255) : CRGBA(120, 230, 120, 255);
        if (course.halfWidth > 0.0f) {
            // the room to the road's edge now and the least so far (green: more than EdgeComfort)
            const float mm = stats.minMargin < 1e8 ? static_cast<float>(stats.minMargin) : R->marginNow;
            const CRGBA mc = mm < 0.0f ? CRGBA(255, 90, 90, 255) : mm < edgeComfortGA ? CRGBA(255, 200, 60, 255) : CRGBA(120, 230, 120, 255);
            Text(cOff, y + 4.0f, Format("%.2f / %.2f", running ? R->marginNow : 0.0f, mm), ts, mc, 2);
        }
        else Text(cOff, y + 4.0f, Format("%.2f / %.2f", mean, stats.maxE), ts, meanC, 2);
        Text(cTime, y + 4.0f, running ? Format("%.0f s", runT) : arrived ? Format("%.1f s", R->endTime) : R->result, ts,
            running ? kDim : arrived ? kWhite : CRGBA(255, 90, 90, 255), 2);
        if (GA::on && !running)
            Text(cScore, y + 4.0f, Format("%.1f", GA::res[k].score), ts, arrived ? CRGBA(120, 230, 120, 255) : CRGBA(255, 90, 90, 255), 2);
    }

    // GA: the best of this generation so far and the best ever, each with what makes its score
    if (GA::on) {
        const float gw = 500.0f, x = W - 24.0f - gw, y = 24.0f;
        Box(x, y, gw, 404.0f, kPanel);
        Box(x, y, 6.0f, 404.0f, CRGBA(175, 110, 255, 255));
        Text(x + 20.0f, y + 8.0f, "ALGORITMO GENETICO", 1.0f, kWhite, 0, FONT_MENU);
        if (GA::steady) {
            // the generations still running: how many of their Runners runs ended
            std::string open;
            int shown = 0;
            for (const auto& bg : GA::batches) if (shown++ < 3) open += Format("g%d %d/%d   ", bg.first, static_cast<int>(bg.second.size()), numRunners);
            Text(x + 20.0f, y + 38.0f, Format("%sarquivo %d  sigma %.3f", open.c_str(), static_cast<int>(GA::archive.size()), GA::sigma), 0.74f, kDim);
        }
        else Text(x + 20.0f, y + 38.0f, Format("geracao %d   populacao %d   sigma %.3f", GA::generation, numRunners, GA::sigma), 0.78f, kDim);
        if (course.halfWidth > 0.0f && GA::edgePenalty <= 0.0f)
            Text(x + 20.0f, y + 60.0f, Format("pontos = tempo + %.0f x s fora da pista (%.0f m): dentro dela, qualquer linha", GA::offPenalty,
                2.0f * course.halfWidth), 0.66f, kDim);
        else if (course.halfWidth > 0.0f)
            Text(x + 20.0f, y + 60.0f, Format("pontos = tempo + %.0f x m.s a menos de %.1f m da borda + %.0f x s fora da pista (%.0f m)",
                GA::edgePenalty, edgeComfortGA, GA::offPenalty, 2.0f * course.halfWidth), 0.66f, kDim);
        else Text(x + 20.0f, y + 60.0f, Format("pontos = tempo + %.0f x desvio medio + %.0f x s fora de %.0f m", GA::meanPenalty, GA::offPenalty, offLimitGA), 0.68f, kDim);
        auto card = [&](float cy, const char* title, int k, const std::string& mode, double score, double time, double mean, double maxE) {
            Box(x + 16.0f, cy, gw - 32.0f, 112.0f, CRGBA(255, 255, 255, 16));
            Text(x + 28.0f, cy + 8.0f, title, 0.72f, kDim);
            if (k < -1) { Text(x + 28.0f, cy + 40.0f, "ainda nenhum carro chegou", 0.85f, kDim); return; }
            if (k >= 0) {
                Box(x + 28.0f, cy + 38.0f, 18.0f, 18.0f, ColorOf(k));
                Text(x + 54.0f, cy + 36.0f, Format("%s  %s", kTags[k], mode.c_str()), 0.95f, kWhite);
            }
            else Text(x + 28.0f, cy + 36.0f, mode, 0.95f, kWhite);
            Text(x + gw - 28.0f, cy + 26.0f, Format("%.1f", score), 1.9f, CRGBA(120, 230, 120, 255), 2, FONT_PRICEDOWN);
            Text(x + gw - 28.0f, cy + 64.0f, "pontos", 0.65f, kDim, 2);
            Text(x + 28.0f, cy + 78.0f, Format("tempo %.1f s    desvio medio %.3f m    max %.2f m", time, mean, maxE), 0.78f, kWhite);
        };
        int bestNow = -2;
        for (int k = 0; k < numRunners; k++)
            if (runners[k].result == "end" && (bestNow < 0 || GA::res[k].score < GA::res[bestNow].score)) bestNow = k;
        const GA::Done* bestDone = nullptr;
        const int oldest = GA::batches.empty() ? -1 : GA::batches.begin()->first;
        if (oldest >= 0) for (const auto& dn : GA::batches.begin()->second) if (dn.r.why == "end" && (!bestDone || dn.r.score < bestDone->r.score)) bestDone = &dn;
        if (GA::steady && bestDone)
            card(y + 88.0f, Format("MELHOR DA GERACAO %d (%d de %d)", oldest, static_cast<int>(GA::batches.begin()->second.size()), numRunners).c_str(), -1,
                bestDone->id + "  " + GA::modes[bestDone->g.mode], bestDone->r.score, bestDone->r.time, bestDone->r.mean, bestDone->r.maxE);
        else if (bestNow >= 0) {
            const GA::Result& b = GA::res[bestNow];
            card(y + 88.0f, "MELHOR DESTA GERACAO", bestNow, CfgOf(bestNow).mode, b.score, b.time, b.mean, b.maxE);
        }
        else card(y + 88.0f, "MELHOR DESTA GERACAO", -2, "", 0, 0, 0, 0);
        if (GA::bestGen >= 0)
            card(y + 208.0f, Format("MELHOR DE TODAS (geracao %d)", GA::bestGen).c_str(), -1, GA::modes[GA::best.mode], GA::bestScore,
                GA::bestRes.time, GA::bestRes.mean, GA::bestRes.maxE);
        else card(y + 208.0f, "MELHOR DE TODAS", -2, "", 0, 0, 0, 0);
        // best of each generation, the last 40 (lower is better: taller bar)
        const auto& h = GA::history;
        const int n = std::min<int>(static_cast<int>(h.size()), 40);
        const float bx = x + 20.0f, by = y + 330.0f, bw = gw - 40.0f, bh = 62.0f, step = bw / 40.0f;
        Box(bx, by, bw, bh, CRGBA(255, 255, 255, 18));
        Text(bx + 6.0f, by + 4.0f, "melhor de cada geracao", 0.6f, kDim);
        if (n > 0) {
            double lo = 1e18, hi = 0.0;
            for (int i = static_cast<int>(h.size()) - n; i < static_cast<int>(h.size()); i++) { lo = std::min(lo, h[i]); hi = std::max(hi, h[i]); }
            for (int i = 0; i < n; i++) {
                const double v = h[h.size() - n + i];
                const float f = hi > lo ? static_cast<float>(1.0 - 0.85 * (v - lo) / (hi - lo)) : 1.0f;
                Box(bx + step * i + 1.0f, by + bh * (1.0f - f), step - 2.0f, bh * f, v < 1000.0 ? CRGBA(120, 200, 255, 220) : CRGBA(255, 90, 90, 200));
            }
        }
    }

    // the followed car
    R = &runners[std::clamp(camOn, 0, numRunners - 1)];
    if (car) {
        const float pw = 820.0f, ph = 194.0f, x = (W - pw) * 0.5f, y = H - 90.0f - ph;
        Box(x, y, pw, ph, kPanel);
        Box(x, y, pw, 4.0f, ColorOf(R->index));
        std::string genes;
        if (GA::on) {
            genes = Format("Grip %.2f (fechadas x%.2f, rapidas x%.2f)  BrakeG %.2f  Look %.2f s", cfg.gripFactor, cfg.gripTight,
                cfg.gripFast, cfg.speedBrakeG, cfg.lookTime);
        }
        Text(x + 16.0f, y + 12.0f, Format("CARRO %s  %s", R->tag, cfg.mode.c_str()), 1.0f, ColorOf(R->index), 0, FONT_MENU);
        Text(x + pw - 16.0f, y + 14.0f, SectionAt(course, course.s.empty() ? 0.0f : course.s[nearIdx]), 1.0f, kWhite, 2);
        Text(x + 16.0f, y + 42.0f, genes, 0.72f, kDim);
        // the grip it learned for each kind of curve (radius, m): green above 1 (faster than planned), red below
        if (cfg.learnRate > 0.0f) {
            Text(x + 16.0f, y + 166.0f, Format("aprendido (passo %.2f): curvas", 0.03f + 0.25f * cfg.learnRate), 0.64f, kDim);
            float tc = 0.0f;
            for (float m : R->tcBand) tc += m / 5.0f;
            (void)tc;
            for (int b = 0; b < 5; b++) {
                const float m = R->learned[b];
                Text(x + 250.0f + 72.0f * b, y + 166.0f, Format("%s x%.2f", b < 5 ? kBandNames[b] : b == 5 ? "ABS" : "TC", m), 0.64f,
                    m > 1.02f ? CRGBA(120, 230, 120, 255) : m < 0.98f ? CRGBA(255, 120, 120, 255) : kWhite);
            }
            Text(x + pw - 16.0f, y + 166.0f, Format("%d/%d mantidas", R->kept, R->trials), 0.64f, kDim, 2);
        }
        // speed
        Text(x + 150.0f, y + 66.0f, Format("%.0f", R->speedNow), 2.6f, kWhite, 2, FONT_PRICEDOWN);
        Text(x + 156.0f, y + 84.0f, "km/h", 0.8f, kDim);
        Text(x + 156.0f, y + 110.0f, targetKmh >= 2000.0f ? std::string("alvo livre") : Format("alvo %.0f", targetKmh), 0.8f,
            targetKmh < R->speedNow - 5.0f ? CRGBA(255, 120, 120, 255) : kDim);
        // wheel, offset, sideways g
        const float bx = x + 270.0f, bw = 330.0f;
        const float lock = car->m_pHandlingData->m_fSteeringLock * 3.14159265f / 180.0f;
        Text(bx, y + 70.0f, "volante", 0.7f, kDim);
        Bar(bx + 80.0f, y + 74.0f, bw - 80.0f, 12.0f, -car->m_fSteerAngle / lock, CRGBA(120, 200, 255, 230), true);
        if (course.halfWidth > 0.0f) {
            // where the car is across the road: the bar is the road, edge to edge
            Text(bx, y + 96.0f, Format("margem %.2f m", R->marginNow), 0.7f, kDim);
            Bar(bx + 80.0f, y + 100.0f, bw - 80.0f, 12.0f, -offsetNow / course.halfWidth,
                R->marginNow < 0.0f ? CRGBA(255, 90, 90, 230) : R->marginNow < edgeComfortGA ? CRGBA(255, 200, 60, 230) : CRGBA(120, 230, 120, 230), true);
        }
        else {
            Text(bx, y + 96.0f, Format("desvio %+.2f (med %.2f)", offsetNow, stats.time > 0.0 ? stats.absE / stats.time : 0.0), 0.7f, kDim);
            Bar(bx + 80.0f, y + 100.0f, bw - 80.0f, 12.0f, -offsetNow / (2.0f * offLimitGA),
                std::abs(offsetNow) > offLimitGA ? CRGBA(255, 90, 90, 230) : CRGBA(120, 230, 120, 230), true);
        }
        Text(bx, y + 122.0f, Format("lateral %.2f g", std::abs(R->latGNow)), 0.7f, kDim);
        Bar(bx + 80.0f, y + 126.0f, bw - 80.0f, 12.0f, std::abs(R->latGNow) / 2.0f, CRGBA(255, 200, 60, 230));
        // pedals
        const float px = x + 620.0f;
        Text(px, y + 70.0f, "gas", 0.7f, kDim);
        Bar(px + 44.0f, y + 74.0f, 140.0f, 12.0f, gasOut, CRGBA(76, 217, 100, 230));
        Text(px, y + 96.0f, "freio", 0.7f, kDim);
        Bar(px + 44.0f, y + 100.0f, 140.0f, 12.0f, brakeOut, CRGBA(255, 89, 94, 230));
        Chip(px - 2.0f, y + 126.0f, "ABS", R->absOn, CRGBA(255, 200, 60, 255));
        Chip(px + 66.0f, y + 126.0f, "TC", R->tcOn, CRGBA(255, 200, 60, 255));
        Chip(px + 134.0f, y + 126.0f, R->skidRNow ? "TRAS" : "FRENTE", R->skidFNow || R->skidRNow, CRGBA(255, 89, 94, 255));
        if (R->windNow != 0.0f)
            Text(x + pw - 16.0f, y + 42.0f, Format("VENTO %s %.1f m/s2", R->windNow > 0.0f ? "<-" : "->", std::abs(R->windNow)), 0.85f, CRGBA(120, 200, 255, 255), 2);
    }

    // progress strip: the course's parts and every car on it
    R = runners;
    if (course.length > 1.0f) {
        const float x0 = 60.0f, x1 = W - 60.0f, y = H - 44.0f;
        Box(x0 - 20.0f, y - 30.0f, x1 - x0 + 40.0f, 64.0f, kPanel);
        Box(x0, y, x1 - x0, 4.0f, CRGBA(255, 255, 255, 90));
        int i = 0;
        for (const auto& p : course.sections) {
            const float sx = x0 + (x1 - x0) * p.first / course.length;
            Box(sx - 1.0f, y - 6.0f, 2.0f, 16.0f, CRGBA(255, 255, 255, 140));
            Text(sx + 4.0f, (i++ % 2) ? y + 8.0f : y - 26.0f, p.second, 0.62f, kDim);
        }
        for (int k = numRunners - 1; k >= 0; k--) {
            const float sx = x0 + (x1 - x0) * std::clamp(runners[k].sMax / course.length, 0.0f, 1.0f);
            Box(sx - 6.0f, y - 4.0f, 12.0f, 12.0f, ColorOf(k));
        }
    }
    R = runners;
}

void Draw() {
    if (!enabled || state == WAIT_GAME) return;
    Perf::Scope perf(Perf::DRAW);
    RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)FALSE);
    RwRenderStateSet(rwRENDERSTATETEXTURERASTER, nullptr);
    RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)TRUE);
    // the grid on the deck the camera is over, each course on its car's deck
    const CVector camPos = TheCamera.GetPosition();
    float z = cfg.origin.z + 0.08f;
    while (deckHeight > 1.0f && camPos.z > z + deckHeight && z < cfg.origin.z + deckHeight * (kMaxRunners - 1) / lanesPerDeck) z += deckHeight;
    const CVector cam = TheCamera.GetPosition();
    const float near2 = 260.0f * 260.0f;
    auto close = [&](const CVector2D& p) { return (p.x - cam.x) * (p.x - cam.x) + (p.y - cam.y) * (p.y - cam.y) < near2; };
    if (cfg.grid && cfg.gridSize > 1.0f) {
        // the grid around the camera, in the course's frame
        const float a = cfg.headingDeg * 3.14159265f / 180.0f;
        const CVector2D d(std::cos(a), std::sin(a)), l(-std::sin(a), std::cos(a));
        const CVector2D O(cfg.origin.x, cfg.origin.y);
        const CVector2D rel(cam.x - cfg.origin.x, cam.y - cfg.origin.y);
        const float cu = std::floor(CVector2D::Dot(rel, d) / cfg.gridSize) * cfg.gridSize;
        const float cv = std::floor(CVector2D::Dot(rel, l) / cfg.gridSize) * cfg.gridSize;
        const float span = 150.0f;
        for (float u = cu - span; u <= cu + span; u += cfg.gridSize) {
            const CVector2D p0 = O + d * u + l * (cv - span), p1 = O + d * u + l * (cv + span);
            const bool axis = std::abs(u) < 0.01f;
            Line3D({ p0.x, p0.y, z }, { p1.x, p1.y, z }, axis ? CRGBA(255, 80, 80, 200) : CRGBA(255, 255, 255, 70));
        }
        for (float v = cv - span; v <= cv + span; v += cfg.gridSize) {
            const CVector2D p0 = O + d * (cu - span) + l * v, p1 = O + d * (cu + span) + l * v;
            const bool axis = std::abs(v) < 0.01f;
            Line3D({ p0.x, p0.y, z }, { p1.x, p1.y, z }, axis ? CRGBA(80, 255, 80, 200) : CRGBA(255, 255, 255, 70));
        }
    }
    FlushLines();
    // from here with the depth test (the road under the cars, not over them)
    RwUInt32 zWrite = 0;
    RwRenderStateGet(rwRENDERSTATEZWRITEENABLE, &zWrite);
    RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)TRUE);
    RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, (void*)FALSE);
    RwRenderStateSet(rwRENDERSTATECULLMODE, (void*)rwCULLMODECULLNONE);
    forRunners {
    if (std::abs(R->baseZ + 0.08f - z) > 1.0f) continue; // (another deck's: seen through the floor)
    const CRGBA col = ColorOf(R->index);
    const int n = static_cast<int>(course.p.size());
    const Course& rd = R->road;
    if (rd.halfWidth > 0.0f && n > 4 && static_cast<int>(rd.p.size()) == n) {
        // A race track: asphalt between the edges (reddish where a sudden brake is, bluish in the crosswind), red and
        // white kerbs outside each edge, a white line on the edge, a chequered line at the start and the end
        const float H = rd.halfWidth, K = 0.6f, za = z + 0.04f, zk = z + 0.05f, zl = z + 0.06f;
        auto normal = [&](int i) { return CVector2D(-std::sin(rd.head[i]), std::cos(rd.head[i])); };
        for (int i = 0; i < n - 1; i += 4) {
            const int j = std::min(i + 4, n - 1);
            const CVector2D a = rd.p[i], b = rd.p[j];
            if (!close(a)) continue;
            const CVector2D la = normal(i), lb = normal(j);
            const CRGBA asphalt = rd.vsud[i] < 1e8f ? CRGBA(120, 45, 45, 235) : rd.wind[i] != 0.0f ? CRGBA(45, 70, 115, 235)
                : CRGBA(52, 54, 60, 235);
            const CVector2D al = a + la * H, ar = a - la * H, bl = b + lb * H, br = b - lb * H;
            Quad3D({ ar.x, ar.y, za }, { al.x, al.y, za }, { bl.x, bl.y, za }, { br.x, br.y, za }, asphalt);
            const CRGBA kerb = (i / 4) % 2 ? CRGBA(205, 35, 35, 255) : CRGBA(235, 235, 235, 255);
            for (float side : { -1.0f, 1.0f }) {
                const CVector2D ai = a + la * (side * H), ao = a + la * (side * (H + K)), bi = b + lb * (side * H), bo = b + lb * (side * (H + K));
                Quad3D({ ai.x, ai.y, zk }, { ao.x, ao.y, zk }, { bo.x, bo.y, zk }, { bi.x, bi.y, zk }, kerb);
                Line3D({ ai.x, ai.y, zl }, { bi.x, bi.y, zl }, CRGBA(255, 255, 255, 255));
            }
        }
        for (int at : { 0, n - 1 }) {
            if (!close(rd.p[at])) continue;
            // chequered: 2 rows of squares across the road
            const CVector2D o = rd.p[at], f(std::cos(rd.head[at]), std::sin(rd.head[at])), l = normal(at);
            const int cells = std::max(static_cast<int>(2.0f * H / 0.5f), 2);
            const float w = 2.0f * H / cells;
            for (int r = 0; r < 2; r++)
                for (int k = 0; k < cells; k++) {
                    const CVector2D p0 = o + l * (-H + k * w) + f * (r * w), p1 = p0 + l * w, p2 = p1 + f * w, p3 = p0 + f * w;
                    Quad3D({ p0.x, p0.y, zk }, { p1.x, p1.y, zk }, { p2.x, p2.y, zk }, { p3.x, p3.y, zk },
                        (k + r) % 2 ? CRGBA(20, 20, 20, 255) : CRGBA(240, 240, 240, 255));
                }
        }
        // the car's own line, in its colour
        for (int i = 0; i + 4 < n; i += 4)
            if (close(course.p[i])) Line3D({ course.p[i].x, course.p[i].y, z + 0.07f }, { course.p[i + 4].x, course.p[i + 4].y, z + 0.07f }, Alpha(col, 170));
    }
    else for (int i = 0; i + 1 < n + (course.closed ? 1 : 0); i += 2) {
        const CVector2D a = course.p[i], b = course.p[(i + 2) % n];
        if (!course.closed && i + 2 >= n) break;
        if (!close(a)) continue;
        // the sudden brakes in red, the posted limits in white
        // (and the crosswind in light blue)
        const CRGBA c = course.vsud[i] < 1e8f ? CRGBA(255, 40, 40, 255) : course.vcap[i] < 1e8f ? CRGBA(255, 255, 255, 230)
            : course.wind[i] != 0.0f ? CRGBA(120, 200, 255, 255) : col;
        Line3D({ a.x, a.y, z + 0.02f }, { b.x, b.y, z + 0.02f }, c);
    }
    for (const auto& c : course.cones) if (close(c)) Line3D({ c.x, c.y, z }, { c.x, c.y, z + 1.0f }, CRGBA(255, 120, 0, 255));
    for (size_t i = 1; i < trail.size(); i++)
        if (close(trail[i].To2D())) Line3D(trail[i - 1] + CVector(0, 0, -0.3f), trail[i] + CVector(0, 0, -0.3f), Alpha(col, 200));
    if (state == RUNNING && car && close(car->GetPosition().To2D())) {
        Line3D({ lookPoint.x, lookPoint.y, z }, { lookPoint.x, lookPoint.y, z + 2.5f }, CRGBA(255, 0, 0, 255));
        Line3D(car->GetPosition(), { lookPoint.x, lookPoint.y, z + 0.5f }, CRGBA(255, 0, 0, 160));
    }
    }
    FlushTris();
    FlushLines();
    RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)FALSE);
    RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, (void*)(uintptr_t)zWrite);
    R = runners;
    Perf::Scope perfHud(Perf::HUD);
    DrawHud();
}

} // namespace ProvingGround

#undef cfg
#undef course
#undef car
#undef carHandle
#undef driverHandle
#undef nearIdx
#undef steerCmd
#undef steerOut
#undef gasOut
#undef brakeOut
#undef handbrakeOut
#undef brakeDone
#undef targetKmh
#undef offsetNow
#undef headErrNow
#undef curvNow
#undef prevVel
#undef havePrevVel
#undef wheelBase
#undef rearAxleY
#undef aiGripG
#undef aiAdhesive
#undef absScale
#undef tcScale
#undef playerSteerFactor
#undef trail
#undef lookPoint
#undef abortWhy
#undef stats
#undef csv
#undef forRunners
#undef runT
