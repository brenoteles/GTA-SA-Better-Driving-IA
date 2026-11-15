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
#include "CSprite2d.h"

namespace ProvingGround {

static bool enabled = false;
static std::string iniPath, baseDir, runDir;
static FILETIME iniStamp{};
static unsigned int lastIniCheck = 0;
static std::ofstream summary;

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
static constexpr int kMaxRunners = 60;
static const char* const kTags[kMaxRunners] = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N",
    "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l",
    "m", "n", "o", "p", "q", "r", "s", "t", "u", "v", "w", "x", "y", "z", "0", "1", "2", "3", "4", "5", "6", "7" };
static int lanesPerDeck = 10;
static float edgeComfortGA = 1.0f; // a corridor: closer than this to an edge costs (stats.deficit; [GA] EdgeComfort)
static float deckHeight = 80.0f;
static void SetRunnerSettings(const Settings* rs);
static int numRunners = 1;
static float laneShiftB = 40.0f;
namespace GA {
static bool on = false;
static void Read(CIniReader& ini, const Settings& s);
static void Start();
static void Score(int k, const std::string& why, float time, double mean, double rms, double maxE, double out, float sMax, float length,
    double deficit, double minMargin);
static void NextGeneration();
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
    std::vector<std::pair<float, std::string>> sections; // track: where each named part starts (m)
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
    std::vector<CVector> trail;
    CVector2D lookPoint;
    std::string abortWhy;
    Stats stats;
    std::ofstream csv;
    Course road;                    // the road itself (course is then the car's own line on it: OwnLine)
    // the run so far (HUD, GA)
    std::string result;             // how its run ended ("" while it runs)
    float endTime = 0.0f, sMax = 0.0f, slowTime = 0.0f;
    float progressS = 0.0f, progressT = 0.0f; // the last 5 m of progress: where and when
    float latGNow = 0.0f, slipNow = 0.0f, speedNow = 0.0f;
    bool skidFNow = false, skidRNow = false, absOn = false, tcOn = false;
    float windNow = 0.0f;
    float bodyFront = 2.2f, bodyRear = -2.2f, bodyHalfW = 1.0f; // the body's corners (m, car frame)
    float marginNow = 99.0f;        // a corridor: the room between its nearest corner and the nearest edge (m)
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

// The car's own line inside the road (its genes LineBias, LineCut): every point moved across by the room x (bias + cut
// x the curve's direction ahead and behind), smoothed over +-10 m (no step for the wheel), never past the room. The
// car steers along it and plans its speed on it (a straighter line is a faster one); the road (R->road) stays what
// its body is judged against
static void OwnLine() {
    const int n = static_cast<int>(course.p.size());
    if (course.halfWidth <= 0.0f || n < 50 || (cfg.lineBias == 0.0f && cfg.lineCut == 0.0f)) return;
    const float room = std::max(course.halfWidth - R->bodyHalfW - 0.1f, 0.0f);
    std::vector<float> o(n), t(n);
    for (int i = 0; i < n; i++) {
        float k = 0.0f;
        int m = 0;
        for (int j = std::max(i - 24, 0); j <= std::min(i + 24, n - 1); j++, m++) k += course.curv[j];
        k /= std::max(m, 1);
        o[i] = room * std::clamp(cfg.lineBias + cfg.lineCut * std::clamp(k * 25.0f, -1.0f, 1.0f), -1.0f, 1.0f);
    }
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < n; i++) {
            float sum = 0.0f;
            int m = 0;
            for (int j = std::max(i - 20, 0); j <= std::min(i + 20, n - 1); j++, m++) sum += o[j];
            t[i] = sum / std::max(m, 1);
        }
        o.swap(t);
    }
    for (int i = 0; i < n; i++)
        course.p[i] += CVector2D(-std::sin(course.head[i]), std::cos(course.head[i])) * std::clamp(o[i], -room, room);
    Geometry(course);
}

static void BuildCourse(const TestDef& d) {
    course = Course();
    frameOrigin = d.hasStart ? d.start : CVector2D(cfg.origin.x, cfg.origin.y);
    frameHeading = d.hasStart ? d.startHeading : cfg.headingDeg;
    if (R->laneShift != 0.0f) {
        const float a = frameHeading * 3.14159265f / 180.0f;
        frameOrigin += CVector2D(std::sin(a), -std::cos(a)) * R->laneShift;
    }
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
            else if ((kind == 'V' || kind == 'B' || kind == 'X') && nums.size() >= 2)
                zones.push_back({ static_cast<int>(local.size()) - 1, nums[1], nums[0], kind });
            else if (kind == 'W' && nums.size() >= 3) {
                const float amp = nums[0], sp = std::max(nums[1], 2.0f);
                const int gates = std::max(static_cast<int>(std::lround(nums[2])), 1);
                for (int k = 0; k < gates; k++) course.cones.push_back(ToWorld(inFrame(sp * (k + 0.5f), 0.0f)));
                sideways(sp * gates, [&](float x) { return amp * std::sin(kPi * x / sp); });
            }
            else if (kind == 'J' && nums.size() >= 4) {
                const float a = nums[0], o = std::max(nums[1], 1.0f), ho = nums[2], b = std::max(nums[3], 1.0f);
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
    for (const Zone& z : zones)
        for (int i = std::clamp(z.i, 0, n - 1); i < n && course.s[i] - course.s[z.i] <= z.len; i++) {
            if (z.kind == 'X') { course.wind[i] = z.value; continue; }
            float& v = z.kind == 'B' ? course.vsud[i] : course.vcap[i];
            v = std::min(v, z.value / 3.6f);
        }
    for (const auto& nm : names) course.sections.emplace_back(course.s[std::clamp(nm.first, 0, n - 1)], nm.second);
    R->road = course;
    OwnLine();
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
static float runT = 0.0f;
static const TestDef* curTest = nullptr;
static int curModel = 0;
static std::ofstream trace;            // (Trace)
static float traceNext[kMaxRunners];
static bool courseWritten = false;
static int camPick = -1;               // (UpdateCamera) -1: the leader
static int camOn = 0;                  // the car the camera is on


static unsigned int Now() { return CTimer::m_snTimeInMilliseconds; }

static void Log(const std::string& s) {
    if (summary.is_open()) { summary << s << "\n"; summary.flush(); }
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
    if (summary.is_open()) summary.close();
    summary.open(runDir + "summary.txt", std::ios::trunc);
    std::ofstream latest(baseDir + "proving\\latest.txt", std::ios::trunc);
    latest << stamp << "\n";
    CopyFileA(iniPath.c_str(), (runDir + "proving.ini").c_str(), FALSE);
}

static void Restart() {
    forRunners { RemoveCar(); if (csv.is_open()) csv.close(); }
    R = runners;
    if (trace.is_open()) trace.close();
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
    else if (cfg.mode == "kinematic") {
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
    else { // pure pursuit
        const float ld = std::clamp(cfg.lookTime * vv, cfg.lookMin, cfg.lookMax);
        const CVector2D tp = course.p[Advance(i, ld)];
        const CVector2D rel = tp - pos;
        const float alpha = std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd));
        d = std::atan2(2.0f * L * std::sin(alpha), ld);
        lookPoint = tp;
    }
    // (Gain only on the pursuit-based modes: kinematic's and stanley's wheel holds the curvature's exact feedforward)
    return cfg.mode == "pursuit" || cfg.mode == "mod" ? d * cfg.gain : d;
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
    if (gripG > 0.0f && !openLoop) {
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
        const float allowed = pedalMax * absScale * R->absLearn;
        R->absOn = brake > allowed;
        brake = std::min(brake, allowed);
    }
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
        gas = std::min(gas, std::clamp(room * (cNow > 1.0f / 400.0f ? R->tcBand[CurveBand(cNow)] : 1.0f), cfg.tcMinGas, 1.0f));
        // sliding: at the rear axle (at the car's centre a tight turn at full lock reads ~0.25 rad with the tyres holding:
        // the gas went off and the car stopped in the 7 m U-turn), and not at walking pace
        const float vRightRear = CVector::Dot(v->m_vecMoveSpeed, v->GetRight()) - v->m_vecTurnSpeed.z * rearAxleY;
        const float slip = std::abs(std::atan2(vRightRear, std::max(std::abs(CVector::Dot(v->m_vecMoveSpeed, v->GetForward())), 0.02f)));
        const bool sliding = slip > cfg.tcSlip && speed > 3.0f;
        if (sliding) gas = 0.0f;
        // wheels spinning (4WD out of a tight turn: all four at full gas): the gas comes off until they grip again
        CAutomobile* a = static_cast<CAutomobile*>(v);
        bool spinning = false;
        for (int w = 0; w < 4; w++) if (a->m_wheelState[w] == 1) spinning = true;
        tcScale = spinning ? std::max(tcScale - cfg.tcDrop, 0.2f) : std::min(tcScale + cfg.tcRecover, 1.0f);
        gas *= tcScale;
        R->tcOn = gas < gasAsked - 0.05f;
    }
    if (!openLoop && cfg.learnRate > 0.0f && speed > 5.0f) LearnByResult(speed, dt, brakeAsked);
    if (d.handbrakeAt && tp >= 0.0f) hb = true;
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
        if (k == 0 && cfg.driver == "player") break; // (the player's car: after its inputs)
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
    if (course.halfWidth > 0.0f && !course.p.empty()) {
        // The road is a corridor between two edges: what has to stay inside is the whole body, not its centre. Each of
        // its four corners on the course (in a curve the body sweeps wider than its centre's path, and the room left
        // shrinks); the room = the edge's distance from the corner nearest to it
        const CVector2D c0 = v->GetPosition().To2D(), f = v->GetForward().To2D(), rt(f.y, -f.x);
        float worst = 0.0f;
        for (float fy : { R->bodyFront, R->bodyRear })
            for (float sx : { -1.0f, 1.0f }) {
                const CVector2D corner = c0 + f * fy + rt * (sx * R->bodyHalfW);
                const OnCourse oc = ProjectIn(R->road, corner, NearestIndexIn(R->road, corner, nearIdx, 16));
                worst = std::max(worst, std::abs(oc.offset));
            }
        R->marginNow = course.halfWidth - worst;
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
    { "Steer", "Bandwidth", &Settings::bandwidth, 0.5f, 4.0f, "kinematic" },
    { "Steer", "Damping", &Settings::damping, 0.4f, 2.0f, "kinematic" },
    { "Steer", "Preview", &Settings::preview, 0.0f, 0.3f, "kinematic stanley" },
    // (LookaheadTime and Gain from 0.2 and 0.7: generations 2-4 converged on those lower ends)
    { "Steer", "LookaheadTime", &Settings::lookTime, 0.0f, 1.5f, "pursuit mod" }, // (mod: its fallback)
    { "Steer", "LookaheadMin", &Settings::lookMin, 1.5f, 15.0f, "pursuit mod" }, // (from 3: 3.2 m strayed 3x less than 5.3 m in the U-turn)
    { "Steer", "StanleyK", &Settings::stanleyK, 0.3f, 5.0f, "stanley" },
    { "Steer", "Gain", &Settings::gain, 0.5f, 1.3f, "pursuit mod" },
    { "Steer", "Smoothing", &Settings::smoothing, 0.0f, 0.12f, "" },
    { "Speed", "GripFactor", &Settings::gripFactor, 0.3f, 0.75f, "" }, // curve speeds: share of the AI grip formula
    { "Speed", "BrakeG", &Settings::speedBrakeG, 0.5f, 1.6f, "" },    // braking planned for curves and limits
    { "Speed", "Kp", &Settings::speedKp, 0.1f, 1.0f, "" },
    { "Speed", "BrakeKp", &Settings::brakeKp, 0.2f, 2.5f, "" },   // (to 2.5: 10 of 40 at 1.5, the 1st among them)
    { "Steer", "LineBias", &Settings::lineBias, -1.0f, 1.0f, "" },  // its own line: to one side of the road
    { "Steer", "LineCut", &Settings::lineCut, -0.5f, 1.0f, "" },    // ... into the inside of the curves
    { "Speed", "GripTight", &Settings::gripTight, 0.35f, 1.6f, "" }, // the grip in tight curves (<= 15 m) x this (from 0.35: 24 of 60 at 0.6)
    { "Speed", "GripFast", &Settings::gripFast, 0.6f, 1.6f, "" },   // ... in fast ones (>= 150 m)
    { "Speed", "LearnRate", &Settings::learnRate, 0.0f, 0.4f, "" }, // how fast it learns curves, ABS and TC itself
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
struct Genome { int mode = 0; float g[kNumGenes] = {}; std::string label; double sum = 0.0; int runs = 0; };
struct Result { double score = 0.0, time = 0.0, mean = 0.0, rms = 0.0, maxE = 0.0, out = 0.0, reached = 0.0, deficit = 0.0, minMargin = 1e9; std::string why; int runs = 0; };

static int elite = 2, tournament = 3, immigrants = 1, maxGenerations = 0;
static float mutRate = 0.3f, sigma0 = 0.15f, sigmaDecay = 0.96f, sigmaMin = 0.03f, modeMutation = 0.1f;
static float meanPenalty = 100.0f, offPenalty = 5.0f, rmsPenalty = 0.0f, edgePenalty = 1.0f;
static bool resume = true;
static unsigned int seed = 0;
static std::vector<std::string> modes{ "kinematic", "stanley", "pursuit", "mod" };
static Settings base;
static Genome pop[kMaxRunners];
static Result res[kMaxRunners];
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
    offPenalty = ini.ReadFloat("GA", "OffPenalty", offPenalty);
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

static void Apply() {
    for (int k = 0; k < numRunners; k++) {
        Settings s = ToSettings(pop[k]);
        s.driver = base.driver;
        CfgOf(k) = s;
    }
}

static void Save() {
    std::ofstream f(StatePath(), std::ios::trunc);
    f << "genes";
    for (const auto& gd : kGenes) f << " " << gd.key;
    f << "\ngeneration " << generation << "\nsigma " << sigma << "\n";
    if (bestGen >= 0) f << "best " << bestScore << " " << bestGen << " " << modes[best.mode] << " " << GenesText(best, " ") << "\n"
                        << "bestrun " << bestRes.time << " " << bestRes.mean << " " << bestRes.maxE << " " << bestRes.out << "\n";
    f << "history";
    for (double h : history) f << " " << h;
    f << "\n";
    for (int k = 0; k < numRunners; k++)
        f << "ind " << modes[pop[k].mode] << " " << GenesText(pop[k], " ") << (pop[k].label.empty() ? "" : " | " + pop[k].label) << "\n";
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
        else if (key == "bestrun") is >> bestRes.time >> bestRes.mean >> bestRes.maxE >> bestRes.out;
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
    std::ofstream f(baseDir + "proving\\ga_best.ini", std::ios::trunc);
    f << Format("; GA best ever: generation %d, score %.2f (lower is better)\n", bestGen, bestScore);
    for (const char* sec : { "Steer", "Speed" }) {
        f << "[" << sec << "]\n";
        if (!strcmp(sec, "Steer")) f << "Mode = " << modes[best.mode] << "\n";
        for (int i = 0; i < kNumGenes; i++) if (!strcmp(kGenes[i].sec, sec)) f << kGenes[i].key << " = " << Format("%.4g", best.g[i]) << "\n";
    }
}

static void Start() {
    if (!on) return;
    rng.seed(seed ? seed : GetTickCount());
    generation = 0;
    sigma = sigma0;
    history.clear();
    bestScore = 1e18;
    bestGen = -1;
    bestRes = Result();
    const bool resumed = resume && Load();
    if (!resumed) {
        // the ini's own controller, then one of each algorithm, then random ones
        pop[0] = FromSettings(base);
        for (int k = 1; k < numRunners; k++) {
            pop[k] = Random();
            if (k <= static_cast<int>(modes.size())) pop[k].mode = (k - 1) % static_cast<int>(modes.size());
        }
    }
    for (int k = 0; k < kMaxRunners; k++) res[k] = Result();
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
    r.score += done ? time + meanPenalty * mean + edgePenalty * deficit + offPenalty * out + rmsPenalty * rms
                    : 1000.0 + 0.5 * std::max(length - sMax, 0.0f) + offPenalty * out;
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

static void NextGeneration() {
    const int n = numRunners;
    // ranked by the average of the generations each genome drove (a new one: this run)
    std::vector<double> avg(n);
    for (int k = 0; k < n; k++) {
        pop[k].sum += res[k].score;
        pop[k].runs++;
        if (pop[k].runs > 5) { pop[k].sum *= 5.0 / 6.0; pop[k].runs = 5; } // (the last ~5)
        avg[k] = pop[k].sum / pop[k].runs;
    }
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
            r + 1, kTags[k], modes[pop[k].mode].c_str(), x.score, x.why.c_str(), x.time, x.mean, x.maxE, x.minMargin < 1e8 ? x.minMargin : 0.0,
            x.deficit, x.out, ShortText(pop[k]).c_str()));
        h << Format("%s,%d,%d,%s,%s,%.3f,%s,%.2f,%.3f,%.3f,%.3f,%.2f,%.0f,%.4f,", stamp, generation, r + 1, kTags[k], modes[pop[k].mode].c_str(),
            x.score, x.why.c_str(), x.time, x.mean, x.rms, x.maxE, x.out, x.reached, sigma) << GenesText(pop[k], ",") << "," << pop[k].label
          << Format(",%.3f,%d,%.3f,%.3f", avg[k], pop[k].runs, x.deficit, x.minMargin < 1e8 ? x.minMargin : 0.0) << "\n";
    }

    // the next generation: elite first (lane A, CJ's car, is the best), children, immigrants
    Genome next[kMaxRunners];
    const int e = std::clamp(elite, 0, n);
    const int imm = std::clamp(immigrants, 0, std::max(n - e - 1, 0));
    for (int i = 0; i < e; i++) next[i] = pop[order[i]];
    // and the best that finished of every other algorithm (in one generation the minorities were gone: a mode has to
    // be tuned before it can compete)
    int kept = e;
    std::vector<int> minorities;
    for (int m = 0; m < static_cast<int>(modes.size()) && kept < n - imm; m++) {
        bool inElite = false;
        for (int i = 0; i < e; i++) inElite |= next[i].mode == m;
        if (inElite) continue;
        for (int r = e; r < n; r++)
            if (pop[order[r]].mode == m && res[order[r]].why == "end") { next[kept++] = pop[order[r]]; minorities.push_back(m); break; }
    }
    // (kept, a minority never won the tournament against 30 pursuits and never changed: one child of its own a
    // generation, from parents of its algorithm)
    size_t minorityChild = 0;
    for (int i = kept; i < n - imm; i++) {
        const int own = minorityChild < minorities.size() ? minorities[minorityChild++] : -1;
        // (a minority's own child: one parent of its algorithm, the other the best overall: it keeps its algorithm and
        // takes the shared genes, speed / ABS / TC, from the best; S1 of Gemini's 5th review)
        const Genome& a = pop[Tournament(avg, own)];
        const Genome& b = pop[Tournament(avg, -1)];
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
        // only the genes its algorithm reads (a third of the mutations went to genes the mode ignores)
        for (int g = 0; g < kNumGenes; g++)
            if (Uses(g, c.mode) && U() < mutRate)
                c.g[g] = std::clamp(c.g[g] + N01() * sigma * (kGenes[g].hi - kGenes[g].lo), kGenes[g].lo, kGenes[g].hi);
        next[i] = c;
    }
    for (int i = n - imm; i < n; i++) next[i] = Random();
    for (int k = 0; k < n; k++) { pop[k] = next[k]; res[k] = Result(); }
    generation++;
    sigma = std::max(sigma * sigmaDecay, sigmaMin);
    Save();
    Apply();
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
    else if (cfg.driver == "player" || cfg.driver == "cjai") {
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

static void OpenTrace() {
    const std::string dir = GA::on ? baseDir + "proving\\ga_trace\\" : runDir;
    CreateDirectoryA(dir.c_str(), nullptr);
    if (trace.is_open()) trace.close();
    trace.open(dir + (GA::on ? Format("gen_%04d.csv", GA::generation) : std::string("trace.csv")), std::ios::trunc);
    trace << "generation,car,mode,t,x,y,speed_kmh,target_kmh,offset,steer,gas,brake,s,lat_g,slip,abs,tc,margin\n";
    for (float& t : traceNext) t = 0.0f;
    if (courseWritten || !curTest) return;
    courseWritten = true;
    Runner* keep = R;
    R = runners;
    std::ofstream c(dir + "R->road.csv", std::ios::trunc);
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
    if (!trace.is_open() || runT < traceNext[R->index]) return;
    traceNext[R->index] = runT + 0.25f;
    const CVector2D q = ToLane(v->GetPosition().To2D());
    trace << Format("%d,%s,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.3f,%.2f,%.2f,%.1f,%.2f,%.3f,%d,%d,%.2f\n", GA::on ? GA::generation : runCount,
        R->tag, cfg.mode.c_str(), runT, q.x, q.y, R->speedNow, targetKmh, offsetNow, v->m_fSteerAngle, gasOut, brakeOut, R->sMax,
        R->latGNow, R->slipNow, R->absOn ? 1 : 0, R->tcOn ? 1 : 0, course.halfWidth > 0.0f ? R->marginNow : 0.0f);
}

// --- Camera ----------------------------------------------------------------------------------------------------------
// On the leading car (Home), or the one picked with PageUp / PageDown; car A is CJ's, the game's own camera
static unsigned int camSince = 0;

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

static void UpdateCamera(unsigned int now) {
    static bool prevUp = false, prevDown = false, prevHome = false;
    const bool up = KeyPressed(VK_PRIOR) != 0, down = KeyPressed(VK_NEXT) != 0, home = KeyPressed(VK_HOME) != 0;
    int want = camOn;
    if (down && !prevDown) camPick = want = (camOn + 1) % numRunners;
    if (up && !prevUp) camPick = want = (camOn + numRunners - 1) % numRunners;
    if (home && !prevHome) camPick = -1;
    prevUp = up; prevDown = down; prevHome = home;
    if (camPick < 0) {
        // the first car still running (no more than one change every 4 s)
        for (int k : Standings()) if (runners[k].result.empty()) { want = k; break; }
        if (want != camOn && now - camSince < 4000) want = camOn;
    }
    if (want != camOn) { PointCamera(want); camSince = now; }
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
        forRunners RemoveCar();
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
        runT = 0.0f;
        forRunners {
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
        runT += dt;
        bool allDone = true;
        forRunners {
        if (!car || !CPools::ms_pVehiclePool->IsObjectValid(car)) { Log("car lost"); state = NEXT_CAR; return; }
        // CJ at the wheel of a car driven by the AI's code: the game's STATUS_PHYSICS (AI grip, no player steering
        // limit, CCarCtrl::SteerAICarWithPhysics -> Control) before every physics frame
        // (and the other cars: far from the player the game would put them on its simple, rail-like movement)
        if (R != runners || cfg.driver == "cjai") car->m_nStatus = STATUS_PHYSICS;
        if (R->finished) {
            if (R->result != "end") { car->m_vecMoveSpeed = CVector(0.0f, 0.0f, 0.0f); car->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f); }
            continue;
        }
        allDone = false;
        Record(car, dt);
        Trace(car);
        if (trail.empty() || (car->GetPosition() - trail.back()).Magnitude() > 1.0f) {
            trail.push_back(car->GetPosition());
            if (trail.size() > 1500) trail.erase(trail.begin());
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
        if (end) {
            EndRun(end);
            R->finished = true;
        }
        }
        if (allDone) {
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
    const std::vector<int> order = Standings();

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
        Text(cMode, y + 4.0f, GA::on && !GA::pop[k].label.empty() ? cfg.mode + " - " + GA::pop[k].label : cfg.mode, ts, kWhite);
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
        Text(x + 20.0f, y + 38.0f, Format("geracao %d   populacao %d   sigma %.3f", GA::generation, numRunners, GA::sigma), 0.78f, kDim);
        if (course.halfWidth > 0.0f)
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
        if (bestNow >= 0) {
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
