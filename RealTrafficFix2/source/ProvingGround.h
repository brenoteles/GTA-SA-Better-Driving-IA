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

static void SetRunnerSettings(const Settings& a, const Settings& b);
static int numRunners = 1;
static float laneShiftB = 40.0f;

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
        d.segments = ini.ReadString(sec, "Segments", "");
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
    // A/B: a second car on the same course, LaneShift m to the right, with [Steer.B] / [Speed.B] over A's settings
    numRunners = ini.ReadInteger("Proving", "AB", 0) ? 2 : 1;
    laneShiftB = ini.ReadFloat("Proving", "LaneShift", laneShiftB);
    Settings sb = s;
    control(sb, ".B");
    SetRunnerSettings(s, sb);
}

// --- Courses ---------------------------------------------------------------------------------------------------------

struct Course {
    std::vector<CVector2D> p;      // world, every ~0.5 m
    std::vector<float> s, head, curv; // m along, direction (rad, world), curvature (1/m, > 0 left)
    std::vector<CVector2D> cones;  // world
    bool closed = false;
    float length = 0.0f;
};

struct Stats {
    double n = 0, absE = 0, sqE = 0, maxE = 0, speedErr = 0, maxLatG = 0, steerAbs = 0, reversals = 0;
    double skidFront = 0, skidRear = 0, spin = 0, time = 0;
    double gainSum = 0, gainN = 0;  // yaw rate / (v tan(steer)): 1/L of the car's turn while the tyres hold
    float lastSteerDelta = 0.0f, lastSign = 0.0f;
    float maxSpeed = 0.0f;
    float t100 = 0.0f;              // full-throttle tests: time to 100 km/h
    float stopDist = 0.0f;          // brake tests
};

// One car on the course (A, and B for A/B tests): its settings, course, car and controller state
struct Runner {
    const char* tag = "A";
    float laneShift = 0.0f;         // m to the right of the test's course
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
};
static Runner runners[2];
static Runner* R = &runners[0];
static void SetRunnerSettings(const Settings& a, const Settings& b) {
    runners[0].cfg = a;
    runners[1].cfg = b;
    runners[0].tag = "A";
    runners[1].tag = "B";
    runners[1].laneShift = laneShiftB;
}
static Settings& CfgOf(int k) { return runners[k].cfg; }
static CVehicle* CarOf(int k) { return runners[k].car; }
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

static void BuildCourse(const TestDef& d) {
    course = Course();
    frameOrigin = d.hasStart ? d.start : CVector2D(cfg.origin.x, cfg.origin.y);
    frameHeading = d.hasStart ? d.startHeading : cfg.headingDeg;
    if (R->laneShift != 0.0f) {
        const float a = frameHeading * 3.14159265f / 180.0f;
        frameOrigin += CVector2D(std::sin(a), -std::cos(a)) * R->laneShift;
    }
    std::vector<CVector2D> local;
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
        // straights and arcs one after the other: "S 300; L 90 30; R 45 120"
        CVector2D at(0.0f, 0.0f);
        float h = 0.0f;
        local.push_back(at);
        std::string seg;
        // (separated by '/': the ini reader cuts a value at ';')
        for (char c : d.segments + "/") {
            if (c != '/' && c != ';') { seg += c; continue; }
            char kind = 0;
            size_t k0 = 0;
            while (k0 < seg.size() && (seg[k0] == ' ' || seg[k0] == '\t')) k0++;
            if (k0 < seg.size()) kind = static_cast<char>(toupper(seg[k0]));
            auto nums = ParseFloats(k0 + 1 < seg.size() ? seg.substr(k0 + 1) : "");
            seg.clear();
            if (kind == 'S' && !nums.empty()) {
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
    course.s.resize(n);
    course.head.resize(n);
    course.curv.resize(n);
    for (int i = 0; i < n; i++) course.s[i] = i ? course.s[i - 1] + (course.p[i] - course.p[i - 1]).Magnitude() : 0.0f;
    course.length = course.s[n - 1] + (course.closed ? (course.p[0] - course.p[n - 1]).Magnitude() : 0.0f);
    auto idx = [&](int i) { return course.closed ? ((i % n) + n) % n : std::clamp(i, 0, n - 1); };
    for (int i = 0; i < n; i++) {
        const CVector2D t = course.p[idx(i + 2)] - course.p[idx(i - 2)];
        course.head[i] = std::atan2(t.y, t.x);
    }
    for (int i = 0; i < n; i++) {
        // over +-3 m (6 points each way)
        const int a = idx(i - 6), b = idx(i + 6);
        const float ds = (course.p[b] - course.p[i]).Magnitude() + (course.p[i] - course.p[a]).Magnitude();
        course.curv[i] = ds > 0.5f ? WrapPi(course.head[b] - course.head[a]) / ds : 0.0f;
    }
}

// Nearest course point to 'pos', searched around the last one (the course may cross itself: figure 8)
static int NearestIndex(const CVector2D& pos, int hint, int window) {
    const int n = static_cast<int>(course.p.size());
    int best = hint;
    float bestD = 1e18f;
    for (int k = -window; k <= window; k++) {
        int i = hint + k;
        if (course.closed) i = ((i % n) + n) % n;
        else if (i < 0 || i >= n) continue;
        const float d = (course.p[i] - pos).MagnitudeSqr();
        if (d < bestD) { bestD = d; best = i; }
    }
    return best;
}
// Where 'pos' is on the course between its points: offset (> 0 left), direction and curvature interpolated (the points'
// own values step every 0.5 m, and the heading feedback shook the wheel every frame on those steps)
struct OnCourse { int i; float frac, offset, head, curv; };
static OnCourse Project(const CVector2D& pos, int hint) {
    const int n = static_cast<int>(course.p.size());
    auto at = [&](int k) { return course.closed ? ((k % n) + n) % n : std::clamp(k, 0, n - 1); };
    OnCourse r{ hint, 0.0f, 0.0f, course.head[hint], course.curv[hint] };
    for (int a : { at(hint - 1), hint }) {
        const int b = at(a + 1);
        if (a == b) continue;
        const CVector2D ab = course.p[b] - course.p[a];
        const float len2 = ab.MagnitudeSqr();
        if (len2 < 1e-6f) continue;
        const float t = CVector2D::Dot(pos - course.p[a], ab) / len2;
        if (t < 0.0f || t > 1.0f) continue;
        r.i = a;
        r.frac = t;
        r.head = course.head[a] + WrapPi(course.head[b] - course.head[a]) * t;
        r.curv = course.curv[a] + (course.curv[b] - course.curv[a]) * t;
        break;
    }
    r.offset = CVector2D::Dot(pos - course.p[hint], CVector2D(-std::sin(r.head), std::cos(r.head)));
    return r;
}

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
    ReadSettings();
    OpenRunDir();
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
        const CVector2D fd(std::cos(course.head[fi]), std::sin(course.head[fi]));
        const float ef = CVector2D::Dot(front - course.p[fi], CVector2D(-fd.y, fd.x));
        const float pf = WrapPi(carHead - course.head[fi]);
        d = -pf - std::atan(cfg.stanleyK * ef / (vv + cfg.stanleySoft));
        if (cfg.feedForward) d += std::atan(L * course.curv[Advance(fi, cfg.preview * vv)]);
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
    return d * cfg.gain;
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
    nearIdx = NearestIndex(v->GetPosition().To2D(), nearIdx, 120);
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
        const float reach = std::clamp(speed * speed / (2.0f * brakeG * 9.81f) + 60.0f, 60.0f, 600.0f);
        for (float a = 0.0f; a < reach; a += 2.0f) {
            const int k = Advance(nearIdx, a);
            if (!course.closed && k >= static_cast<int>(course.p.size()) - 1) break;
            const float c = std::abs(course.curv[k]);
            if (c < 1e-4f) continue;
            const float vc = std::sqrt(gripG * 9.81f / c);
            vt = std::min(vt, std::sqrt(vc * vc + 2.0f * brakeG * 9.81f * a));
        }
    }
    targetKmh = vt * 3.6f;
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
        float pedalMax = 0.9f * std::min(lockF, lockR);
        const float G = aiGripG * 9.81f;
        const float aLat = std::abs(speed * v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS);
        pedalMax *= std::sqrt(std::max(1.0f - (aLat / G) * (aLat / G), 0.04f));
        // a wheel locked anyway: off a little more until it turns again
        CAutomobile* a = static_cast<CAutomobile*>(v);
        bool locked = false;
        for (int w = 0; w < 4; w++) if (a->m_wheelState[w] == 3) locked = true;
        absScale = locked ? std::max(absScale - 0.15f, 0.3f) : std::min(absScale + 0.05f, 1.0f);
        brake = std::min(brake, pedalMax * absScale);
    }
    if (!openLoop && cfg.tractionControl) {
        // The tyres share one grip between turning and driving (ProcessWheel: the thrust and the sideways pull are scaled
        // down together past the adhesion): gas only with the grip the curve leaves, none while sliding
        const float G = (gripG > 0.0f ? gripG : cfg.gripFactor * aiGripG) * 9.81f;
        const float aLat = std::abs(speed * v->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS);
        const float room = std::sqrt(std::max(G * G - aLat * aLat, 0.0f)) / G;
        gas = std::min(gas, std::max(room, 0.15f));
        const float slip = std::abs(std::atan2(CVector::Dot(v->m_vecMoveSpeed, v->GetRight()), std::max(std::abs(CVector::Dot(v->m_vecMoveSpeed, v->GetForward())), 0.02f)));
        if (slip > 0.08f) gas = 0.0f;
        // wheels spinning (4WD out of a tight turn: all four at full gas): the gas comes off until they grip again
        CAutomobile* a = static_cast<CAutomobile*>(v);
        bool spinning = false;
        for (int w = 0; w < 4; w++) if (a->m_wheelState[w] == 1) spinning = true;
        tcScale = spinning ? std::max(tcScale - 0.1f, 0.2f) : std::min(tcScale + 0.03f, 1.0f);
        gas *= tcScale;
    }
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

static void StartCsv() {
    if (csv.is_open()) csv.close();
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
    csv << Format("%.3f,%.4f,%.2f,%.2f,%.2f,%.4f,%.2f,%.3f,%.3f,%.4f,%.2f,%.2f,%.4f,%.4f,%.2f,%.2f,%d,%d,%.3f,%.4f,%.5f,%.1f,%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%.1f,%d,%.4f,%d\n",
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
    if (!abortWhy.empty()) line += " | " + abortWhy;
    Log(line);
}

static CVehicle* MakeCar(int model) {
    const CVector2D start = course.p.empty() ? CVector2D(cfg.origin.x, cfg.origin.y) : course.p[0];
    Command<COMMAND_CLEAR_AREA>(start.x, start.y, cfg.origin.z, 30.0f, 0);
    Command<COMMAND_CREATE_CAR>(model, start.x, start.y, cfg.origin.z + 1.0f, &carHandle);
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
    float z = FindGroundZ({ p0.x, p0.y, cfg.origin.z + 3.0f }, &found);
    if (!found) z = cfg.origin.z;
    Command<COMMAND_SET_CAR_COORDINATES>(carHandle, p0.x, p0.y, z + 0.5f);
    Command<COMMAND_SET_CAR_HEADING>(carHandle, head * 180.0f / 3.14159265f - 90.0f);
    v->m_vecMoveSpeed = CVector(std::cos(head), std::sin(head), 0.0f) * (kmh / 3.6f / GAME_SPEED_TO_MPS);
    v->m_vecTurnSpeed = CVector(0.0f, 0.0f, 0.0f);
    v->m_fSteerAngle = 0.0f;
}

// --- Per frame -------------------------------------------------------------------------------------------------------

void Init(const std::string& moduleDir) {
    baseDir = moduleDir;
    iniPath = moduleDir + "RealTrafficFix.proving.ini";
    if (GetFileAttributesA(iniPath.c_str()) == INVALID_FILE_ATTRIBUTES) return;
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
            CColStore::RequestCollision(CVector(p0.x, p0.y, cfg.origin.z), 0);
            CColStore::AddCollisionNeededAtPosn(CVector(p0.x, p0.y, cfg.origin.z));
            CColPoint cp;
            CEntity* hit = nullptr;
            const bool ground = CWorld::ProcessVerticalLine(CVector(p0.x, p0.y, cfg.origin.z + 5.0f), cfg.origin.z - 10.0f, cp, hit,
                true, false, false, false, true, false, nullptr);
            if (!ground) {
                if (now - stateTime > 8000) { Log("no ground at the start of this course: check Origin / Start"); state = NEXT_CAR; }
                else if (now - stateTime > 1500 && now - stateTime < 1600) Command<COMMAND_LOAD_SCENE>(p0.x, p0.y, cfg.origin.z);
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
            PlaceCar(car, curTest->startSpeed >= 0.0f ? curTest->startSpeed : curTest->speeds[speedIdx]);
            brakeDone = false;
            StartCsv();
        }
        state = RUNNING;
        stateTime = now;
        return;
    }

    case RUNNING: {
        const float dt = CTimer::ms_fTimeStep / GAME_SPEED_TO_MPS;
        if (CTimer::m_UserPause || CTimer::m_CodePause) return;
        runT += dt;
        bool allDone = true;
        forRunners {
        if (!car || !CPools::ms_pVehiclePool->IsObjectValid(car)) { Log("car lost"); state = NEXT_CAR; return; }
        // CJ at the wheel of a car driven by the AI's code: the game's STATUS_PHYSICS (AI grip, no player steering
        // limit, CCarCtrl::SteerAICarWithPhysics -> Control) before every physics frame
        if (cfg.driver == "cjai" && R == runners) car->m_nStatus = STATUS_PHYSICS;
        if (R->finished) continue;
        allDone = false;
        Record(car, dt);
        if (trail.empty() || (car->GetPosition() - trail.back()).Magnitude() > 1.0f) {
            trail.push_back(car->GetPosition());
            if (trail.size() > 1500) trail.erase(trail.begin());
        }
        const float speed = car->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS;
        if (curTest->pedal == "brake" && runT >= curTest->pedalStart && !brakeDone) stats.stopDist += speed * dt;
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
        else if (car->GetPosition().z < cfg.origin.z - 5.0f) end = "FELL";
        if (end) {
            EndRun(end);
            R->finished = true;
        }
        }
        if (allDone) {
            speedIdx++;
            if (speedIdx >= static_cast<int>(curTest->speeds.size())) { speedIdx = 0; testIdx++; }
            if (testIdx >= static_cast<int>(CfgOf(0).tests.size())) { state = NEXT_CAR; return; }
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

static void Line3D(const CVector& a, const CVector& b, CRGBA c) {
    RwIm3DVertex v[2];
    RwIm3DVertexSetPos(&v[0], a.x, a.y, a.z);
    RwIm3DVertexSetPos(&v[1], b.x, b.y, b.z);
    RwIm3DVertexSetRGBA(&v[0], c.r, c.g, c.b, c.a);
    RwIm3DVertexSetRGBA(&v[1], c.r, c.g, c.b, c.a);
    if (RwIm3DTransform(v, 2, nullptr, 0)) {
        RwIm3DRenderLine(0, 1);
        RwIm3DEnd();
    }
}

void Draw() {
    if (!enabled || state == WAIT_GAME) return;
    RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)FALSE);
    RwRenderStateSet(rwRENDERSTATETEXTURERASTER, nullptr);
    RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)TRUE);
    const float z = cfg.origin.z + 0.08f;
    const CVector cam = TheCamera.GetPosition();
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
    forRunners {
    const int n = static_cast<int>(course.p.size());
    for (int i = 0; i + 1 < n + (course.closed ? 1 : 0); i += 2) {
        const CVector2D a = course.p[i], b = course.p[(i + 2) % n];
        if (!course.closed && i + 2 >= n) break;
        Line3D({ a.x, a.y, z + 0.02f }, { b.x, b.y, z + 0.02f }, CRGBA(255, 220, 0, 255));
    }
    for (const auto& c : course.cones) Line3D({ c.x, c.y, z }, { c.x, c.y, z + 1.0f }, CRGBA(255, 120, 0, 255));
    for (size_t i = 1; i < trail.size(); i++) Line3D(trail[i - 1] + CVector(0, 0, -0.3f), trail[i] + CVector(0, 0, -0.3f), CRGBA(0, 220, 255, 220));
    if (state == RUNNING && car) {
        Line3D({ lookPoint.x, lookPoint.y, z }, { lookPoint.x, lookPoint.y, z + 2.5f }, CRGBA(255, 0, 0, 255));
        Line3D(car->GetPosition(), { lookPoint.x, lookPoint.y, z + 0.5f }, CRGBA(255, 0, 0, 160));
    }
    }
    R = runners;

    CFont::SetBackground(false, false);
    CFont::SetJustify(false);
    CFont::SetProportional(true);
    CFont::SetOrientation(ALIGN_LEFT);
    CFont::SetFontStyle(FONT_SUBTITLES);
    CFont::SetColor({ 255, 255, 0, 255 });
    CFont::SetEdge(1);
    CFont::SetScale(0.4f, 0.9f);
    std::string hud;
    if (state == DONE) hud = "PROVING GROUND: done - edit RealTrafficFix.proving.ini to run again";
    else if (state == RUNNING && curTest) {
        float y = 40.0f;
        forRunners {
            if (!car) continue;
            const float speed = car->m_vecMoveSpeed.Magnitude() * GAME_SPEED_TO_MPS * 3.6f;
            hud = Format("%s %s  model %d  run %d  t %.1f s  %s  |  %.0f / %.0f km/h  offset %+.2f m (max %.2f)  steer %+.3f  gas %.2f brake %.2f%s",
                R->tag, curTest->name.c_str(), curModel, runCount, runT, curTest->steer != "none" ? curTest->steer.c_str() : cfg.mode.c_str(),
                speed, targetKmh, offsetNow, stats.maxE, car->m_fSteerAngle, gasOut, brakeOut, R->finished ? "  (done)" : "");
            CFont::PrintString(10.0f, y, hud.c_str());
            y += 22.0f;
        }
        R = runners;
        return;
    }
    else hud = "PROVING GROUND: setting up";
    CFont::PrintString(10.0f, 40.0f, hud.c_str());
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
