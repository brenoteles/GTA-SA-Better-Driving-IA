#pragma once
// The proving ground's best driver (neural2, GA generation 225: 102.47 s on the 5.1 km circuit) on the traffic's lanes.
// The same driver as ProvingGround.h's Neural2Steer, with the lane's centre line (LaneLine) as its course:
//  - its teacher: pure pursuit on the line, the look-ahead LookaheadTime x v (LookaheadMin..LookaheadMax) x e^(0.5 o6),
//    x Gain, plus the yaw damping (the rotation past what the line asks for, CounterSteer)
//  - its net (80 -> 48 -> 24 -> 8, tanh, the inputs normalized as in the proving ground): o0 a residual on the wheel
//    (+-1/4 of the lock), o1 on the pedal (+-0.5, applied in FollowPathHook on a free road only), o6 its look-ahead.
//    The racing line, the obstacles, the progress and the crosswind are the proving ground's: here 0 / none / the
//    mean the net saw (in[36]), so they push nothing. Its line, TC, ABS, handbrake and aggressiveness outputs are not
//    used: the traffic keeps its lane centre, its own aids and its speed planner
// Weights: scripts\RealTrafficFix.neural.txt (the input means and spreads, 2 x 80, then the 5,264 weights: the proving
// ground's nn2_norm.txt and nn2_best.txt without its score)
namespace NeuralDriver {

static constexpr int kIn = 80, kHid = 48, kHid2 = 24, kOut = 8;
static constexpr int kL2 = (kIn + 1) * kHid, kL3 = kL2 + (kHid + 1) * kHid2, kN = kL3 + (kHid2 + 1) * kOut;
static constexpr float kSteerSpan = 0.25f, kPedalSpan = 0.5f;

static bool ready = false;
static std::vector<float> theta, mu(kIn, 0.0f), sd(kIn, 1.0f);
// the GA's best genes (ga_best.ini, generation 225)
static float lookTime = 0.5869f, lookMin = 3.524f, lookMax = 60.0f, gain = 0.8308f, counterSteer = 0.3573f;
static float residual = 1.0f, pedalResidual = 1.0f; // the share of the net's wheel and pedal (0: its teacher alone)
static int frames = 0, pedalFrames = 0;
static double residualSum = 0.0;

static void Load(const std::string& path) {
    ready = false;
    std::ifstream f(path);
    std::vector<float> v;
    float x;
    while (f >> x) v.push_back(x);
    if (static_cast<int>(v.size()) != 2 * kIn + kN) return;
    for (int q = 0; q < kIn; q++) { mu[q] = v[q]; sd[q] = std::max(v[kIn + q], 0.01f); }
    theta.assign(v.begin() + 2 * kIn, v.end());
    ready = true;
}

static float LockRad(CVehicle* v) { return v->m_pHandlingData->m_fSteeringLock * 3.14159265f / 180.0f; }

// the line's curvature at sAt (> 0: bends left), 0 past its end
static float CurvAt(const LaneLine& line, float sAt) {
    const float end = line.cum[line.n - 1], h = 6.0f;
    if (sAt + h > end || sAt - h < 0.0f) return 0.0f;
    const CVector2D p0 = LinePointAt(line, sAt - h), p1 = LinePointAt(line, sAt), p2 = LinePointAt(line, sAt + h);
    const float a = (p1 - p0).Magnitude(), b = (p2 - p1).Magnitude(), c = (p2 - p0).Magnitude();
    return a > 1.0f && b > 1.0f && c > 1.0f ? 2.0f * CVector2D::Cross(p1 - p0, p2 - p1) / (a * b * c) : 0.0f;
}
// > 0: left of the line
static float LeftOf(const LaneLine& line, const CVector2D& pt) {
    float along, right;
    ProjectOnLine(line, pt, along, right);
    return -right;
}

// what the net sees (ProvingGround.h Neural2InputsTo, in the same order), the lane as its road
static void Inputs(CVehicle* v, const ExtendedData& xdata, const LaneLine& line, float s0, float half, float teacher, float* in) {
    const CVector2D pos = v->GetPosition().To2D(), fwd = v->GetForward().To2D(), rt(fwd.y, -fwd.x);
    const float carHead = std::atan2(fwd.y, fwd.x);
    int k = 0;
    // 19 rays to the lane's edges, every 10 degrees over the front 180
    for (int r = 0; r < 19; r++) {
        const float a = carHead + (-90.0f + 10.0f * r) * 3.14159265f / 180.0f;
        const CVector2D dir(std::cos(a), std::sin(a));
        float dist = 60.0f;
        for (float t = 4.0f; t <= 60.0f; t += 4.0f)
            if (std::abs(LeftOf(line, pos + dir * t)) > half) {
                dist = std::abs(LeftOf(line, pos + dir * (t - 2.0f))) > half ? t - 2.0f : t;
                break;
            }
        in[k++] = dist / 60.0f;
    }
    // the road and its own line (the same here): offset, heading error; the room each side (the body's corners)
    const CVector2D tan = LinePointAt(line, s0 + 3.0f) - LinePointAt(line, s0 - 3.0f);
    const float lineHead = std::atan2(tan.y, tan.x);
    const float off = LeftOf(line, pos), head = WrapAngle(carHead - lineHead);
    in[k++] = off / half;
    in[k++] = off / half;
    in[k++] = head;
    in[k++] = head;
    CVector mn, mx;
    GetEntityDimensions(v, mn, mx);
    const float halfW = std::max(-mn.x, mx.x);
    float latMax = -99.0f, latMin = 99.0f;
    for (float fy : { mx.y, mn.y })
        for (float sx : { -1.0f, 1.0f }) {
            const float lat = LeftOf(line, pos + fwd * fy + rt * (sx * halfW));
            latMax = std::max(latMax, lat);
            latMin = std::min(latMin, lat);
        }
    in[k++] = (half - latMax) / 2.0f;
    in[k++] = (half + latMin) / 2.0f;
    // the curvature ahead, the next curve, no obstacle, the progress the net saw on average
    for (float m : { 0.0f, 10.0f, 25.0f, 50.0f, 100.0f, 200.0f }) in[k++] = std::clamp(CurvAt(line, s0 + m) * 20.0f, -1.0f, 1.0f);
    float curveD = 1.0f, curveK = 0.0f;
    for (float m = 0.0f; m <= 300.0f; m += 6.0f) {
        const float c = CurvAt(line, s0 + m);
        if (std::abs(c) > 1.0f / 150.0f) { curveD = m / 300.0f; curveK = std::clamp(c * 20.0f, -1.0f, 1.0f); break; }
    }
    in[k++] = curveD;
    in[k++] = curveK;
    in[k++] = 1.0f;
    in[k++] = 0.0f;
    in[k++] = 0.0f;
    in[k] = mu[k]; // (the progress: the mean the net saw)
    k++;
    // the car's motion
    const CVector vel = v->m_vecMoveSpeed * GAME_SPEED_TO_MPS, turn = v->m_vecTurnSpeed * GAME_SPEED_TO_MPS;
    const float vF = CVector::Dot(vel, v->GetForward()), vR = CVector::Dot(vel, v->GetRight());
    CVector bmn, bmx;
    GetEntityDimensions(v, bmn, bmx);
    const float rearY = -0.5f * GetWheelBase(v, bmn, bmx);
    const float latG = vF * turn.z / GRAVITY;
    in[k++] = vF / 80.0f;
    in[k++] = vR / 10.0f;
    in[k++] = vel.z / 10.0f;
    in[k++] = turn.z / 2.0f;
    in[k++] = CVector::Dot(turn, v->GetForward()) / 2.0f;
    in[k++] = CVector::Dot(turn, v->GetRight()) / 2.0f;
    in[k++] = std::clamp(xdata.measAccel / 20.0f, -2.0f, 2.0f);
    in[k++] = std::clamp(latG / 2.0f, -2.0f, 2.0f);
    in[k++] = std::atan2(vR, std::max(std::abs(vF), 0.5f));
    in[k++] = std::atan2(vR - turn.z * rearY, std::max(std::abs(vF), 0.5f));
    in[k++] = v->GetRight().z;
    in[k++] = v->GetForward().z;
    // its wheel last frame, its teacher's now, the planner's speed against its own
    const float lock = LockRad(v);
    in[k++] = v->m_fSteerAngle / lock;
    in[k++] = teacher / lock;
    in[k++] = std::clamp((xdata.targetSpeed - vF) / 50.0f, -2.0f, 2.0f);
    // each wheel: its state, its suspension, on the ground or not
    CAutomobile* a = static_cast<CAutomobile*>(v);
    int contact = 0;
    for (int w = 0; w < 4; w++) in[k++] = static_cast<float>(a->m_wheelState[w]) / 3.0f;
    for (int w = 0; w < 4; w++) { const float c = a->m_fWheelsSuspensionCompressionPrev[w]; in[k++] = c; contact += c < 1.0f; }
    in[k++] = contact / 4.0f;
    in[k++] = contact == 0 ? 1.0f : 0.0f;
    in[k++] = std::clamp(std::abs(latG) / std::max(AiGripG(v), 0.1f), 0.0f, 2.0f);
    const float vFGame = CVector::Dot(v->m_vecMoveSpeed, v->GetForward());
    for (int w = 0; w < 4; w++)
        in[k++] = std::clamp((a->m_fWheelSpeed[w] * 0.5f * v->m_fWheelScale - vFGame) * GAME_SPEED_TO_MPS / 20.0f, -1.0f, 1.0f);
    for (int w = 0; w < 4; w++)
        in[k++] = std::clamp((a->m_fWheelsSuspensionCompressionPrev[w] - a->m_fWheelsSuspensionCompression[w]) * 5.0f / std::max(CTimer::ms_fTimeStep, 0.2f), -1.0f, 1.0f);
    const cTransmission& tr = v->m_pHandlingData->m_transmissionData;
    const int gear = std::clamp<int>(v->m_nCurrentGear, 0, std::clamp<int>(tr.m_nNumberOfGears, 1, 5));
    in[k++] = static_cast<float>(gear) / std::max<int>(tr.m_nNumberOfGears, 1);
    in[k++] = std::clamp(vFGame / std::max(std::abs(tr.m_aGears[gear].m_fMaxVelocity), 0.05f), -1.0f, 1.0f);
    in[k++] = a->m_nNumDriveWheelsOnGroundLastFrame / 4.0f;
    in[k++] = std::clamp(CVector::Dot(v->m_vecFrictionMoveSpeed, v->GetRight()) * GAME_SPEED_TO_MPS / 2.0f, -1.0f, 1.0f);
    in[k++] = std::clamp(v->m_vecFrictionTurnSpeed.z * GAME_SPEED_TO_MPS / 0.5f, -1.0f, 1.0f);
    // the racing line 20, 50, 100 m on and the crosswind: none in traffic
    while (k < kIn) in[k++] = 0.0f;
}

static void Forward(const float* in, float* o) {
    float xn[kIn], h[kHid], h2[kHid2];
    for (int q = 0; q < kIn; q++) xn[q] = std::clamp((in[q] - mu[q]) / sd[q], -5.0f, 5.0f);
    const float* w = theta.data();
    for (int j = 0; j < kHid; j++) {
        const float* row = w + j * (kIn + 1);
        float s = row[kIn];
        for (int q = 0; q < kIn; q++) s += row[q] * xn[q];
        h[j] = std::tanh(s);
    }
    for (int j = 0; j < kHid2; j++) {
        const float* row = w + kL2 + j * (kHid + 1);
        float s = row[kHid];
        for (int q = 0; q < kHid; q++) s += row[q] * h[q];
        h2[j] = std::tanh(s);
    }
    for (int m = 0; m < kOut; m++) {
        const float* row = w + kL3 + m * (kHid2 + 1);
        float s = row[kHid2];
        for (int q = 0; q < kHid2; q++) s += row[q] * h2[q];
        o[m] = std::tanh(s);
    }
}

// The wheel (> 0: left) on the line from s0 (the car's place along it); refOffset: where across it should be (> 0:
// right, a lane change's S) t s on. NAN: not a car this driver can drive
static float Steer(CVehicle* veh, ExtendedData& xdata, const LaneLine& line, float s0, float speed,
    const std::function<float(float)>& refOffset, float maxSteer) {
    if (!ready || veh->m_nVehicleSubClass != VEHICLE_AUTOMOBILE) return NAN;
    const float lock = LockRad(veh);
    const float vF = std::max(CVector::Dot(veh->m_vecMoveSpeed, veh->GetForward()) * GAME_SPEED_TO_MPS, 0.0f);
    const float vv = std::max(speed, 3.0f);
    CVector mn, mx;
    GetEntityDimensions(veh, mn, mx);
    const float L = GetWheelBase(veh, mn, mx);
    // the teacher: pursuit (its look-ahead the net's from last frame)
    const float ld = std::clamp(lookTime * vv, lookMin, lookMax) * std::exp(0.5f * xdata.nnLook);
    const float end = line.cum[line.n - 1];
    const float sAim = std::min(s0 + ld, end);
    const CVector2D t = LinePointAt(line, std::min(sAim + 1.0f, end)) - LinePointAt(line, std::max(sAim - 1.0f, 0.0f));
    const CVector2D tn = t * (1.0f / std::max(t.Magnitude(), 0.001f));
    const CVector2D aim = LinePointAt(line, sAim) + CVector2D(tn.y, -tn.x) * (refOffset ? refOffset(ld / vv) : 0.0f);
    const CVector2D pos = veh->GetPosition().To2D(), fwd = veh->GetForward().To2D(), left(-fwd.y, fwd.x);
    const CVector2D rel = aim - pos;
    float teacher = std::atan2(2.0f * L * std::sin(std::atan2(CVector2D::Dot(rel, left), CVector2D::Dot(rel, fwd))), ld) * gain;
    // its yaw damping: the rotation past what the line asks for at this speed, taken off as the wheel that makes it
    {
        const float yaw = veh->m_vecTurnSpeed.z * GAME_SPEED_TO_MPS;
        float yawEx = yaw - vF * CurvAt(line, s0);
        yawEx = yawEx > 0.0f ? std::max(yawEx - 0.05f, 0.0f) : std::min(yawEx + 0.05f, 0.0f);
        teacher += counterSteer * (-yawEx * L / std::max(vF, 5.0f));
    }
    teacher = std::clamp(teacher, -lock, lock);
    // the net
    float in[kIn], o[kOut];
    Inputs(veh, xdata, line, s0, 0.5f * LANE_WIDTH, teacher, in);
    Forward(in, o);
    xdata.nnLook = o[6];
    xdata.nnPedal = o[1];
    xdata.nnFrame = frameCounter;
    frames++;
    residualSum += std::abs(o[0]);
    return std::clamp(teacher + residual * kSteerSpan * lock * o[0], -maxSteer, maxSteer);
}

// Its pedal over the traffic's, on a free road only (nothing ahead to keep a gap to, no light, no junction): more brake
// always, more gas only while not braking
static void Pedal(CVehicle* veh, ExtendedData& xdata, float speed, float* gas, float* brake) {
    if (!ready || pedalResidual <= 0.0f || xdata.nnFrame + 1 < frameCounter) return;
    const bool freeRoad = (xdata.limiter == LIMIT_DESIRED || xdata.limiter == LIMIT_CURVE || xdata.limiter == LIMIT_HEADING ||
        xdata.limiter == LIMIT_ROAD) && (xdata.obstacle == OBSTACLE_NONE || xdata.gap > std::max(40.0f, 3.0f * speed));
    if (!freeRoad || speed < 3.0f) return;
    const float d = pedalResidual * kPedalSpan * std::clamp(xdata.nnPedal, -1.0f, 1.0f);
    if (d > 0.0f && *brake > 0.0f) return;
    const float pedal = std::clamp(*gas - *brake + d, -1.0f, 1.0f);
    *gas = std::max(pedal, 0.0f);
    *brake = std::max(-pedal, 0.0f);
    pedalFrames++;
}

} // namespace NeuralDriver
