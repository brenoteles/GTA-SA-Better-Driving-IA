#pragma once
// The test town (tools/make_town.py): its roads are path nodes the game drives like any street, on the proving grid's top
// deck; this draws them there (asphalt, edges, the middle line of two-way roads, lane dividers, stop lines coloured by
// their junction's signal). Read from scripts\RealTrafficFix.town.txt; nothing drawn without it or away from the town
namespace TownDraw {

struct Seg { CVector2D a, b; CRGBA c; };
struct QuadT { CVector2D p[4]; CVector2D mid; };
struct Stop { CVector2D a, b; CNodeAddress junction; int cycle; };
static std::vector<QuadT> quads;
static std::vector<Seg> lines;
static std::vector<Stop> stops;
static float z = 0.0f;
static bool loaded = false, have = false;
static constexpr float kX0 = -150.0f, kX1 = 2950.0f, kY0 = -3000.0f, kY1 = -1700.0f;

static void Load(const std::string& path) {
    loaded = true;
    std::ifstream f(path);
    std::string kind;
    while (f >> kind) {
        if (kind == "Z") f >> z;
        else if (kind == "Q") {
            QuadT q;
            for (auto& p : q.p) f >> p.x >> p.y;
            q.mid = (q.p[0] + q.p[2]) * 0.5f;
            quads.push_back(q);
        }
        else if (kind == "L") {
            Seg s;
            int r, g, b;
            f >> s.a.x >> s.a.y >> s.b.x >> s.b.y >> r >> g >> b;
            s.c = CRGBA(r, g, b, 230);
            lines.push_back(s);
        }
        else if (kind == "D") {
            // a lane divider: 3 m dashes every 9 m
            CVector2D a, b;
            f >> a.x >> a.y >> b.x >> b.y;
            const float L = (b - a).Magnitude();
            const CVector2D u = (b - a) * (1.0f / std::max(L, 0.01f));
            for (float s = 0.0f; s < L; s += 9.0f) lines.push_back({ a + u * s, a + u * std::min(s + 3.0f, L), CRGBA(255, 255, 255, 200) });
        }
        else if (kind == "S") {
            Stop s;
            int area, node;
            f >> s.a.x >> s.a.y >> s.b.x >> s.b.y >> area >> node >> s.cycle;
            s.junction = CNodeAddress(static_cast<short>(area), static_cast<short>(node));
            stops.push_back(s);
        }
        else std::getline(f, kind);
    }
    have = !quads.empty();
}

static std::vector<RwIm3DVertex> buf;
static void Flush(RwPrimitiveType type) {
    if (buf.empty()) return;
    if (RwIm3DTransform(buf.data(), static_cast<RwUInt32>(buf.size()), nullptr, 0)) {
        RwIm3DRenderPrimitive(type);
        RwIm3DEnd();
    }
    buf.clear();
}
static void Vertex(float x, float y, float zz, CRGBA c) {
    RwIm3DVertex v;
    RwIm3DVertexSetPos(&v, x, y, zz);
    RwIm3DVertexSetRGBA(&v, c.r, c.g, c.b, c.a);
    buf.push_back(v);
}

static void Draw(const std::string& path) {
    if (!loaded) Load(path);
    if (!have) return;
    const CVector cam = TheCamera.GetPosition();
    if (cam.x < kX0 || cam.x > kX1 || cam.y < kY0 || cam.y > kY1 || std::abs(cam.z - z) > 400.0f) return;
    const CVector2D c2(cam.x, cam.y);
    const float near2 = 450.0f * 450.0f;
    RwRenderStateSet(rwRENDERSTATETEXTURERASTER, nullptr);
    RwRenderStateSet(rwRENDERSTATEVERTEXALPHAENABLE, (void*)TRUE);
    RwRenderStateSet(rwRENDERSTATEZTESTENABLE, (void*)TRUE);
    RwRenderStateSet(rwRENDERSTATEZWRITEENABLE, (void*)FALSE);
    RwRenderStateSet(rwRENDERSTATECULLMODE, (void*)rwCULLMODECULLNONE);
    const CRGBA asphalt(52, 55, 60, 255);
    for (const QuadT& q : quads) {
        if ((q.mid - c2).MagnitudeSqr() > near2) continue;
        for (int i : { 0, 1, 2, 0, 2, 3 }) Vertex(q.p[i].x, q.p[i].y, z + 0.04f, asphalt);
        if (buf.size() >= 3000) Flush(rwPRIMTYPETRILIST);
    }
    // stop lines: a 0.6 m bar in the light's colour
    for (const Stop& s : stops) {
        if ((s.a - c2).MagnitudeSqr() > near2) continue;
        const eLightColor col = cfgActuatedLights ? SignalColor(s.junction, s.cycle)
                                                  : static_cast<eLightColor>(GlobalLightForCars(s.cycle));
        const CRGBA c = col == LIGHT_GREEN ? CRGBA(40, 220, 60, 255) : col == LIGHT_YELLOW ? CRGBA(255, 200, 0, 255) : CRGBA(230, 30, 30, 255);
        CVector2D d = s.b - s.a;
        const CVector2D u = CVector2D(d.y, -d.x) * (0.3f / std::max(d.Magnitude(), 0.01f));
        const CVector2D p[4] = { s.a - u, s.b - u, s.b + u, s.a + u };
        for (int i : { 0, 1, 2, 0, 2, 3 }) Vertex(p[i].x, p[i].y, z + 0.07f, c);
    }
    Flush(rwPRIMTYPETRILIST);
    for (const Seg& s : lines) {
        if ((s.a - c2).MagnitudeSqr() > near2) continue;
        Vertex(s.a.x, s.a.y, z + 0.08f, s.c);
        Vertex(s.b.x, s.b.y, z + 0.08f, s.c);
        if (buf.size() >= 4000) Flush(rwPRIMTYPELINELIST);
    }
    // the signal heads: a post 4 m up at the right end of each stop line, its lamp the light's colour
    for (const Stop& s : stops) {
        if ((s.a - c2).MagnitudeSqr() > near2) continue;
        const eLightColor col = cfgActuatedLights ? SignalColor(s.junction, s.cycle)
                                                  : static_cast<eLightColor>(GlobalLightForCars(s.cycle));
        const CRGBA c = col == LIGHT_GREEN ? CRGBA(40, 255, 60, 255) : col == LIGHT_YELLOW ? CRGBA(255, 200, 0, 255) : CRGBA(255, 30, 30, 255);
        Vertex(s.b.x, s.b.y, z, CRGBA(30, 30, 30, 255));
        Vertex(s.b.x, s.b.y, z + 4.0f, CRGBA(30, 30, 30, 255));
        for (float h = 4.0f; h <= 5.0f; h += 0.1f) {
            Vertex(s.b.x - 0.3f, s.b.y, z + h, c);
            Vertex(s.b.x + 0.3f, s.b.y, z + h, c);
        }
    }
    Flush(rwPRIMTYPELINELIST);
}

} // namespace TownDraw
