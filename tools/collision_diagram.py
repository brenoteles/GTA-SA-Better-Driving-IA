"""Draws the collision diagrams of an AutoTest run: collision_NNN.trk -> collision_NNN.png (seen from above).

usage: python collision_diagram.py <autotest folder or .trk files>

Vehicle A's final heading points up. Blue = A, red = B, grey = vehicles near them. Boxes every second
(-4 s ... 0), trail dots yellow while moving over (lane change), amber triangles = indicator on. Below: speed and
target speed of A and B over the last 5 s.
"""
import math
import os
import sys
import textwrap

from PIL import Image, ImageDraw, ImageFont

LIMITERS = ["-", "desired", "leader", "curve", "junction", "bump", "bicycle", "heading", "light", "crest",
            "cross traffic", "damage", "road", "car top", "bypass", "yield", "overtaking"]
LANE_STATES = ["keep", "signal overtake", "overtaking", "signal move", "CHANGING"]
LANE_WIDTH = 5.4
W, MAP_H, PANEL_H = 1000, 1000, 330
COLORS = {"A": (40, 120, 255), "B": (235, 50, 50), "O": (150, 150, 150)}


def font(size):
    for name in ("consola.ttf", "arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def parse(path):
    d = {"header": {}, "roads": [], "lanes": [], "vehs": {}, "impact": (0.0, 0.0)}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            key, _, rest = line.partition(" ")
            p = rest.split()
            if key in ("TITLE", "CAUSE", "WHO_A", "WHO_B"):
                d["header"][key] = rest
            elif key == "IMPACT":
                d["impact"] = (float(p[0]), float(p[1]))
            elif key == "ROAD":
                d["roads"].append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4]), int(p[5])))
            elif key == "LANE":
                d["lanes"].append((float(p[0]), float(p[1]), float(p[2]), float(p[3]), int(p[4])))
            elif key == "VEH":
                d["vehs"][p[0]] = {"role": p[1], "mnx": float(p[2]), "mxx": float(p[3]), "mny": float(p[4]),
                                   "mxy": float(p[5]), "model": p[6], "label": p[7].replace("_", " "), "s": []}
            elif key == "S" and p[0] in d["vehs"]:
                d["vehs"][p[0]]["s"].append({
                    "t": int(p[1]), "x": float(p[2]), "y": float(p[3]), "hx": float(p[4]), "hy": float(p[5]),
                    "fwd": float(p[6]), "target": float(p[7]), "lim": int(p[8]), "lane": int(p[9]), "ind": int(p[10]),
                    "cur": int(p[11]), "next": int(p[12]), "obst": p[13], "gap": float(p[14])})
    return d


def render(path):
    d = parse(path)
    vehs = d["vehs"]
    a = next((v for v in vehs.values() if v["role"] == "A"), None)
    b = next((v for v in vehs.values() if v["role"] == "B"), None)
    if not a or not a["s"]:
        return None
    ix, iy = d["impact"]
    # Rotation: A's last heading up
    last = a["s"][-1]
    ang = math.atan2(last["hy"], last["hx"]) - math.pi / 2
    ca, sa = math.cos(-ang), math.sin(-ang)

    def rot(x, y):
        x, y = x - ix, y - iy
        return x * ca - y * sa, x * sa + y * ca

    # Scale: fit the tracks of A and B
    pts = [rot(s["x"], s["y"]) for v in (a, b) if v for s in v["s"] if s["t"] > -2500]
    ext = max([abs(c) for p in pts for c in p] + [12.0]) + 8.0
    ext = min(ext, 60.0)
    scale = (MAP_H / 2) / ext

    def px(x, y):
        rx, ry = rot(x, y)
        return W / 2 + rx * scale, MAP_H / 2 - ry * scale

    img = Image.new("RGB", (W, MAP_H + PANEL_H), (34, 92, 40))
    over = Image.new("RGBA", img.size, (0, 0, 0, 0))
    dr = ImageDraw.Draw(over)

    # Asphalt: every lane a strip LANE_WIDTH wide
    for x1, y1, x2, y2, _ in d["lanes"]:
        dr.line([px(x1, y1), px(x2, y2)], fill=(70, 70, 74, 255), width=max(int(LANE_WIDTH * scale), 1))
    # Lane centres (thin dashes) with direction ticks
    for x1, y1, x2, y2, lane in d["lanes"]:
        p1, p2 = px(x1, y1), px(x2, y2)
        L = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        if L < 1:
            continue
        ux, uy = (p2[0] - p1[0]) / L, (p2[1] - p1[1]) / L
        t = 0.0
        while t < L:
            e = min(t + 8, L)
            dr.line([(p1[0] + ux * t, p1[1] + uy * t), (p1[0] + ux * e, p1[1] + uy * e)], fill=(200, 200, 200, 110), width=1)
            t += 16
        mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
        dr.polygon([(mx + ux * 7, my + uy * 7), (mx - ux * 4 + uy * 4, my - uy * 4 - ux * 4),
                    (mx - ux * 4 - uy * 4, my - uy * 4 + ux * 4)], fill=(220, 220, 220, 150))

    def box(v, s, color, alpha, outline_only=False):
        hx, hy = s["hx"], s["hy"]
        n = math.hypot(hx, hy) or 1
        hx, hy = hx / n, hy / n
        rx, ry = hy, -hx  # right
        corners = []
        for lx, ly in ((v["mnx"], v["mny"]), (v["mxx"], v["mny"]), (v["mxx"], v["mxy"]), (v["mnx"], v["mxy"])):
            corners.append(px(s["x"] + rx * lx + hx * ly, s["y"] + ry * lx + hy * ly))
        if outline_only:
            dr.polygon(corners, outline=color + (alpha,))
        else:
            dr.polygon(corners, fill=color + (alpha,), outline=(255, 255, 255, min(alpha + 60, 255)))
        # Front marker
        fx, fy = px(s["x"] + hx * v["mxy"], s["y"] + hy * v["mxy"])
        dr.ellipse([fx - 3, fy - 3, fx + 3, fy + 3], fill=(255, 255, 255, alpha))
        # Indicator: amber triangle on that side
        if s["ind"]:
            side = 1 if s["ind"] > 0 else -1
            lx = (v["mxx"] if side > 0 else v["mnx"]) + side * 0.8
            cx, cy = px(s["x"] + rx * lx, s["y"] + ry * lx)
            dr.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(255, 170, 0, alpha))
        return corners

    f_small, f_med, f_big = font(15), font(17), font(20)
    order = sorted(vehs.values(), key=lambda v: {"O": 0, "B": 1, "A": 2}[v["role"]])
    for v in order:
        if not v["s"]:
            continue
        color = COLORS[v["role"]]
        trail = [px(s["x"], s["y"]) for s in v["s"]]
        if len(trail) > 1:
            dr.line(trail, fill=color + (200,), width=2)
        for s, p in zip(v["s"], trail):
            if s["lane"] == 4:
                dr.ellipse([p[0] - 3, p[1] - 3, p[0] + 3, p[1] + 3], fill=(255, 230, 0, 230))
        # Boxes every second
        for sec in (-4000, -3000, -2000, -1000):
            s = min(v["s"], key=lambda q: abs(q["t"] - sec))
            if abs(s["t"] - sec) < 150:
                c = box(v, s, color, 55 if v["role"] != "O" else 30)
                if v["role"] != "O":
                    cx = sum(p[0] for p in c) / 4
                    cy = sum(p[1] for p in c) / 4
                    dr.text((cx + 8, cy - 8), f"{sec // 1000}s {s['fwd']:.0f}", font=f_small, fill=(255, 255, 255, 170))
        s = v["s"][-1]
        c = box(v, s, color, 235 if v["role"] != "O" else 140)
        cx = max(p[0] for p in c) + 8
        cy = sum(p[1] for p in c) / 4
        if v["role"] == "O":
            txt = f"{v['label']} {s['fwd']:.0f}"
        else:
            txt = (f"{v['role']} {v['label']} {s['fwd']:.0f}>{s['target']:.0f} {LIMITERS[s['lim']] if s['lim'] < len(LIMITERS) else s['lim']}\n"
                   f"lane {s['cur']}>{s['next']} {LANE_STATES[s['lane']] if s['lane'] < 5 else s['lane']}"
                   f"{'  ind ' + ('>' if s['ind'] > 0 else '<') if s['ind'] else ''}  {s['obst']} {s['gap']:.1f}m")
        dr.multiline_text((cx, cy - 10), txt, font=f_small if v["role"] == "O" else f_med,
                          fill=(255, 255, 255, 255), stroke_width=2, stroke_fill=(0, 0, 0, 255))

    # Impact
    ipx, ipy = W / 2, MAP_H / 2
    dr.line([(ipx - 10, ipy - 10), (ipx + 10, ipy + 10)], fill=(255, 255, 0, 255), width=3)
    dr.line([(ipx - 10, ipy + 10), (ipx + 10, ipy - 10)], fill=(255, 255, 0, 255), width=3)
    # Scale bar
    bar = 10 * scale
    dr.line([(20, MAP_H - 25), (20 + bar, MAP_H - 25)], fill=(255, 255, 255, 255), width=3)
    dr.text((25 + bar, MAP_H - 35), "10 m", font=f_small, fill=(255, 255, 255, 255))

    img = Image.alpha_composite(img.convert("RGBA"), over).convert("RGB")
    dr = ImageDraw.Draw(img)

    # Panel: text and speed graph
    dr.rectangle([0, MAP_H, W, MAP_H + PANEL_H], fill=(20, 20, 24))
    h = d["header"]
    y = MAP_H + 8
    dr.text((10, y), h.get("TITLE", ""), font=f_big, fill=(255, 255, 255)); y += 26
    dr.text((10, y), "CAUSE: " + h.get("CAUSE", ""), font=f_big, fill=(255, 220, 80)); y += 28
    for key, role in (("WHO_A", "A"), ("WHO_B", "B")):
        for i, line in enumerate(textwrap.wrap(h.get(key, ""), 100)):
            dr.text((10, y), (role + ": " if i == 0 else "   ") + line, font=f_small, fill=COLORS[role]); y += 18
    # Speed graph (km/h over the last 5 s)
    gx0, gy0, gx1, gy1 = 60, y + 10, W - 20, MAP_H + PANEL_H - 25
    if gy1 - gy0 > 60:
        vmax = max([s["fwd"] for v in (a, b) if v for s in v["s"]] + [s["target"] for v in (a, b) if v for s in v["s"]] + [20])
        vmax = min(math.ceil(vmax / 20) * 20, 300)
        dr.rectangle([gx0, gy0, gx1, gy1], outline=(90, 90, 90))
        for k in range(0, int(vmax) + 1, 20):
            yy = gy1 - (gy1 - gy0) * k / vmax
            dr.line([(gx0, yy), (gx1, yy)], fill=(45, 45, 50))
            dr.text((10, yy - 8), f"{k}", font=f_small, fill=(160, 160, 160))
        for sec in range(-5, 1):
            xx = gx0 + (gx1 - gx0) * (sec + 5) / 5
            dr.text((xx - 10, gy1 + 4), f"{sec}s", font=f_small, fill=(160, 160, 160))

        def gp(s, key):
            return gx0 + (gx1 - gx0) * (s["t"] + 5000) / 5000, gy1 - (gy1 - gy0) * max(min(s[key], vmax), 0) / vmax
        for v in (a, b):
            if not v or len(v["s"]) < 2:
                continue
            col = COLORS[v["role"]]
            dim = tuple(c // 2 + 40 for c in col)
            dr.line([gp(s, "target") for s in v["s"]], fill=dim, width=1)
            dr.line([gp(s, "fwd") for s in v["s"]], fill=col, width=3)
        dr.text((gx1 - 330, gy0 + 4), "thick: speed  thin: target (km/h)", font=f_small, fill=(180, 180, 180))

    out = os.path.splitext(path)[0] + ".png"
    img.save(out)
    return out


def main():
    paths = []
    for arg in sys.argv[1:]:
        if os.path.isdir(arg):
            paths += [os.path.join(arg, f) for f in sorted(os.listdir(arg)) if f.endswith(".trk")]
        else:
            paths.append(arg)
    for p in paths:
        try:
            print(render(p))
        except Exception as e:  # one broken file doesn't stop the rest
            print(f"{p}: {e}")


if __name__ == "__main__":
    main()
