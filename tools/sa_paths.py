"""GTA San Andreas path data (data/paths/NODES0..63.DAT): reader, writer and analysis of the road network.

File layout (one per 750 x 750 m area, 8 x 8 areas from (-3000, -3000)):
  header       5 x uint32: nodes, vehicle nodes, ped nodes, navi nodes, links
  path nodes   28 bytes each (vehicle nodes first, then ped nodes)
  navi nodes   14 bytes each (the "links" cars drive along: position, direction, lanes, traffic light)
  links        4 bytes each (area, node) - the neighbours of each node, from its first link index
  filler       768 bytes
  navi links   2 bytes each per link (navi node id | area << 10)
  link lengths 1 byte each per link
  intersection flags 1 byte each per link (only in some files)
"""
import math
import os
import struct
import sys
from collections import Counter, defaultdict


class Node:
    __slots__ = ('area', 'id', 'x', 'y', 'z', 'heur', 'link', 'width', 'flood', 'flags', 'raw')

    @property
    def links(self):
        return self.flags & 0xF

    @property
    def traffic(self):          # spawn density 0..3
        return (self.flags >> 4) & 3

    @property
    def road_block(self):
        return bool(self.flags & (1 << 6))

    @property
    def boats(self):
        return bool(self.flags & (1 << 7))

    @property
    def emergency(self):
        return bool(self.flags & (1 << 8))

    @property
    def not_highway(self):
        return bool(self.flags & (1 << 12))

    @property
    def highway(self):
        return bool(self.flags & (1 << 13))

    @property
    def parking(self):
        return bool(self.flags & (1 << 21))

    @property
    def speed(self):            # 0 slow, 1 normal, 2 freeway (the game's speed class)
        return (self.flags >> 16) & 0x3


class Navi:
    __slots__ = ('area', 'id', 'x', 'y', 'tarea', 'tnode', 'dx', 'dy', 'flags', 'area_index')

    @property
    def width(self):
        return self.flags & 0xFF

    @property
    def left(self):
        return (self.flags >> 8) & 7

    @property
    def right(self):
        return (self.flags >> 11) & 7

    @property
    def light_dir(self):
        return (self.flags >> 14) & 1

    @property
    def light(self):            # 0 none, 1 north-south cycle, 2 west-east cycle
        return (self.flags >> 16) & 3

    @property
    def train(self):
        return (self.flags >> 18) & 1


class Area:
    def __init__(self, path):
        self.path = path
        d = open(path, 'rb').read()
        self.data = d
        self.n_nodes, self.n_veh, self.n_ped, self.n_navi, self.n_links = struct.unpack_from('<5I', d, 0)
        o = 20
        self.nodes = []
        for i in range(self.n_nodes):
            n = Node()
            (_, _, x, y, z, n.heur, n.link, n.area, n.id, n.width, n.flood, n.flags) = struct.unpack_from('<IIhhhhHHHBBI', d, o)
            n.x, n.y, n.z = x / 8.0, y / 8.0, z / 8.0
            n.raw = o
            self.nodes.append(n)
            o += 28
        self.navi = []
        for i in range(self.n_navi):
            v = Navi()
            x, y, v.tarea, v.tnode, v.dx, v.dy, v.flags = struct.unpack_from('<hhHHbbI', d, o)
            v.x, v.y = x / 8.0, y / 8.0
            v.id = i
            self.navi.append(v)
            o += 14
        self.navi_off = 20 + 28 * self.n_nodes
        self.links = []
        for i in range(self.n_links):
            self.links.append(struct.unpack_from('<HH', d, o))
            o += 4
        o += 768
        self.navi_links = []
        for i in range(self.n_links):
            self.navi_links.append(struct.unpack_from('<H', d, o)[0])
            o += 2
        self.lengths = list(d[o:o + self.n_links])
        o += self.n_links
        self.inter = list(d[o:o + self.n_links]) if len(d) >= o + self.n_links else None

    def write(self, path):
        """Writes the file back with the (possibly changed) navi node flags"""
        b = bytearray(self.data)
        for v in self.navi:
            struct.pack_into('<I', b, self.navi_off + 14 * v.id + 10, v.flags)
        open(path, 'wb').write(bytes(b))


def load_all(folder):
    areas = {}
    for i in range(64):
        p = os.path.join(folder, 'NODES%d.DAT' % i)
        if os.path.exists(p):
            areas[i] = Area(p)
            for v in areas[i].navi:
                v.area_index = i
    return areas


class Network:
    """Vehicle road graph: nodes by (area, id), edges with the navi node (lanes, light) between them"""

    def __init__(self, areas):
        self.areas = areas
        self.node = {}
        for a, ar in areas.items():
            for n in ar.nodes[:ar.n_veh]:
                self.node[(a, n.id)] = n
        self.adj = defaultdict(list)    # (a, id) -> [((a2, id2), navi)]
        for a, ar in areas.items():
            for n in ar.nodes[:ar.n_veh]:
                for k in range(n.links):
                    li = n.link + k
                    ta, tn = ar.links[li]
                    nl = ar.navi_links[li]
                    navi_area, navi_id = nl >> 10, nl & 0x3FF
                    nv = areas[navi_area].navi[navi_id] if navi_area in areas and navi_id < len(areas[navi_area].navi) else None
                    self.adj[(a, n.id)].append(((ta, tn), nv))

    def lanes_toward(self, a_key, b_key, nv):
        """Lanes for a car driving from node a to node b on navi nv (same rule as the game: the navi's direction)"""
        # The game (LanesToward): the navi's own node is the one it points to; towards it = 'left' lanes field
        return nv.left if (nv.tarea, nv.tnode) == b_key else nv.right


def angle(dx, dy):
    return math.atan2(dy, dx)


def analyze(net, out):
    node = net.node
    junctions = [k for k, n in node.items() if len(net.adj[k]) > 2]
    out.write('vehicle nodes %d, junctions (3+ roads) %d\n' % (len(node), len(junctions)))
    # Speed classes, lanes
    sp = Counter(n.speed for n in node.values())
    out.write('speed classes: %s\n' % dict(sp))
    lanes = Counter()
    for k, lst in net.adj.items():
        for nb, nv in lst:
            if nv and nb in node:
                lanes[(nv.left, nv.right)] += 1
    out.write('links by lanes (one way, other way): %s\n' % dict(lanes.most_common(12)))

    # Traffic lights per junction: the navi links into it with a light and their cycle (1 N-S, 2 W-E)
    lit = 0
    issues = Counter()
    examples = defaultdict(list)
    for k in junctions:
        n = node[k]
        appr = []
        for nb, nv in net.adj[k]:
            if not nv or nb not in node:
                continue
            m = node[nb]
            ang = math.degrees(angle(m.x - n.x, m.y - n.y)) % 360
            appr.append((ang, nv.light, nb))
        lights = [a for a in appr if a[1]]
        if not lights:
            continue
        lit += 1
        # Axis of each lit approach: N-S (60..120 or 240..300 deg) or W-E
        for ang, l, nb in lights:
            ns = 45 <= ang <= 135 or 225 <= ang <= 315
            expected = 1 if ns else 2
            if l != expected:
                issues['light cycle does not match the road axis'] += 1
                examples['light cycle does not match the road axis'].append((k, round(n.x), round(n.y), round(ang), l))
        if len(lights) < len(appr):
            issues['junction with lights on some roads only'] += 1
            examples['junction with lights on some roads only'].append((k, round(n.x), round(n.y), len(lights), len(appr)))
        cycles = set(l for _, l, _ in lights)
        if len(cycles) == 1 and len(lights) > 1:
            issues['all lit roads on the same cycle (all green together)'] += 1
            examples['all lit roads on the same cycle (all green together)'].append((k, round(n.x), round(n.y), len(lights)))
    out.write('junctions with traffic lights: %d\n' % lit)
    for name, c in issues.most_common():
        out.write('  %-55s %d   e.g. %s\n' % (name, c, examples[name][:4]))

    # Lane count changes along a road (2-link nodes): lanes that end/appear without a junction
    drops = 0
    for k, lst in net.adj.items():
        if len(lst) != 2 or k not in node:
            continue
        (b1, v1), (b2, v2) = lst
        if not v1 or not v2 or b1 not in node or b2 not in node:
            continue
        l_in = net.lanes_toward(b1, k, v1)
        l_out = net.lanes_toward(k, b2, v2)
        if l_in and l_out and l_in != l_out:
            drops += 1
    out.write('lane count changes in the middle of a road (one direction, 2-link nodes): %d\n' % drops)


if __name__ == '__main__':
    folder = sys.argv[1] if len(sys.argv) > 1 else r'C:\Users\death\Games\GTA\GTA San Andreas\data\paths'
    areas = load_all(folder)
    net = Network(areas)
    analyze(net, sys.stdout)
