"""GTA SA road network -> SUMO (plain XML, then netconvert), to evaluate it with SUMO's tools.

The network the game loads: data/Paths with the mod's files (modloader) over it. Each vehicle node is a SUMO node (z kept,
so bridges and tunnels don't meet); each direction of a link with lanes towards it is an edge with that many lanes. Speed by
the node speed class (0 slow, 1 normal, 2 freeway). Junctions whose arriving roads have lights are traffic_light nodes; the
GTA light cycle (1 north-south, 2 west-east) goes into the edge 'type' so the analysis can check it.

Representative run (Gemini's review of the first one): junction nodes within 18 m joined (dual carriageways are 2-4 GTA
nodes), lights of one junction joined and actuated, freeway ramps guessed (no stop where a ramp merges), U-turns allowed
except at lights. Roads outside the strongly connected main network and parking spots are kept in
the network but open to emergency vehicles only, so random trips never start, end or route there.

usage: python sa_to_sumo.py <data/Paths> <override folder or -> <out dir>
writes out/gta.nod.xml, out/gta.edg.xml and runs netconvert -> out/gta.net.xml
"""
import math
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402

SPEED = {0: 40 / 3.6, 1: 70 / 3.6, 2: 110 / 3.6, 3: 110 / 3.6}


def nid(k):
    return 'n%d_%d' % k


def merged_paths(base, override):
    tmp = tempfile.mkdtemp(prefix='gta_paths_')
    for i in range(64):
        name = 'NODES%d.DAT' % i
        src = os.path.join(override, name) if override and override != '-' and os.path.exists(os.path.join(override, name)) \
            else os.path.join(base, name)
        if os.path.exists(src):
            shutil.copy(src, os.path.join(tmp, name))
    return tmp


def main(base, override, out):
    os.makedirs(out, exist_ok=True)
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    roads = {k for k, n in node.items() if not n.boats}
    from network_check import scc
    succ = {k: [b for b, nv in net.adj[k] if b in roads and nv and net.lanes_toward(k, b, nv) > 0] for k in roads}
    comp = scc(list(roads), succ)
    from collections import Counter
    main_c = Counter(comp.values()).most_common(1)[0][0]
    closed = {k for k in roads if comp[k] != main_c or node[k].parking}  # bit 8 ("emergency") is on 7669 normal roads
    lit = set()
    for k in roads:
        if node[k].links < 3:
            continue
        for nb, nv in net.adj[k]:
            if nv and nv.light and (nv.light_dir == 1) == ((nv.tarea, nv.tnode) == k):
                lit.add(k)
                break
    with open(os.path.join(out, 'gta.nod.xml'), 'w') as f:
        f.write('<nodes>\n')
        for k in sorted(roads):
            n = node[k]
            t = 'traffic_light' if k in lit else 'priority'
            f.write('  <node id="%s" x="%.2f" y="%.2f" z="%.2f" type="%s"/>\n' % (nid(k), n.x, n.y, n.z, t))
        f.write('</nodes>\n')
    edges = 0
    with open(os.path.join(out, 'gta.edg.xml'), 'w') as f:
        f.write('<edges>\n')
        for a in sorted(roads):
            for b, nv in net.adj[a]:
                if b not in roads or not nv:
                    continue
                lanes = net.lanes_toward(a, b, nv)
                if lanes <= 0:
                    continue
                na, nb = node[a], node[b]
                if math.hypot(na.x - nb.x, na.y - nb.y) < 0.5:
                    continue
                speed = SPEED[na.speed]
                light = nv.light if (nv.light and (nv.light_dir == 1) == ((nv.tarea, nv.tnode) == b)) else 0
                allow = ' allow="emergency"' if (a in closed or b in closed) else ''
                f.write('  <edge id="%s__%s" from="%s" to="%s" numLanes="%d" speed="%.2f" type="gta_light%d"%s/>\n' % (
                    nid(a), nid(b), nid(a), nid(b), lanes, speed, light, allow))
                edges += 1
        f.write('</edges>\n')
    print('nodes %d (traffic light junctions %d, closed to traffic %d), edges %d' % (len(roads), len(lit), len(closed), edges))
    import sumo
    netconvert = os.path.join(sumo.SUMO_HOME, 'bin', 'netconvert')
    cmd = [netconvert, '-n', os.path.join(out, 'gta.nod.xml'), '-e', os.path.join(out, 'gta.edg.xml'),
           '-o', os.path.join(out, 'gta.net.xml'), '--geometry.remove', '--no-turnarounds.tls',
           '--junctions.join', '--junctions.join-dist', '18', '--tls.join', '--tls.join-dist', '25',
           '--tls.default-type', 'actuated', '--ramps.guess', '--default.junctions.keep-clear', 'false',
           '--tls.guess-signals', 'false', '--ignore-errors', '--no-warnings', '--offset.disable-normalization']
    r = subprocess.run(cmd, capture_output=True, text=True)
    print(r.stdout[-1500:], r.stderr[-1500:])


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3])
