"""Directed connectivity of the GTA SA road network (what SUMO's netcheck does for weak components, done strongly):
which road nodes a car can reach and leave again. Nodes outside the largest strongly connected component are traps
(enter, can't get out) or unreachable pieces; listed with their position and why (dead end, one-way into a corner).

usage: python network_check.py <data/Paths> <override folder or -> [out.csv]
"""
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402


def scc(nodes, succ):
    """Kosaraju, iterative: component index per node"""
    order, seen = [], set()
    for s in nodes:
        if s in seen:
            continue
        seen.add(s)
        stack = [(s, iter(succ[s]))]
        while stack:
            v, it = stack[-1]
            nxt = next(it, None)
            if nxt is None:
                stack.pop()
                order.append(v)
            elif nxt not in seen:
                seen.add(nxt)
                stack.append((nxt, iter(succ[nxt])))
    pred = defaultdict(list)
    for v in nodes:
        for w in succ[v]:
            pred[w].append(v)
    comp, c = {}, 0
    for s in reversed(order):
        if s in comp:
            continue
        stack = [s]
        comp[s] = c
        while stack:
            v = stack.pop()
            for w in pred[v]:
                if w not in comp:
                    comp[w] = c
                    stack.append(w)
        c += 1
    return comp


def main(base, override, out_csv=None):
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    roads = [k for k, n in node.items() if not n.boats]
    rs = set(roads)
    succ = defaultdict(list)
    for a in roads:
        for b, nv in net.adj[a]:
            if b in rs and nv and net.lanes_toward(a, b, nv) > 0:
                succ[a].append(b)
    comp = scc(roads, succ)
    sizes = Counter(comp.values())
    main_c = sizes.most_common(1)[0][0]
    outside = [k for k in roads if comp[k] != main_c]
    # Why: can it get into the main network (reach), can the main network reach it
    main_nodes = {k for k in roads if comp[k] == main_c}

    def reach(starts, graph):
        seen = set(starts)
        stack = list(starts)
        while stack:
            v = stack.pop()
            for w in graph[v]:
                if w not in seen:
                    seen.add(w)
                    stack.append(w)
        return seen
    pred = defaultdict(list)
    for v in roads:
        for w in succ[v]:
            pred[w].append(v)
    from_main = reach(main_nodes, succ)       # reachable from the main network
    to_main = reach(main_nodes, pred)         # can reach the main network
    kinds = Counter()
    rows = []
    for k in outside:
        if k in from_main and k not in to_main:
            kind = 'trap (enter, no way out)'
        elif k in to_main and k not in from_main:
            kind = 'source (leave, no way in)'
        else:
            kind = 'island'
        kinds[kind] += 1
        n = node[k]
        rows.append((kind, k, n.x, n.y, n.z, len(succ[k]), n.speed))
    print('road nodes %d, strongly connected main network %d (%.1f%%), components %d' % (
        len(roads), len(main_nodes), 100.0 * len(main_nodes) / len(roads), len(sizes)))
    for kind, c in kinds.most_common():
        print('  %-28s %d nodes' % (kind, c))
    dead_ends = sum(1 for k in roads if not succ[k])
    print('  nodes with no way on (dead ends for cars): %d' % dead_ends)
    if out_csv:
        with open(out_csv, 'w') as f:
            f.write('kind,area,node,x,y,z,exits,speed\n')
            for kind, k, x, y, z, ex, sp in sorted(rows):
                f.write('%s,%d,%d,%.1f,%.1f,%.1f,%d,%d\n' % (kind, k[0], k[1], x, y, z, ex, sp))
    return rows


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
