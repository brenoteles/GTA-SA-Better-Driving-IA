"""Where traffic gets stuck in the SUMO run of the GTA network: per junction (the GTA node an edge ends at), the time
vehicles spent waiting on the edges arriving there and the teleports (jam / yield) that started there.

usage: python sumo_hotspots.py <sumo dir with edgedata.xml and the sumo log> <log file> [top N]
"""
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

NODE = re.compile(r'n(\d+)_(\d+)')


def end_node(edge_or_lane):
    """GTA node an edge/lane id ends at ('n12_299__n12_309_1' -> (12, 309); internal ':n12_298_4_0' -> (12, 298))"""
    if edge_or_lane.startswith(':'):
        m = NODE.match(edge_or_lane[1:])
        return (int(m.group(1)), int(m.group(2))) if m else None
    parts = edge_or_lane.split('__')
    if len(parts) < 2:
        return None
    m = NODE.match(parts[1])
    return (int(m.group(1)), int(m.group(2))) if m else None


def main(sumo_dir, log, top=25):
    wait = Counter()
    for e in ET.parse(sumo_dir + '/edgedata.xml').getroot().iter('edge'):
        k = end_node(e.get('id'))
        if k:
            wait[k] += float(e.get('waitingTime', 0))
    tele = defaultdict(Counter)
    for line in open(log, encoding='utf-8', errors='replace'):
        m = re.search(r"Teleporting vehicle '[^']+'; (?:waited too long|collision) \(?([a-z ]+)\)?, lane='([^']+)'", line)
        if m:
            k = end_node(m.group(2))
            if k:
                tele[k][m.group(1).strip()] += 1
    nodes = {}
    for n in ET.parse(sumo_dir + '/gta.nod.xml').getroot().iter('node'):
        m = NODE.match(n.get('id'))
        nodes[(int(m.group(1)), int(m.group(2)))] = (float(n.get('x')), float(n.get('y')), float(n.get('z')), n.get('type'))
    score = Counter()
    for k, w in wait.items():
        score[k] += w / 60.0
    for k, c in tele.items():
        score[k] += 10 * sum(c.values())
    print('%-10s %8s %8s %-22s %s' % ('node', 'wait min', 'telep', 'kind', 'x, y, z'))
    for k, s in score.most_common(top):
        x, y, z, t = nodes.get(k, (0, 0, 0, '?'))
        tc = tele.get(k, Counter())
        print('%-10s %8.0f %8d %-22s %.0f, %.0f, %.0f   %s' % ('%d:%d' % k, wait[k] / 60.0, sum(tc.values()), t, x, y, z,
                                                              dict(tc)))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 25)
