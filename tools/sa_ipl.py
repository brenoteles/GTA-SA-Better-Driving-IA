"""Map object instances of GTA SA: text IPL files (data/maps) and the binary stream IPLs inside gta3.img.

Inst = (model id, model name or '', x, y, z, qx, qy, qz, qw, interior, lod, source)
"""
import glob
import os
import struct

TRAFFIC_LIGHT_MODELS = {1283: 'MTraffic1', 1284: 'MTraffic2', 1315: 'trafficlight1', 1350: 'CJ_TRAFFIC_LIGHT4',
                        1351: 'CJ_TRAFFIC_LIGHT5', 1352: 'CJ_TRAFFIC_LIGHT3', 1262: 'MTraffic4', 1263: 'MTraffic3'}


def img_entries(img_path):
    """(name, offset bytes, size bytes) of a VER2 img archive"""
    with open(img_path, 'rb') as f:
        if f.read(4) != b'VER2':
            raise ValueError('not a VER2 img: ' + img_path)
        count = struct.unpack('<I', f.read(4))[0]
        out = []
        for _ in range(count):
            off, stream_size, arch_size = struct.unpack('<IHH', f.read(8))
            name = f.read(24).split(b'\0')[0].decode('latin-1')
            out.append((name, off * 2048, (arch_size or stream_size) * 2048))
        return out


def binary_ipl_insts(data, source):
    if data[:4] != b'bnry':
        return []
    num_inst = struct.unpack_from('<i', data, 4)[0]
    inst_off = struct.unpack_from('<i', data, 0x1C)[0]
    out = []
    for i in range(num_inst):
        x, y, z, qx, qy, qz, qw, mid, interior, lod = struct.unpack_from('<7f3i', data, inst_off + i * 40)
        out.append((mid, '', x, y, z, qx, qy, qz, qw, interior, lod, source))
    return out


def text_ipl_insts(path):
    out = []
    section = None
    with open(path, encoding='latin-1') as f:
        for line in f:
            s = line.split('#')[0].strip()
            if not s:
                continue
            low = s.lower()
            if low in ('inst', 'cull', 'path', 'grge', 'enex', 'pick', 'cars', 'jump', 'tcyc', 'auzo', 'mult', 'occl', 'zone'):
                section = low
                continue
            if low == 'end':
                section = None
                continue
            if section != 'inst':
                continue
            p = [t.strip() for t in s.split(',')]
            if len(p) < 11:
                continue
            out.append((int(p[0]), p[1], float(p[3]), float(p[4]), float(p[5]), float(p[6]), float(p[7]), float(p[8]),
                        float(p[9]), int(p[2]), int(p[10]), os.path.basename(path)))
    return out


def all_insts(game_dir, img_name='models/gta3.img'):
    insts = []
    for path in glob.glob(os.path.join(game_dir, 'data', 'maps', '**', '*.ipl'), recursive=True):
        insts += text_ipl_insts(path)
    img = os.path.join(game_dir, img_name)
    with open(img, 'rb') as f:
        for name, off, size in img_entries(img):
            if not name.lower().endswith('.ipl'):
                continue
            f.seek(off)
            insts += binary_ipl_insts(f.read(size), name)
    return insts


if __name__ == '__main__':
    import sys
    from collections import Counter
    insts = all_insts(sys.argv[1])
    lights = [i for i in insts if i[0] in TRAFFIC_LIGHT_MODELS]
    print('instances:', len(insts), 'traffic light poles:', len(lights))
    print(Counter(TRAFFIC_LIGHT_MODELS[i[0]] for i in lights))
