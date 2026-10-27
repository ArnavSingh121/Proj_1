# Occupancy-grid helpers without ROS, shared by save_map.py, load_map.py and map_monitor.py (and the tests).
# Files use the ROS map_server format: a PGM image plus a YAML file with resolution and origin, so a saved
# map also opens in nav2's map_server and in any image viewer.
import math
import os

# map_saver's thresholds ("trinary" mode). FREE_THRESH sits just below the unknown gray (205 -> 0.196...),
# so unknown cells read back as unknown.
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.196
# PGM gray levels: black = occupied, white = free, gray = unknown.
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205


def grid_stats(data):
    """(known, occupied, free) cell counts of OccupancyGrid data (-1 unknown, 0-100 occupancy %)."""
    occupied = sum(1 for v in data if v >= OCCUPIED_THRESH * 100)
    free = sum(1 for v in data if 0 <= v < FREE_THRESH * 100)
    known = sum(1 for v in data if v >= 0)
    return known, occupied, free


def to_pgm(width, height, data):
    """Binary PGM (P5) bytes of an OccupancyGrid. ROS grids start at the bottom-left; images at the top-left."""
    if len(data) != width * height:
        raise ValueError(f'grid has {len(data)} cells, expected {width}x{height}')
    pixels = bytearray(width * height)
    for row in range(height):
        src = (height - 1 - row) * width
        for col in range(width):
            v = data[src + col]
            if v < 0:
                pixels[row * width + col] = PGM_UNKNOWN
            elif v >= OCCUPIED_THRESH * 100:
                pixels[row * width + col] = PGM_OCCUPIED
            elif v < FREE_THRESH * 100:
                pixels[row * width + col] = PGM_FREE
            else:
                pixels[row * width + col] = PGM_UNKNOWN
    return b'P5\n%d %d\n255\n' % (width, height) + bytes(pixels)


def from_pgm(raw):
    """(width, height, data) of OccupancyGrid data from PGM (P5) bytes written by to_pgm or map_saver."""
    tokens, pos = [], 0
    while len(tokens) < 4:
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b'#':  # comment line
            pos = raw.index(b'\n', pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    if tokens[0] != b'P5':
        raise ValueError('not a binary PGM (P5) file')
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    pixels = raw[pos + 1:pos + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError('PGM file is truncated')
    data = [0] * (width * height)
    for row in range(height):
        dst = (height - 1 - row) * width
        for col in range(width):
            # map_server: occupancy = (255 - pixel) / 255, then the trinary thresholds
            p = (maxval - pixels[row * width + col]) / maxval
            if p >= OCCUPIED_THRESH:
                data[dst + col] = 100
            elif p < FREE_THRESH:
                data[dst + col] = 0
            else:
                data[dst + col] = -1
    return width, height, data


def map_yaml(image, resolution, origin):
    """map_server YAML text. origin = (x, y, yaw) of the bottom-left cell, in the map frame."""
    x, y, yaw = origin
    return (f'image: {image}\nmode: trinary\nresolution: {resolution}\n'
            f'origin: [{x}, {y}, {yaw}]\nnegate: 0\n'
            f'occupied_thresh: {OCCUPIED_THRESH}\nfree_thresh: {FREE_THRESH}\n')


def parse_map_yaml(text):
    """(image, resolution, origin) from map_server YAML (only the keys map_yaml writes; no PyYAML needed)."""
    values = {}
    for line in text.splitlines():
        key, sep, value = line.partition(':')
        if sep:
            values[key.strip()] = value.strip()
    origin = [float(v) for v in values['origin'].strip('[]').split(',')]
    return values['image'], float(values['resolution']), tuple(origin)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def save(path_base, width, height, data, resolution, origin):
    """Writes <path_base>.pgm and <path_base>.yaml; returns the YAML path."""
    os.makedirs(os.path.dirname(os.path.abspath(path_base)), exist_ok=True)
    with open(path_base + '.pgm', 'wb') as f:
        f.write(to_pgm(width, height, data))
    with open(path_base + '.yaml', 'w') as f:
        f.write(map_yaml(os.path.basename(path_base) + '.pgm', resolution, origin))
    return path_base + '.yaml'


def load(yaml_path):
    """(width, height, data, resolution, origin) of a map saved by save() or nav2's map_saver."""
    with open(yaml_path) as f:
        image, resolution, origin = parse_map_yaml(f.read())
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    with open(image, 'rb') as f:
        width, height, data = from_pgm(f.read())
    return width, height, data, resolution, origin
