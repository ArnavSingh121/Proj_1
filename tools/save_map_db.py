#!/usr/bin/env python3
# Saves a whole RTAB-Map map to maps/<name>_<time>/: a copy of the database (reload it to continue mapping
# or to localize), the 2D occupancy grid (map.pgm + map.yaml, the ROS map_server format), the 3D point
# cloud (cloud.ply) and the path walked (poses.txt). tools/save_map.py saves only the 2D grid.
# Run it while mapping: RTAB-Map first flushes the map to disk (`backup`, ~3 s pause). Without RTAB-Map
# running it exports what is in the database (no 2D grid then).
import argparse
import datetime
import os
import shutil
import subprocess
import sys
import time

import numpy as np
import rclpy
from nav_msgs.srv import GetMap
from rclpy.node import Node
from std_srvs.srv import Empty

RTABMAP = '/rtabmap/rtabmap/'


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=5.0):
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    return future.result()


def write_grid(grid, folder):
    """OccupancyGrid -> map.pgm + map.yaml (free white, occupied black, unknown grey)."""
    w, h, info = grid.info.width, grid.info.height, grid.info
    cells = np.array(grid.data, dtype=np.int16).reshape(h, w)[::-1]  # PGM starts at the top row
    pgm = np.full((h, w), 205, np.uint8)  # unknown: grey
    pgm[(cells >= 0) & (cells < 25)] = 254  # free: white
    pgm[cells > 65] = 0  # occupied: black
    with open(os.path.join(folder, 'map.pgm'), 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode() + pgm.tobytes())
    o = info.origin.position
    with open(os.path.join(folder, 'map.yaml'), 'w') as f:
        f.write(f'image: map.pgm\nmode: trinary\nresolution: {info.resolution}\n'
                f'origin: [{o.x}, {o.y}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return f'{w}x{h} cells at {info.resolution * 100:.0f} cm ({w * info.resolution:.1f} x {h * info.resolution:.1f} m)'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--db', required=True, help='the database RTAB-Map maps into, e.g. maps/halls.db')
    ap.add_argument('--out', help='output folder (default: <db folder>/<name>_<date-time>)')
    a = ap.parse_args()
    db = os.path.abspath(os.path.expanduser(a.db))
    name = os.path.splitext(os.path.basename(db))[0]
    out = a.out or os.path.join(os.path.dirname(db), f'{name}_{datetime.datetime.now():%Y%m%d-%H%M%S}')
    os.makedirs(out, exist_ok=True)

    rclpy.init()
    node = Node('save_map')
    snapshot = db
    asked = time.time()
    if call(node, node.create_client(Empty, RTABMAP + 'backup'), Empty.Request(), 60.0) is not None:
        # Flushed to <db>.back; only valid if the running RTAB-Map maps into this database.
        if os.path.exists(db + '.back') and os.path.getmtime(db + '.back') >= asked - 1:
            snapshot = db + '.back'
            print(f'Flushed the running map to {snapshot}')
        else:
            print(f'warning: the running RTAB-Map is not mapping into {db}; exporting the file as it is')
        grid = call(node, node.create_client(GetMap, RTABMAP + 'get_map'), GetMap.Request(), 60.0)
        if grid is not None and grid.map.info.width:
            print('2D grid: ' + write_grid(grid.map, out))
        else:
            print('warning: no 2D grid from RTAB-Map')
    else:
        print('RTAB-Map is not running: exporting the database only (no 2D grid)')
    node.destroy_node()
    rclpy.try_shutdown()

    if not os.path.exists(snapshot):
        sys.exit(f'error: {snapshot} does not exist')
    shutil.copy2(snapshot, os.path.join(out, name + '.db'))
    # Same 0.3 m near limit as mapping; 5 m far limit and noise filter for CPU stereo depth.
    export = ['rtabmap-export', '--cloud', '--poses', '--min_range', '0.3', '--max_range', '5',
              '--voxel', '0.03', '--noise_radius', '0.1', '--noise_k', '5',
              '--output_dir', out, '--output', name, os.path.join(out, name + '.db')]
    r = subprocess.run(export, capture_output=True, text=True)
    for produced, final in ((name + '_cloud.ply', 'cloud.ply'), (name + '_poses.txt', 'poses.txt')):
        if os.path.exists(os.path.join(out, produced)):
            os.replace(os.path.join(out, produced), os.path.join(out, final))
    if r.returncode != 0 or not os.path.exists(os.path.join(out, 'cloud.ply')):
        print(r.stdout[-2000:], r.stderr[-2000:])
        sys.exit('error: rtabmap-export failed')
    with open(os.path.join(out, 'cloud.ply'), 'rb') as f:
        header = f.read(400).decode(errors='ignore')
    points = next((l.split()[-1] for l in header.splitlines() if l.startswith('element vertex')), '?')
    print(f'3D cloud: {points} points')
    print(f'Saved to {out}')


if __name__ == '__main__':
    main()
