# Flight bookkeeping without ROS, used by flight_recorder.py (and the tests).
import datetime
import glob
import json
import os
import re

MIN_POSES = 3  # fewer map poses: armed on the bench without really moving; never replaces `latest`


class ArmWatcher:
    """Turns PX4's armed state into flight events: 'takeoff', 'joined' or 'landed'."""

    def __init__(self):
        self.armed = None  # unknown until PX4 is first heard from

    def update(self, connected, armed):
        # A lost link says nothing about the flight, so it must not read as a landing. It would also
        # turn the reconnection into a new takeoff, which wipes the map in the middle of a flight.
        if not connected:
            return None
        before, self.armed = self.armed, armed
        if armed and before is None:
            return 'joined'  # started during a flight (e.g. a restart): keep the map
        if armed and before is False:
            return 'takeoff'
        if not armed and before:
            return 'landed'
        return None


def new_flight_folder(root, now):
    """Creates and returns <root>/<date>_<time>[-n]/ for a flight starting at datetime `now`."""
    os.makedirs(root, exist_ok=True)
    base = os.path.join(root, f'{now:%Y-%m-%d_%H-%M-%S}')
    path, n = base, 1
    while True:
        try:
            os.mkdir(path)
            return path
        except FileExistsError:
            n += 1
            path = f'{base}-{n}'


def set_latest(root, folder):
    """Points <root>/latest at `folder`, in one step so readers never find it missing."""
    tmp = os.path.join(root, '.latest.tmp')
    if os.path.lexists(tmp):
        os.remove(tmp)
    os.symlink(os.path.basename(folder), tmp)
    os.replace(tmp, os.path.join(root, 'latest'))


def write_manifest(folder, **fields):
    """Writes <folder>/flight.json in one step, so a power cut can't leave half a file."""
    tmp = os.path.join(folder, '.flight.json.tmp')
    with open(tmp, 'w') as f:
        json.dump(fields, f, indent=2)
        f.write('\n')
    os.replace(tmp, os.path.join(folder, 'flight.json'))


def count_poses(path):
    """Number of poses in a poses.txt, 0 if it doesn't exist."""
    if not os.path.exists(path):
        return 0
    with open(path) as f:
        return sum(1 for line in f if line.strip() and not line.startswith('#'))


def bag_files(bag_dir):
    """The storage files of a ros bag folder, written while it records."""
    return sorted(glob.glob(os.path.join(bag_dir, '*.db3')) + glob.glob(os.path.join(bag_dir, '*.mcap')))


def bag_summary(bag_dir):
    """{seconds, messages, megabytes} of a closed ros bag, from its metadata.yaml; None if it has none."""
    meta = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.exists(meta):
        return None
    with open(meta) as f:
        text = f.read()
    duration = re.search(r'^  duration:\s*\n\s+nanoseconds:\s*(\d+)', text, re.M)
    messages = re.search(r'^  message_count:\s*(\d+)', text, re.M)
    size = sum(os.path.getsize(p) for p in bag_files(bag_dir))
    return {'seconds': round(int(duration.group(1)) / 1e9, 1) if duration else None,
            'messages': int(messages.group(1)) if messages else None,
            'megabytes': round(size / 1e6, 1)}


def timestamp(now=None):
    return (now or datetime.datetime.now()).isoformat(timespec='seconds')
