# Every flight leaves a map: arm and disarm become flight events, and flight folders stay trustworthy.
import datetime
import json
import os

import pytest

import flight_tools
from flight_tools import ArmWatcher

NOW = datetime.datetime(2026, 11, 5, 17, 5, 12)


def events(*states):
    w = ArmWatcher()
    return [w.update(connected, armed) for connected, armed in states]


def test_a_flight_is_a_takeoff_then_a_landing():
    assert events((True, False), (True, True), (True, True), (True, False)) == [None, 'takeoff', None, 'landed']


def test_every_flight_of_a_session_is_seen():
    assert events((True, False), (True, True), (True, False), (True, True), (True, False)) == \
        [None, 'takeoff', 'landed', 'takeoff', 'landed']


def test_starting_during_a_flight_keeps_the_map():
    assert events((True, True), (True, False)) == ['joined', 'landed']


def test_a_lost_link_is_neither_a_landing_nor_a_new_takeoff():
    assert events((True, False), (True, True), (False, False), (True, True), (True, False)) == \
        [None, 'takeoff', None, None, 'landed']


def test_a_landing_during_a_lost_link_is_seen_on_reconnection():
    assert events((True, False), (True, True), (False, False), (True, False)) == [None, 'takeoff', None, 'landed']


def test_nothing_happens_before_px4_is_connected():
    assert events((False, False), (False, True)) == [None, None]


def test_flight_folders_are_named_by_time_and_never_shared(tmp_path):
    first = flight_tools.new_flight_folder(str(tmp_path / 'flights'), NOW)
    second = flight_tools.new_flight_folder(str(tmp_path / 'flights'), NOW)
    assert os.path.basename(first) == '2026-11-05_17-05-12'
    assert os.path.basename(second) == '2026-11-05_17-05-12-2'
    assert os.path.isdir(first) and os.path.isdir(second)


def test_latest_follows_the_newest_flight(tmp_path):
    root = str(tmp_path)
    for name in ('a', 'b'):
        folder = tmp_path / name
        folder.mkdir()
        (folder / 'map.yaml').write_text(name)
        flight_tools.set_latest(root, str(folder))
        assert (tmp_path / 'latest' / 'map.yaml').read_text() == name
    assert os.readlink(tmp_path / 'latest') == 'b'  # relative: the folder can be copied off the drone


def test_manifest_round_trip_leaves_no_temporary_file(tmp_path):
    flight_tools.write_manifest(str(tmp_path), status='saved', poses=12)
    assert json.loads((tmp_path / 'flight.json').read_text()) == {'status': 'saved', 'poses': 12}
    assert os.listdir(tmp_path) == ['flight.json']


@pytest.mark.parametrize('text, count', [('', 0), ('# header\n', 0), ('1 0 0\n\n2 0 0\n', 2)])
def test_count_poses(tmp_path, text, count):
    (tmp_path / 'poses.txt').write_text(text)
    assert flight_tools.count_poses(str(tmp_path / 'poses.txt')) == count


def test_count_poses_of_a_missing_file(tmp_path):
    assert flight_tools.count_poses(str(tmp_path / 'poses.txt')) == 0


# The head of a real rosbag2 (Humble) metadata.yaml, trimmed.
METADATA = '''rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:
    nanoseconds: 52000219405
  starting_time:
    nanoseconds_since_epoch: 1793923867615240620
  message_count: 6244
  topics_with_message_count:
    - topic_metadata:
        name: /zed/zed_node/rgb/color/rect/image
      message_count: 1561
'''


def test_bag_summary_of_a_closed_recording(tmp_path):
    (tmp_path / 'metadata.yaml').write_text(METADATA)
    (tmp_path / 'bag_0.db3').write_bytes(b'x' * 2_500_000)
    assert flight_tools.bag_summary(str(tmp_path)) == {'seconds': 52.0, 'messages': 6244, 'megabytes': 2.5}


def test_a_recording_that_never_closed_has_no_summary_but_its_files_are_found(tmp_path):
    (tmp_path / 'bag_0.db3').write_bytes(b'x')
    assert flight_tools.bag_summary(str(tmp_path)) is None
    assert flight_tools.bag_files(str(tmp_path)) == [str(tmp_path / 'bag_0.db3')]
