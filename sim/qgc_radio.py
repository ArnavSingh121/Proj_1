#!/usr/bin/env python3
# Sets up an RC transmitter (EdgeTX/OpenTX radio, e.g. Radiomaster Pocket) connected over USB as
# QGroundControl's joystick, instead of QGroundControl's default layout, which is an Xbox pad's.
# EdgeTX sends its first 8 channels as axes in the radio's channel order, AETR by default (as
# /dev/input/js* shows them): 0 roll, 1 pitch, 2 throttle, 3 yaw, 4-7 switches (channels 5-8); stick
# up/right is positive. But the SDL in QGroundControl 5.0.8 (2.0.20) has a gamepad mapping for EdgeTX
# radios, so QGroundControl reads them as a gamepad, with these axes instead:
#   0 leftx = roll    1 lefty = pitch    2 rightx = yaw    3 righty = channel 5 (switch)
#   4 lefttrigger = throttle, 0 (stick down) to 32767 (up)    5 righttrigger = channel 6 (switch)
# QGroundControl's default layout, an Xbox pad's, took switch and trigger axes for yaw and roll, so the
# drone turned or flew on its own. Each joystick keeps its own settings in QGroundControl (by name),
# so the gamepad's are left as they are.
# QGroundControl writes its settings back when it exits, so it must be closed first.
# Usage: python3 sim/qgc_radio.py [--name "EdgeTX Radiomaster Pocket Joystick"] [--settings FILE]
import argparse
import glob
import os
import shutil
import sys
from pathlib import Path

SETTINGS = Path.home() / '.config/QGroundControl/QGroundControl.ini'
# Names under which radios in USB joystick mode show up.
RADIO_MAKERS = ('EdgeTX', 'OpenTX')
AXES = 8  # QGroundControl keeps a calibration for each of the radio's 8 axes, though it reads 6
THROTTLE_AXIS = 4


def is_radio(name):
    return any(maker in name for maker in RADIO_MAKERS)


def radio_settings():
    """QGroundControl joystick settings for an AETR radio read as a gamepad (see the top): calibrated
    full range, no axis reversed, throttle from stick down (none) to stick up (full), which PX4 holds
    height at in the middle. The gimbal axes, unused on the x500, go to the switches, off the throttle."""
    settings = {'RollAxis': 0, 'PitchAxis': 1, 'YawAxis': 2, 'ThrottleAxis': THROTTLE_AXIS, 'ThrottleMode': 1,
                'GimbalPitchAxis': 3, 'GimbalYawAxis': 5, 'Calibrated4': 'true', 'Deadband': 'false'}
    for axis in range(AXES):
        settings.update({f'Axis{axis}Min': -32767, f'Axis{axis}Max': 32767, f'Axis{axis}Trim': 0,
                         f'Axis{axis}Rev': 'false', f'Axis{axis}Deadbnd': 0})
    # A trigger axis: 0 to 32767, middle at 16384.
    settings.update({f'Axis{THROTTLE_AXIS}Min': 0, f'Axis{THROTTLE_AXIS}Trim': 16384})
    return {key: str(value) for key, value in settings.items()}


def settings_key(name):
    """A joystick name as QSettings writes it in an .ini key (space -> %20, ...)."""
    return ''.join(c if c.isascii() and (c.isalnum() or c in '_-.') else f'%{ord(c):02X}' for c in name)


def apply(lines, name, settings):
    """Lines of QGroundControl.ini with the joystick `name` set to `settings` and made the active one.
    Other lines are kept as they are."""
    prefix = settings_key(name) + '\\'
    wanted = {'Joysticks': {prefix + key: value for key, value in settings.items()},
              'JoystickManager': {'ActiveJoystick': name}}
    out, section = [], None

    def finish(section):
        # Keys the section did not have yet go at its end, before the blank line between sections.
        end = len(out)
        while end and not out[end - 1].strip():
            end -= 1
        out[end:end] = [f'{key}={value}' for key, value in wanted.pop(section, {}).items()]

    for line in lines:
        stripped = line.strip()
        if stripped.startswith('[') and stripped.endswith(']'):
            finish(section)
            section = stripped[1:-1]
        elif section in wanted and '=' in line:
            key = line.split('=', 1)[0]
            if key in wanted[section]:
                line = f'{key}={wanted[section].pop(key)}'
        out.append(line)
    finish(section)
    for section in list(wanted):  # sections the file did not have
        if out and out[-1].strip():
            out.append('')
        out.append(f'[{section}]')
        finish(section)
    return out


def connected_radios():
    """Names of the radios plugged in as joysticks."""
    names = []
    for device in sorted(glob.glob('/sys/class/input/js*/device/name')):
        name = Path(device).read_text().strip()
        if is_radio(name):
            names.append(name)
    return names


def qgc_running():
    for comm in glob.glob('/proc/[0-9]*/comm'):
        try:
            if Path(comm).read_text().strip() == 'QGroundControl':
                return True
        except OSError:
            pass
    return False


def main():
    parser = argparse.ArgumentParser(description='Set up a USB RC radio (EdgeTX/OpenTX) in QGroundControl.')
    parser.add_argument('--name', help='joystick name as QGroundControl shows it (default: the plugged-in radio)')
    parser.add_argument('--settings', type=Path, default=SETTINGS)
    args = parser.parse_args()

    names = [args.name] if args.name else connected_radios()
    if not names:
        sys.exit('No EdgeTX/OpenTX radio plugged in. Plug it in (USB, Joystick mode) or pass --name.')
    if len(names) > 1:
        sys.exit(f'Several radios plugged in ({", ".join(names)}): pick one with --name.')
    if qgc_running():
        sys.exit('Close QGroundControl first: it writes its own settings back when it exits.')
    if not args.settings.exists():
        sys.exit(f'No {args.settings}: start QGroundControl once (make qgc-run), close it, then run this again.')

    backup = args.settings.with_name(args.settings.name + '.bak')
    shutil.copy2(args.settings, backup)
    lines = apply(args.settings.read_text().splitlines(), names[0], radio_settings())
    tmp = args.settings.with_name(args.settings.name + '.tmp')
    tmp.write_text('\n'.join(lines) + '\n')
    os.replace(tmp, args.settings)
    print(f'{names[0]}: right stick roll/pitch, left stick throttle/yaw (RC Mode 2, AETR), now the active joystick.')
    print(f'Previous settings saved as {backup}. Start QGroundControl (make qgc-run) and check the')
    print('sliders on Vehicle Configuration -> Joystick -> General.')


if __name__ == '__main__':
    main()
