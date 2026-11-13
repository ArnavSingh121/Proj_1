# QGroundControl joystick settings for a USB RC radio (sim/qgc_radio.py), on a copy of the settings file.
from qgc_radio import apply, is_radio, radio_settings, settings_key

RADIO = 'EdgeTX Radiomaster Pocket Joystick'
KEY = 'EdgeTX%20Radiomaster%20Pocket%20Joystick\\'
# As QGroundControl 5.0 left it: the radio got the Xbox pad's default layout.
INI = f"""[General]
virtualJoystick=true

[JoystickManager]
ActiveJoystick=Generic X-Box pad

[Joysticks]
{KEY}Axis1Rev=true
{KEY}PitchAxis=3
{KEY}RollAxis=2
{KEY}ThrottleAxis=1
{KEY}YawAxis=0
Generic%20X-Box%20pad\\ThrottleAxis=1
TXMode_MultiRotor=2

[Vehicle1]
JoystickEnabled=true
""".splitlines()


def values(lines):
    section, found = None, {}
    for line in lines:
        if line.startswith('['):
            section = line[1:-1]
        elif '=' in line:
            key, value = line.split('=', 1)
            assert (section, key) not in found, f'{key} twice in [{section}]'
            found[section, key] = value
    return found


def test_radios_are_told_apart_from_gamepads():
    assert is_radio(RADIO) and is_radio('OpenTX Jumper T-Lite Joystick')
    assert not is_radio('Generic X-Box pad') and not is_radio('PowerA Xbox Series X Controller')


def test_name_is_escaped_like_qsettings():
    assert settings_key(RADIO) == KEY[:-1]
    assert settings_key('Pad (v2)') == 'Pad%20%28v2%29'


def test_radio_gets_aetr_layout_without_reversed_axes():
    s = radio_settings()
    assert (s['RollAxis'], s['PitchAxis'], s['ThrottleAxis'], s['YawAxis']) == ('0', '1', '2', '3')
    assert all(s[f'Axis{i}Rev'] == 'false' for i in range(8))
    assert s['ThrottleMode'] == '1'  # stick down = no throttle, as on a radio; not centered


def test_apply_replaces_the_radio_keys_and_keeps_everything_else():
    out = values(apply(INI, RADIO, radio_settings()))
    before = values(INI)
    assert out['Joysticks', KEY + 'ThrottleAxis'] == '2' and out['Joysticks', KEY + 'Axis1Rev'] == 'false'
    assert out['Joysticks', KEY + 'Axis7Max'] == '32767'  # added
    assert out['JoystickManager', 'ActiveJoystick'] == RADIO
    for key in [('Joysticks', 'Generic%20X-Box%20pad\\ThrottleAxis'), ('Joysticks', 'TXMode_MultiRotor'),
                ('General', 'virtualJoystick'), ('Vehicle1', 'JoystickEnabled')]:
        assert out[key] == before[key]
    radio_keys = {key for section, key in out if section == 'Joysticks' and key.startswith(KEY)}
    assert radio_keys == {KEY + key for key in radio_settings()}


def test_new_keys_go_inside_their_section_and_sections_stay_apart():
    out = apply(INI, RADIO, radio_settings())
    joysticks = out.index('[Joysticks]')
    vehicle = out.index('[Vehicle1]')
    assert out[vehicle - 1] == ''
    assert all(line.startswith((KEY, 'Generic', 'TXMode')) for line in out[joysticks + 1:vehicle - 1])


def test_apply_adds_missing_sections_and_is_idempotent():
    out = apply(['[General]', 'virtualJoystick=true'], RADIO, radio_settings())
    assert values(out)['JoystickManager', 'ActiveJoystick'] == RADIO
    assert values(out)['Joysticks', KEY + 'YawAxis'] == '3'
    assert apply(out, RADIO, radio_settings()) == out
