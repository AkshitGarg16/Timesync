"""VISCA command/reply encoding for the USB3 NeoHD PTZ camera."""

ADDR = 0x81  # camera address 1

# ---------------------------------------------------------------------------
# Zoom
# ---------------------------------------------------------------------------

ZOOM_STOP = bytes([ADDR, 0x01, 0x04, 0x07, 0x00, 0xFF])
# 0x4000 (30x) is the documented optical-only max, but this camera's zoom
# drive continues seamlessly into digital zoom beyond that. Measured by
# driving zoom fully to tele repeatedly and reading CAM_ZoomPosInq each
# time (see calibrate_zoom.py) - consistently 0x7AC0, never 0x4000.
ZOOM_MAX = 0x7AC0

ZOOM_INQUIRY = bytes([ADDR, 0x09, 0x04, 0x47, 0xFF])


def zoom_drive(speed: int, direction: str) -> bytes:
    """Continuous zoom in/out at given speed (0-7)."""
    speed &= 0x07
    sub = 0x02 if direction == "in" else 0x03
    return bytes([ADDR, 0x01, 0x04, 0x07, (sub << 4) | speed, 0xFF])


def zoom_absolute(position: int) -> bytes:
    """Move to an absolute zoom position (0 = widest/min, 0x4000 = full tele)."""
    position = max(0, min(ZOOM_MAX, position))
    return bytes([ADDR, 0x01, 0x04, 0x47, *_pack_nibbles(position, 4), 0xFF])


def parse_zoom_inquiry(reply: bytes) -> int:
    """Parse a CAM_ZoomPosInq reply (yy 50 0p 0q 0r 0s FF) into 0-0x4000."""
    return _parse_position_reply(reply)


# ---------------------------------------------------------------------------
# Focus
# ---------------------------------------------------------------------------

FOCUS_STOP = bytes([ADDR, 0x01, 0x04, 0x08, 0x00, 0xFF])
FOCUS_AUTO = bytes([ADDR, 0x01, 0x04, 0x38, 0x02, 0xFF])
FOCUS_MANUAL = bytes([ADDR, 0x01, 0x04, 0x38, 0x03, 0xFF])
FOCUS_ONE_PUSH = bytes([ADDR, 0x01, 0x04, 0x18, 0x01, 0xFF])
FOCUS_MAX = 0xFFFF

FOCUS_INQUIRY = bytes([ADDR, 0x09, 0x04, 0x48, 0xFF])


def focus_drive(speed: int, direction: str) -> bytes:
    """Continuous focus far/near at given speed (0-7)."""
    speed &= 0x07
    sub = 0x02 if direction == "far" else 0x03
    return bytes([ADDR, 0x01, 0x04, 0x08, (sub << 4) | speed, 0xFF])


def focus_absolute(position: int) -> bytes:
    """Move to an absolute focus position (0 = near, 0xFFFF = far)."""
    position = max(0, min(FOCUS_MAX, position))
    return bytes([ADDR, 0x01, 0x04, 0x48, *_pack_nibbles(position, 4), 0xFF])


def parse_focus_inquiry(reply: bytes) -> int:
    """Parse a CAM_FocusPosInq reply (yy 50 0p 0q 0r 0s FF) into 0-0xFFFF."""
    return _parse_position_reply(reply)


# ---------------------------------------------------------------------------
# Exposure (AE)
# ---------------------------------------------------------------------------

AE_FULL_AUTO = bytes([ADDR, 0x01, 0x04, 0x39, 0x00, 0xFF])
AE_SHUTTER_PRIORITY = bytes([ADDR, 0x01, 0x04, 0x39, 0x0A, 0xFF])
AE_IRIS_PRIORITY = bytes([ADDR, 0x01, 0x04, 0x39, 0x0B, 0xFF])
AE_MANUAL = bytes([ADDR, 0x01, 0x04, 0x39, 0x03, 0xFF])

AE_MODES = {
    "auto": AE_FULL_AUTO,
    "shutter_priority": AE_SHUTTER_PRIORITY,
    "iris_priority": AE_IRIS_PRIORITY,
    "manual": AE_MANUAL,
}

SHUTTER_MAX = 21  # 22 steps, 1/1 to 1/10000 sec
IRIS_MAX = 13  # 14 steps, F1.6 to Close
EXPOSURE_COMP_RANGE = 7  # +/-7 steps of 1.5dB = +/-10.5dB


def ae_mode(mode: str) -> bytes:
    try:
        return AE_MODES[mode]
    except KeyError:
        raise ValueError(f"unknown AE mode: {mode!r}") from None


def shutter_direct(value: int) -> bytes:
    value = max(0, min(SHUTTER_MAX, value))
    return bytes([ADDR, 0x01, 0x04, 0x4A, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


def iris_direct(value: int) -> bytes:
    value = max(0, min(IRIS_MAX, value))
    return bytes([ADDR, 0x01, 0x04, 0x4B, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


def gain_direct(value: int) -> bytes:
    value = max(0, min(0xFF, value))
    return bytes([ADDR, 0x01, 0x04, 0x4C, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


def exposure_comp_direct(value: int) -> bytes:
    """value is signed, +/-EXPOSURE_COMP_RANGE steps of 1.5dB."""
    value = max(-EXPOSURE_COMP_RANGE, min(EXPOSURE_COMP_RANGE, value))
    return bytes([
        ADDR, 0x01, 0x04, 0x4E, 0x00, 0x00,
        *_pack_nibbles(value + EXPOSURE_COMP_RANGE, 2), 0xFF,
    ])


# ---------------------------------------------------------------------------
# White balance
# ---------------------------------------------------------------------------

WB_MODES = {
    "auto": 0x00,
    "indoor": 0x01,
    "outdoor": 0x02,
    "onepush": 0x03,
    "atw": 0x04,
    "manual": 0x05,
}


def wb_mode(mode: str) -> bytes:
    try:
        sub = WB_MODES[mode]
    except KeyError:
        raise ValueError(f"unknown white balance mode: {mode!r}") from None
    return bytes([ADDR, 0x01, 0x04, 0x35, sub, 0xFF])


def wb_red_gain(value: int) -> bytes:
    value = max(0, min(0xFF, value))
    return bytes([ADDR, 0x01, 0x04, 0x43, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


def wb_blue_gain(value: int) -> bytes:
    value = max(0, min(0xFF, value))
    return bytes([ADDR, 0x01, 0x04, 0x44, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


# ---------------------------------------------------------------------------
# IR / Day-Night (ICR)
# ---------------------------------------------------------------------------

_ICR_STATES = {"bw": 0x02, "color": 0x04, "off": 0x03}


def icr(state: str) -> bytes:
    """Manual IR-cut filter control: 'bw' (IR cut removed), 'color', or 'off'."""
    try:
        sub = _ICR_STATES[state]
    except KeyError:
        raise ValueError(f"unknown ICR state: {state!r}") from None
    return bytes([ADDR, 0x01, 0x04, 0x01, sub, 0xFF])


def auto_icr(state: str) -> bytes:
    """Automatic day/night switching: 'bw', 'color', or 'off'."""
    try:
        sub = _ICR_STATES[state]
    except KeyError:
        raise ValueError(f"unknown auto ICR state: {state!r}") from None
    return bytes([ADDR, 0x01, 0x04, 0x51, sub, 0xFF])


def auto_icr_threshold(value: int) -> bytes:
    value = max(0, min(0xFF, value))
    return bytes([ADDR, 0x01, 0x04, 0x21, 0x00, 0x00, *_pack_nibbles(value, 2), 0xFF])


# ---------------------------------------------------------------------------
# Image stabilizer
# ---------------------------------------------------------------------------

IS_ON = bytes([ADDR, 0x01, 0x04, 0x34, 0x02, 0xFF])
IS_OFF = bytes([ADDR, 0x01, 0x04, 0x34, 0x03, 0xFF])


# ---------------------------------------------------------------------------
# Dynamic range / low-light
# ---------------------------------------------------------------------------

WDR_ON = bytes([ADDR, 0x01, 0x04, 0x3D, 0x02, 0xFF])
WDR_OFF = bytes([ADDR, 0x01, 0x04, 0x3D, 0x03, 0xFF])

DEFOG_OFF = bytes([ADDR, 0x01, 0x04, 0x37, 0x03, 0xFF])
DEFOG_LEVELS = {1: "low", 2: "mid", 3: "high"}


def defog(level: int) -> bytes:
    """0 turns defog off; 1-3 select low/mid/high."""
    if level == 0:
        return DEFOG_OFF
    if level not in DEFOG_LEVELS:
        raise ValueError(f"unknown defog level: {level!r}")
    return bytes([ADDR, 0x01, 0x04, 0x37, 0x02, level, 0xFF])


BACKLIGHT_COMPENSATION_ON = bytes([ADDR, 0x01, 0x04, 0x33, 0x02, 0xFF])
BACKLIGHT_COMPENSATION_OFF = bytes([ADDR, 0x01, 0x04, 0x33, 0x03, 0xFF])

HLC_ON = bytes([ADDR, 0x01, 0x04, 0x14, 0x02, 0xFF])
HLC_OFF = bytes([ADDR, 0x01, 0x04, 0x14, 0x03, 0xFF])

HIGH_SENSITIVITY_ON = bytes([ADDR, 0x01, 0x04, 0x5E, 0x02, 0xFF])
HIGH_SENSITIVITY_OFF = bytes([ADDR, 0x01, 0x04, 0x5E, 0x03, 0xFF])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _pack_nibbles(value: int, count: int) -> list:
    """Split value into `count` big-endian nibbles, e.g. (0x1234, 4) -> [1,2,3,4]."""
    return [(value >> (4 * (count - 1 - i))) & 0x0F for i in range(count)]


def _parse_position_reply(reply: bytes) -> int:
    """Parse a yy 50 0p 0q 0r 0s FF style position inquiry reply into 0-0xFFFF."""
    payload = _extract_payload(reply, expected_len=7)
    if payload is None or payload[0] != 0x50:
        raise ValueError(f"unexpected position inquiry reply: {reply.hex()}")
    p, q, r, s = payload[1:5]
    return (p << 12) | (q << 8) | (r << 4) | s


def _extract_payload(reply: bytes, expected_len: int):
    """Strip leading ACK frame(s) and the terminating 0xFF, keep the completion payload."""
    frames = []
    frame = []
    for b in reply:
        frame.append(b)
        if b == 0xFF:
            frames.append(bytes(frame))
            frame = []
    for f in frames:
        if len(f) == expected_len and (f[0] & 0xF0) == 0x90 and (f[1] & 0xF0) == 0x50:
            return f[1:]
    return None