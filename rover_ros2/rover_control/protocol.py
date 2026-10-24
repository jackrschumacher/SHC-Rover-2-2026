# Shared command and sensor messages.
AXES = ("Axis1", "Axis2", "Axis3A", "Axis3B")
CONTROL_COMMANDS = ("ARM_ENABLE", "STOP_ALL", "HEARTBEAT", "STATUS", "LINK_LOST")
SENSOR_FIELDS = ("temperature", "pressure", "accel_x", "accel_y", "accel_z", "altitude")

def is_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and value == value and value not in (float("inf"), float("-inf")))

def validate_command(request):
    if isinstance(request, str):
        request = {"command": request}
    if not isinstance(request, dict):
        raise ValueError("Command must be an object")
    if set(request) == {"command"}:
        if request["command"] not in CONTROL_COMMANDS:
            raise ValueError("Unknown control command")
        return dict(request)
    if set(request) == {"motor", "speed"}:
        if request["motor"] != "DC" or not is_number(request["speed"]):
            raise ValueError("Motor command needs DC and a finite speed")
        if not -100 <= request["speed"] <= 100:
            raise ValueError("Motor speed must be between -100 and 100 percent")
        return {"motor": "DC", "speed": request["speed"]}
    if set(request) != {"axis", "angle"}:
        raise ValueError("Servo command needs axis and angle")
    if not isinstance(request["axis"], str):
        raise ValueError("Axis must be a name")
    axis = next((name for name in AXES if name.casefold() == request["axis"].casefold()), None)
    if axis not in AXES:
        raise ValueError("Unknown axis: " + axis)
    if not is_number(request["angle"]):
        raise ValueError("Angle must be a finite number")
    return {"axis": axis, "angle": request["angle"]}

# Pico serial lines are CSV; see pico/rover_pico/rover_pico.ino for the format.
SERIAL_CODES = {"HEARTBEAT": "H", "ARM_ENABLE": "E", "STOP_ALL": "X", "LINK_LOST": "X", "STATUS": "S"}


def encode_command(request):
    request = validate_command(request)
    if "command" in request:
        line = SERIAL_CODES[request["command"]]
    elif "motor" in request:
        line = "M,%g" % request["speed"]
    else:
        line = "A,%d,%g" % (AXES.index(request["axis"]), request["angle"])
    return (line + "\n").encode("ascii")


def _number(text, allow_empty=False):
    if text == "" and allow_empty:
        return None
    value = float(text)
    if not is_number(value):
        raise ValueError("Non-finite value")
    return value


def _flag(text):
    if text not in ("0", "1"):
        raise ValueError("Invalid flag: " + text)
    return text == "1"


def parse_line(line):
    """Return ("sensors" | "status", dict) for one Pico line, or raise ValueError."""
    fields = line.decode("ascii").strip().split(",")
    if fields[0] == "D" and len(fields) == 2 + len(SENSOR_FIELDS):
        values = [_number(text, allow_empty=True) for text in fields[2:]]
        if all(value is None for value in values):
            raise ValueError("Sensor line has no readings")
        return "sensors", {"type": "sensors", "mode": "hardware", "elapsed_ms": int(fields[1]),
                           "data": dict(zip(SENSOR_FIELDS, values))}
    joint_end = 9 + 3 * len(AXES)
    if fields[0] == "T" and len(fields) > joint_end:
        joints = [_number(text) for text in fields[9:joint_end]]
        sd_rows = int(fields[8])
        if sd_rows < 0:
            raise ValueError("Invalid SD row count")
        return "status", {
            "type": "status",
            "mode": "simulation" if _flag(fields[1]) else "hardware",
            "ready": True,
            "enabled": _flag(fields[2]),
            "motor_ready": True,
            "motor_speed": _number(fields[3]),
            "motor_limit": _number(fields[4]),
            "imu_ready": _flag(fields[5]),
            "bmp_ready": _flag(fields[6]),
            "sd_ready": _flag(fields[7]),
            "sd_rows": sd_rows,
            "targets": {axis: joints[3 * i] for i, axis in enumerate(AXES)},
            "limits": {axis: joints[3 * i + 1:3 * i + 3] for i, axis in enumerate(AXES)},
            "note": ",".join(fields[joint_end:]),
        }
    raise ValueError("Unrecognised Pico line")
