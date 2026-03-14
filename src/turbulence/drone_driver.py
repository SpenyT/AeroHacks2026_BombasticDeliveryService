"""
The following functions can be used to communicate with the drone

general advice:
do not have constant high-bandwidth communications with the drone,
because processing time doing wifi stuff is processing time not spent updating the gyroscope,
which will lead to increased drift
"""


from enum import IntEnum
import socket
import threading
import time
from dataclasses import dataclass

DRONE_IP = "192.168.4.1"
DRONE_PORT = 8080

MAX_THRUST = 250
MIN_THRUST = 0

SOCKET_TIMEOUT = 1.0

_lock = threading.Lock()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(SOCKET_TIMEOUT)
s.connect((DRONE_IP, DRONE_PORT))


def _clamp(v, lo=MIN_THRUST, hi=MAX_THRUST):
    """
    Clamp a value to the allowed thrust range.

    Motor thrust values must always remain between ``0`` and ``250``.

    :param v: value to clamp
    :param lo: minimum allowed value
    :param hi: maximum allowed value
    :return: clamped integer value
    """
    return max(lo, min(hi, int(v)))


def msg(tx: str) -> str:
    """
    Send a command to the drone and wait for a response.

    The drone communication protocol is line-based ASCII. Every command must
    end with a newline and responses are terminated by ``\\n``.

    This function is thread-safe and protected by a socket timeout.

    :param tx: command string to send
    :return: response string (without trailing newline)

    Example::

        >>> msg("gMode")
        '2'
    """
    with _lock:
        try:
            s.sendall((tx + "\n").encode("ASCII"))

            data = b""
            while not data.endswith(b"\n"):
                chunk = s.recv(64) # TODO if it doest fit in 64 bytes, switch to 1 byte recv to avoid splitting the response
                if not chunk:
                    raise ConnectionError("Drone closed connection")
                data += chunk

            # handle multiple messages in the buffer by only returning the
            # first one and leaving the rest for the next call
            if b"\n" in data:
                line, _ = data.split(b"\n", 1)
                return line.decode("ascii")

            return data[:-1].decode("ASCII")

        except Exception as e:
            try:
                emergency_stop()
            except Exception:
                pass
            raise RuntimeError(f"Drone communication failure: {e}")


def emergency_stop():
    """
    Immediately disable the motors.

    This sends ``mode0`` to the drone, which shuts off all motor output.
    """
    try:
        s.sendall(b"mode0\n")
    except Exception:
        pass


def e():
    """
    Alias for :func:`emergency_stop`.
    """
    emergency_stop()


def close():
    """
    Safely terminate the connection.

    The drone motors are stopped before closing the socket.
    """
    emergency_stop()
    try:
        s.close()
    except Exception:
        pass


class Modes(IntEnum):
    """
    Drone operating modes.

    ``Off``
        All motors disabled.

    ``Manual``
        Direct motor control using :func:`manual_thrusts`.

    ``Pid``
        Stabilized mode where pitch and roll are controlled by onboard
        PID loops.
    """

    Off = 0
    Manual = 1
    Pid = 2


@dataclass
class Thrust:
    """
    Data class representing the thrust values for the four motors.
    """
    A: int
    B: int
    C: int
    D: int

def set_mode(m: Modes):
    """
    Change the drone operating mode.

    :param m: desired mode

    Example::

        set_mode(Modes.Pid)
    """
    if m not in (0, 1, 2):
        raise ValueError("Mode must be 0, 1 or 2")
    msg(f"mode{m}")


def get_mode():
    """
    Query the current drone mode.

    :return: :class:`Modes` value

    Example::

        >>> get_mode()
        Modes.Pid
    """
    return Modes[msg("gMode")]


def manual_thrusts(thrust: Thrust):
    """
    Set the thrust of each motor.

    Thrust values are integers in the range ``0``–``250``.

    The mapping of motors (A,B,C,D) depends on the drone hardware layout.

    :param thrust: thrust for all the motors

    In ``Modes.Pid`` this sets the **baseline thrust**, while the PID controller
    adds corrections for stabilization.
    """
    A = _clamp(thrust.A)
    B = _clamp(thrust.B)
    C = _clamp(thrust.C)
    D = _clamp(thrust.D)

    msg(f"manT\n{A},{B},{C},{D}")


def get_pitch():
    """
    Read the current pitch angle.

    :return: pitch angle in approximate degrees
    """
    return float(msg("angX")) / 16


def get_roll():
    """
    Read the current roll angle.

    :return: roll angle in approximate degrees
    """
    return float(msg("angY")) / 16


def get_gyro_pitch():
    """
    Get pitch rotation rate.

    :return: pitch angular velocity in degrees per second
    """
    return float(msg("gyroX"))


def get_gyro_roll():
    """
    Get roll rotation rate.

    :return: roll angular velocity in degrees per second
    """
    return float(msg("gyroY"))


def set_pitch(r):
    """
    Set target pitch angle.

    Used in :class:`Modes.Pid` to control forward/backward tilt.

    :param r: target pitch angle
    """
    msg(f"gx{r}")


def set_roll(r):
    """
    Set target roll angle.

    Used in :class:`Modes.Pid` to control left/right tilt.

    :param r: target roll angle
    """
    msg(f"gy{r}")


def set_yaw(y):
    """
    Apply yaw correction.

    This directly adjusts the torque difference between opposing motors,
    causing the drone to rotate around the vertical axis.

    :param y: yaw correction value
    """
    msg(f"yaw{y}")


def set_p_gain(p):
    """
    Set proportional gain of the onboard PID controller.

    Typical values are approximately ``0``–``0.5``.
    """
    msg(f"gainP{p}")


def set_i_gain(i):
    """
    Set integral gain of the onboard PID controller.

    Typical values are very small (below ``0.00003``).
    """
    msg(f"gainI{i}")


def set_d_gain(d):
    """
    Set derivative gain of the onboard PID controller.

    Typical values are approximately ``0``–``10``.
    """
    msg(f"gainD{d}")


def reset_integral():
    """
    Reset the integral terms of the PID controller.

    This is useful when the drone has accumulated integral error,
    for example after a disturbance or during testing.
    """
    msg("irst")


def get_i_values():
    """
    Retrieve the current integral values from the PID controller.

    :return: list ``[I_pitch, I_roll]``

    Example::

        >>> get_i_values()
        [0.0012, -0.0004]
    """
    resp = msg("geti").split(",")
    return [float(resp[0]), float(resp[1])]
