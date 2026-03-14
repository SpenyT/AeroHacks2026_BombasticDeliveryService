from dataclasses import dataclass
import asyncio
import math, time, contextlib, sys, tty
import termios
from contextlib import suppress
import drone_driver



def log(msg: str):
    sys.stdout.write("\r" + msg + "\n")
    sys.stdout.flush()

@dataclass
class PositionEstimate:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    valid: bool = True


class DroneController: 
    def __init__(self ) -> None:
        self.enabled = asyncio.Event()
        self.shutdown = asyncio.Event()
        self.baseline_thrust = 120

    def send_pid_commands(self, roll, pitch, yaw_rate, thrust):
        drone_driver.set_roll(roll)
        drone_driver.set_pitch(pitch)
        drone_driver.set_yaw(yaw_rate)
        self.baseline_thrust = thrust
        self.send_baseline_thrust()

    def send_position_target(self, x, y, z, yaw): ...
    def send_external_pose(self, pose): ...
    def set_param(self, name, value): ...
    def subscribe_log(self, variables, period_ms): ...

    def is_running(self) -> bool:
        return self.enabled.is_set() and not self.shutdown.is_set()

    def start(self) -> None:
        drone_driver.set_mode(drone_driver.Modes.Pid)
        time.sleep(0.1)
        assert drone_driver.get_mode() == drone_driver.Modes.Pid, "Failed to set drone mode"

        # TODO move somewhere else, this only needs to be done once
        self.turn_on_leds()

        self.enabled.set()
        self.send_baseline_thrust()

    def stop(self) -> None:
        self.emergency_stop()
        self.enabled.clear()

    def request_shutdown(self) -> None:
        self.close_connection()
        self.shutdown.set()

    def turn_on_leds(self):
        drone_driver.blue_LED_on()
        drone_driver.red_LED_on()
        drone_driver.green_LED_on()

    def send_baseline_thrust(self) -> None:
        baseline_thrust = drone_driver.Thrust(A=self.baseline_thrust, B=self.baseline_thrust, C=self.baseline_thrust, D=self.baseline_thrust)
        drone_driver.manual_thrusts(baseline_thrust)

    def increase_thrust(self, delta: int) -> None:
        self.baseline_thrust += delta
        self.send_baseline_thrust()

    def emergency_stop(self) -> None:
        drone_driver.emergency_stop()

    def close_connection(self) -> None:
        drone_driver.close()
    


        
# async def keyboard_task(controller: DroneController) -> None:
#     """
#     Reads single key presses asynchronously.

#     Keys:
#       w -> forward
#       s -> backward
#       d -> right
#       a -> left
#       j -> clockwize yaw
#       k -> counterclockwize yaw
#       i -> up
#       o -> down

#       1 -> start
#       2 -> stop
#       3 -> status
#       q -> quit
#       h -> help
#     """

#     log("Keyboard commands: s=start | x=stop | d=status | q=quit | h=help")

#     loop = asyncio.get_running_loop()

#     # Save terminal settings
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)

#     try:
#         # Set raw mode so keypresses are delivered immediately
#         tty.setraw(fd)

#         while not controller.shutdown.is_set():
#             char = await loop.run_in_executor(None, sys.stdin.read, 1)
#             cmd = char.lower()

#             match cmd:
#                 case "w":
#                     log("[kbd] forward")
#                     controller.send_pid_commands(roll=0, pitch=-10, yaw_rate=0, thrust=0)

#                 case "s"

#             if cmd == "s":
#                 controller.start()
#                 log("[kbd] control ENABLED")

#             elif cmd == "x":
#                 controller.stop()
#                 log("[kbd] control DISABLED")

#             elif cmd == "d":
#                 log(
#                     f"[kbd] running={controller.is_running()} "
#                     f"shutdown={controller.shutdown.is_set()}"
#                 )

#             elif cmd == "q":
#                 log("[kbd] shutting down")
#                 controller.request_shutdown()

#             elif cmd == "h":
#                 log("Keys: s=start | x=stop | d=status | q=quit | h=help")

#             else:
#                 pass

#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

async def keyboard_task(controller: DroneController) -> None:
    """
    Reads single key presses asynchronously.

    Keys
    ----
    w : forward
    s : backward
    d : right
    a : left
    j : clockwise yaw
    k : counter-clockwise yaw
    i : up
    o : down

    1 : start controller
    2 : stop controller
    3 : status
    q : quit
    h : help
    """

    log("Keyboard commands: w/s/a/d move | j/k yaw | i/o thrust | 1 start | 2 stop | 3 status | q quit")

    loop = asyncio.get_running_loop()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)

        while not controller.shutdown.is_set():
            char = await loop.run_in_executor(None, sys.stdin.read, 1)
            cmd = char.lower()

            match cmd:

                # movement
                case "w":
                    log("[kbd] forward")
                    controller.send_pid_commands(roll=0, pitch=-10, yaw_rate=0, thrust=0)

                case "s":
                    log("[kbd] backward")
                    controller.send_pid_commands(roll=0, pitch=10, yaw_rate=0, thrust=0)

                case "a":
                    log("[kbd] left")
                    controller.send_pid_commands(roll=-10, pitch=0, yaw_rate=0, thrust=0)

                case "d":
                    log("[kbd] right")
                    controller.send_pid_commands(roll=10, pitch=0, yaw_rate=0, thrust=0)

                # yaw
                case "j":
                    log("[kbd] yaw clockwise")
                    controller.send_pid_commands(roll=0, pitch=0, yaw_rate=10, thrust=0)

                case "k":
                    log("[kbd] yaw counter-clockwise")
                    controller.send_pid_commands(roll=0, pitch=0, yaw_rate=-10, thrust=0)

                # altitude
                case "i":
                    log("[kbd] up")
                    controller.increase_thrust(10)

                case "o":
                    log("[kbd] down")
                    controller.increase_thrust(-10)

                # controller state
                case "1":
                    controller.start()
                    log("[kbd] control ENABLED")

                case "2":
                    controller.stop()
                    log("[kbd] control DISABLED")

                case "3":
                    log(
                        f"[kbd] running={controller.is_running()} "
                        f"shutdown={controller.shutdown.is_set()}"
                    )

                case "q":
                    log("[kbd] shutting down")
                    controller.request_shutdown()

                case "h":
                    log(
                        "Keys: w/s/a/d move | j/k yaw | i/o thrust | "
                        "1 start | 2 stop | 3 status | q quit"
                    )

                case _:
                    pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


async def position_consume_task(state: DroneController, position_queue: asyncio.Queue[PositionEstimate] ) -> None:
    """
    Waits for a PositionEstimate from the OpenCV pipeline and sends it to the drone as an external pose.
    """

    latest_position: PositionEstimate | None = None

    while not state.shutdown.is_set():
        try:
            pose = await asyncio.wait_for(position_queue.get(), timeout=0.2)
            latest_position = pose
        except asyncio.TimeoutError:
            continue

        if not pose.valid:
            log("[ctrl] received invalid pose, skipping")
            continue

        if not state.is_running():
            log("[ctrl] received pose but control is disabled, skipping")
            continue

        # Target posion 
        target_x, target_y, target_z, target_yaw = 0.0, 0.0, 1.0, 0.0

        err_x = target_x - pose.x
        err_y = target_y - pose.y
        err_z = target_z = pose.z
        err_yaw = target_yaw - pose.yaw

        log(f"[ctrl] pose t={pose.t:.2f} x={pose.x:.2f} y={pose.y:.2f} z={pose.z:.2f} yaw={pose.yaw:.1f}")

        # TODO change this shit
        kx = 10
        ky = 10
        kz = 10
        kyaw = 10
        base = 10

        roll_cmd = kx * err_x
        pitch_imd_cmd = ky * err_y
        thrust_cmd = base + kz * err_z
        yaw_rate_cmd = kyaw * err_yaw

        state.send_pid_commands(roll=roll_cmd, pitch=pitch_imd_cmd, yaw_rate=yaw_rate_cmd, thrust=thrust_cmd)
        


async def fake_pose_producer_task(
    pose_queue: asyncio.Queue[PositionEstimate],
    state: DroneController,
) -> None:
    """
    Demo producer that simulates another task.
    """

    t0 = time.monotonic()

    while not state.shutdown.is_set():
        t = time.monotonic() - t0

        pose = PositionEstimate(
            t=t,
            x=0.2 * math.sin(t),
            y=0.2 * math.cos(t),
            z=1.0 + 0.05 * math.sin(2 * t),
            yaw=15.0 * math.sin(0.5 * t),
            valid=True,
        )

        if pose_queue.full():
            with contextlib.suppress(asyncio.QueueEmpty):
                _ = pose_queue.get_nowait()
                pose_queue.task_done()

        await pose_queue.put(pose)
        await asyncio.sleep(0.1)

    log("[vision] producer exiting")


async def main() -> None:
    state = DroneController()

    pose_queue: asyncio.Queue[PositionEstimate] = asyncio.Queue(maxsize=10)

    tasks = [
        asyncio.create_task(keyboard_task(state), name="Keyboard"),
        asyncio.create_task(position_consume_task(state, pose_queue ), name="PositionConsumer"),
        asyncio.create_task(fake_pose_producer_task(pose_queue, state), name="FakePoseProducer"),
    ]

    try:
        await state.shutdown.wait()
    finally:
        state.request_shutdown()

        for task in tasks:
            task.cancel()

        for task in tasks:
            with contextlib.suppress(asyncio.CancelledError):
                await task


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("Interrupted by user, exiting...")

