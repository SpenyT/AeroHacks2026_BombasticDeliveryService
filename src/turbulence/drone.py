from dataclasses import dataclass
import asyncio
import math, time, contextlib, sys, tty
import termios
from contextlib import suppress



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

class DroneLink:
    def connect(self): ...
    def send_attitude_thrust(self, roll, pitch, yaw_rate, thrust): ...
    def send_position_target(self, x, y, z, yaw): ...
    def send_external_pose(self, pose): ...
    def set_param(self, name, value): ...
    def subscribe_log(self, variables, period_ms): ...



class DroneState: 
    def __init__(self) -> None:
        self.enabled = asyncio.Event()
        self.shutdown = asyncio.Event()

    def is_running(self) -> bool:
        return self.enabled.is_set() and not self.shutdown.is_set()

    def start(self) -> None:
        self.enabled.set()

    def stop(self) -> None:
        self.enabled.clear()

    def request_shutdown(self) -> None:
        self.shutdown.set()



        
async def keyboard_task(state: DroneState) -> None:
    """
    Reads single key presses asynchronously.

    Keys:
      s -> start
      x -> stop
      d -> status
      q -> quit
      h -> help
    """

    log("Keyboard commands: s=start | x=stop | d=status | q=quit | h=help")

    loop = asyncio.get_running_loop()

    # Save terminal settings
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        # Set raw mode so keypresses are delivered immediately
        tty.setraw(fd)

        while not state.shutdown.is_set():
            char = await loop.run_in_executor(None, sys.stdin.read, 1)
            cmd = char.lower()

            if cmd == "s":
                state.start()
                log("[kbd] control ENABLED")

            elif cmd == "x":
                state.stop()
                log("[kbd] control DISABLED")

            elif cmd == "d":
                log(
                    f"[kbd] running={state.is_running()} "
                    f"shutdown={state.shutdown.is_set()}"
                )

            elif cmd == "q":
                log("[kbd] shutting down")
                state.request_shutdown()

            elif cmd == "h":
                log("Keys: s=start | x=stop | d=status | q=quit | h=help")

            else:
                pass

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)



async def position_consume_task(state: DroneState, position_queue: asyncio.Queue[PositionEstimate], link: DroneLink) -> None:
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

        link.send_position_target(err_x, err_y, err_z, err_yaw)
        


async def fake_pose_producer_task(
    pose_queue: asyncio.Queue[PositionEstimate],
    state: DroneState,
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
    state = DroneState()

    pose_queue: asyncio.Queue[PositionEstimate] = asyncio.Queue(maxsize=10)

    tasks = [
        asyncio.create_task(keyboard_task(state), name="Keyboard"),
        asyncio.create_task(position_consume_task(state, pose_queue, DroneLink()), name="PositionConsumer"),
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

