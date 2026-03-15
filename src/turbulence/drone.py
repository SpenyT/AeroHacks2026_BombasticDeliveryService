from dataclasses import dataclass
import asyncio
import math, time, contextlib, sys, tty
import termios
from contextlib import suppress
from typing import Optional

import drone_driver
from drone_driver import log
import os

@dataclass
class PositionEstimate:
    t: float
    x: float
    y: float
    z: float
    yaw: float
    valid: bool = True

@dataclass()
class DroneState:
    roll: float
    pitch: float
    yaw: float
    position: Optional[ PositionEstimate ] = None

class DroneController: 
    def __init__(self ) -> None:
        self.enabled = asyncio.Event()
        self.shutdown = asyncio.Event()
        self.manual_override = False 
        self.baseline_thrust = 120
        self.state = DroneState(pitch=0, roll=0, yaw=0)

    def send_pid_commands(self, roll, pitch, yaw_rate):
        drone_driver.set_roll(roll)
        drone_driver.set_pitch(pitch)
        drone_driver.set_yaw(yaw_rate)

    def is_running(self) -> bool:
        return self.enabled.is_set() and not self.shutdown.is_set()

    async def start(self) -> None:
        self.enabled.set()
        drone_driver.set_mode(drone_driver.Modes.Pid)
        await asyncio.sleep(1)
        mode = drone_driver.get_mode()
        log(f"mode: {mode}")

        if drone_driver.get_mode() != drone_driver.Modes.Pid:
            raise RuntimeError(f"wrong mode returned {drone_driver.get_mode()}")

        # TODO move somewhere else, this only needs to be done once
        self.turn_on_leds()

        drone_driver.reset_integral()
        
        pitch = drone_driver.get_pitch()
        roll = drone_driver.get_roll()

        log(f"pitch start: {pitch}")
        log(f"roll start: {roll}")

        self.state.pitch = pitch
        self.state.roll = roll

        drone_driver.set_pitch(-pitch)
        drone_driver.set_roll(-roll)

        drone_driver.set_yaw(0)

        drone_driver.reset_integral()

        drone_driver.set_p_gain(0.15)
        drone_driver.set_i_gain(0.00002)
        drone_driver.set_d_gain(5)

        self.send_baseline_thrust()

    def stop(self) -> None:
        self.emergency_stop()
        self.enabled.clear()

    def request_shutdown(self) -> None:
        self.close_connection()
        self.shutdown.set()

    def turn_on_leds(self):
        drone_driver.blue_LED_on()
        # drone_driver.red_LED_on()
        drone_driver.green_LED_on()

    def send_baseline_thrust(self) -> None:
        baseline_thrust = drone_driver.Thrust(A=self.baseline_thrust, B=self.baseline_thrust, C=self.baseline_thrust, D=self.baseline_thrust)
        drone_driver.manual_thrusts(baseline_thrust)

    def increase_thrust(self, delta: int) -> None:
        self.baseline_thrust = drone_driver._clamp(self.baseline_thrust + delta)
        self.send_baseline_thrust()

    def emergency_stop(self) -> None:
        drone_driver.emergency_stop()

    def close_connection(self) -> None:
        drone_driver.close()
    
    def log_status(self):
        pitch = drone_driver.get_pitch()
        roll = drone_driver.get_roll()

        gyro_pitch = drone_driver.get_gyro_pitch()
        gyro_roll = drone_driver.get_gyro_roll()

        log(f"pitch={pitch} roll={roll} gyro_pitch={gyro_pitch} gyro_roll={gyro_roll}")



async def keyboard_task(controller: DroneController) -> None:
    """
    Reads single key presses asynchronously and auto-stops the drone when no key is held.

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

    log("\n [kbd] Keyboard commands: w/s/a/d move | j/k yaw | i/o thrust | 1 start | 2 stop | 3 status | q quit")

    loop = asyncio.get_running_loop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    os.set_blocking(fd, False)

    def on_input():
        try:
            char = os.read(fd, 1).decode('utf-8')
            if char:
                queue.put_nowait(char.lower())
        except BlockingIOError:
            pass


    queue = asyncio.Queue()

    try:
        # setcbreak so keys are read immediately without requiring Enter
        tty.setraw(fd)
        loop.add_reader(fd, on_input)

        is_moving = False

        while not controller.shutdown.is_set():
            try:
                # Wait for key press. If we don't get one within 0.2s, timeout triggers hover reset
                cmd = await asyncio.wait_for(queue.get(), timeout=0.3)
                
                match cmd:
                    # movement
                    case "w":
                        log("\n[kbd] forward")
                        controller.manual_override = True
                        pitch = controller.state.pitch
                        roll = controller.state.roll

                        controller.send_pid_commands(roll=0, pitch=-10, yaw_rate=0)
                        is_moving = True

                    case "s":
                        log("\n[kbd] backward")
                        controller.manual_override = True
                        controller.send_pid_commands(roll=0, pitch=10, yaw_rate=0)
                        is_moving = True

                    case "a":
                        log("\n[kbd] left")
                        controller.manual_override = True
                        controller.send_pid_commands(roll=-10, pitch=0, yaw_rate=0)
                        is_moving = True

                    case "d":
                        log("\n[kbd] right")
                        controller.manual_override = True
                        controller.send_pid_commands(roll=10, pitch=0, yaw_rate=0)
                        is_moving = True

                    case "j":
                        log("\n[kbd] yaw clockwise")
                        controller.manual_override = True
                        controller.send_pid_commands(roll=0, pitch=0, yaw_rate=10)
                        is_moving = True

                    case "k":
                        log("\n[kbd] yaw counter-clockwise")
                        controller.manual_override = True
                        controller.send_pid_commands(roll=0, pitch=0, yaw_rate=-10)
                        is_moving = True

                    # altitude
                    case "i":
                        log("\n[kbd] up")
                        controller.manual_override = True
                        controller.increase_thrust(10)
                        is_moving = True

                    case "o":
                        log("\n[kbd] down")
                        controller.manual_override = True
                        controller.increase_thrust(-10)
                        is_moving = True

                    # controller state
                    case "1":
                        await controller.start()
                        log("\n[kbd] control ENABLED")

                    case "2":
                        controller.stop()
                        log("\n[kbd] control DISABLED")

                    case "3":
                        # controller.log_status()
                        log(
                            f"[kbd] running={controller.is_running()} "
                            f"shutdown={controller.shutdown.is_set()}"
                        )

                    case "q":
                        log("\n[kbd] shutting down")
                        controller.request_shutdown()

                    case "h":
                        log(
                            "Keys: w/s/a/d move | j/k yaw | i/o thrust | "
                            "1 start | 2 stop | 3 status | q quit"
                        )

                    case _:
                        pass

            except asyncio.TimeoutError:
                # If no key was pressed and we previously sent movement commands, stop moving.
                if is_moving and controller.is_running():
                    controller.manual_override = False
                    # controller.send_pid_commands(roll=0, pitch=0, yaw_rate=0)
                    is_moving = False

    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


async def position_consume_task(state: DroneController, position_queue: asyncio.Queue[PositionEstimate] ) -> None:
    """
    Waits for a PositionEstimate from the OpenCV pipeline and sends it to the drone as an external pose.
    """

    latest_position: PositionEstimate | None = None

    while not state.shutdown.is_set():

        if state.manual_override:
            await asyncio.sleep(0.1)
            continue  
        try:
            pose = await asyncio.wait_for(position_queue.get(), timeout=0.2)
            latest_position = pose
        except asyncio.TimeoutError:
            continue

        if not pose.valid:
            log("\n[ctrl] received invalid pose, skipping")
            continue

        if not state.is_running():
            log("\n[ctrl] received pose but control is disabled, skipping")
            continue

        # Target posion 
        target_x, target_y, target_z, target_yaw = 0.0, 0.0, 1.0, 0.0

        err_x = target_x - pose.x
        err_y = target_y - pose.y
        err_z = target_z - pose.z
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
        # thrust_cmd = base + kz * err_z
        yaw_rate_cmd = kyaw * err_yaw

        state.send_pid_commands(roll=roll_cmd, pitch=pitch_imd_cmd, yaw_rate=yaw_rate_cmd)
        


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
        await asyncio.sleep(0.05)

    log("\n[vision] producer exiting")


async def main() -> None:
    state = DroneController()

    log("---- IMU status ----")
    log(f"pitch={drone_driver.get_pitch()}")
    log(f"roll={drone_driver.get_roll()}")
    log(f"gyro_pitch={drone_driver.get_gyro_pitch()}")
    log(f"gyro_roll={drone_driver.get_gyro_roll()}")

    pose_queue: asyncio.Queue[PositionEstimate] = asyncio.Queue(maxsize=10)

    tasks = [
        asyncio.create_task(keyboard_task(state), name="Keyboard"),
        asyncio.create_task(position_consume_task(state, pose_queue ), name="PositionConsumer"),
        # asyncio.create_task(fake_pose_producer_task(pose_queue, state), name="FakePoseProducer"),
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
        log("\nInterrupted by user, exiting...")

