import asyncio
import sys
import tty
import termios
import os
import time

import drone_driver
from drone_driver import log


class AutoHoverController:
    def __init__(self):
        self.enabled = asyncio.Event()
        self.shutdown = asyncio.Event()
        self.base_thrust = 120
        # Simple P-D gains (need tuning)
        self.kp_roll = 1.0
        self.kp_pitch = 1.0
        self.kd_roll = 0.5
        self.kd_pitch = 0.5

    async def start(self):
        self.enabled.set()
        drone_driver.set_mode(drone_driver.Modes.Manual)
        await asyncio.sleep(0.5)
        log(f"[hover] mode: {drone_driver.get_mode()}")
        log(f"[hover] baseline_thrust: {self.base_thrust}")

    def stop(self):
        self.enabled.clear()
        drone_driver.emergency_stop()

    def request_shutdown(self):
        self.stop()
        drone_driver.close()
        self.shutdown.set()

    def increase_thrust(self, delta: int):
        self.base_thrust = drone_driver._clamp(self.base_thrust + delta)
        log(f"[hover] base_thrust = {self.base_thrust}")


async def hover_task(controller: AutoHoverController):
    """
    Feedback loop for auto-hover using pure manual torque specification
    """
    while not controller.shutdown.is_set():
        if controller.enabled.is_set():
            # Get current angles and rotation rates
            roll = drone_driver.get_roll()
            await asyncio.sleep(0.005)
            pitch = drone_driver.get_pitch()
            await asyncio.sleep(0.005)
            gyro_roll = drone_driver.get_gyro_roll()
            await asyncio.sleep(0.005)
            gyro_pitch = drone_driver.get_gyro_pitch()
            await asyncio.sleep(0.005)

            # PD control mixing
            # Positive roll = tilted right -> thrust right side (B,D)
            # Positive pitch = tilted back -> thrust back side (C,D)
            adj_roll = controller.kp_roll * roll + controller.kd_roll * gyro_roll
            adj_pitch = controller.kp_pitch * pitch + controller.kd_pitch * gyro_pitch
            
            thrust_A = controller.base_thrust - adj_roll - adj_pitch
            thrust_B = controller.base_thrust + adj_roll - adj_pitch
            thrust_C = controller.base_thrust - adj_roll + adj_pitch
            thrust_D = controller.base_thrust + adj_roll + adj_pitch
            
            t_data = drone_driver.Thrust(
                A=int(thrust_A),
                B=int(thrust_B),
                C=int(thrust_C),
                D=int(thrust_D)
            )
            drone_driver.manual_thrusts(t_data)
        
        # Moderate frequency loop for manual stabilization
        await asyncio.sleep(0.05)


async def keyboard_task(controller: AutoHoverController):
    log("\n[kbd] i: up | o: down | 1: start | 2: stop | q: quit")

    loop = asyncio.get_running_loop()
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    os.set_blocking(fd, False)

    queue = asyncio.Queue()

    def on_input():
        try:
            char = os.read(fd, 1).decode('utf-8')
            if char:
                queue.put_nowait(char.lower())
        except BlockingIOError:
            pass

    try:
        tty.setraw(fd)
        loop.add_reader(fd, on_input)

        while not controller.shutdown.is_set():
            try:
                cmd = await asyncio.wait_for(queue.get(), timeout=2)
                
                match cmd:
                    case "i":
                        controller.increase_thrust(5)
                    case "o":
                        controller.increase_thrust(-5)
                    case "1":
                        await controller.start()
                        log("\n[kbd] Hover ENABLED")
                    case "2":
                        controller.stop()
                        log("\n[kbd] Hover DISABLED")
                    case "q":
                        log("\n[kbd] Shutting down")
                        controller.request_shutdown()

                    case _:
                        pass
            except asyncio.TimeoutError:
                pass
            
    finally:
        loop.remove_reader(fd)
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


async def main():
    controller = AutoHoverController()
    
    log("---- IMU status ----")
    log(f"pitch={drone_driver.get_pitch()}")
    log(f"roll={drone_driver.get_roll()}")
    
    tasks = [
        asyncio.create_task(keyboard_task(controller), name="Keyboard"),
        asyncio.create_task(hover_task(controller), name="HoverFeedbackLoop"),
    ]
    
    try:
        await controller.shutdown.wait()
    finally:
        controller.request_shutdown()

        for task in tasks:
            task.cancel()

        from contextlib import suppress
        for task in tasks:
            with suppress(asyncio.CancelledError):
                await task


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        log("\nInterrupted by user, exiting...")
