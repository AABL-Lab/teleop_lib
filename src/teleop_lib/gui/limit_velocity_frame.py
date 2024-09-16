#!/usr/bin/env python3

import functools
import numpy as np
import tkinter
from teleop_lib import command_queue

LIMIT_VELOCITY_NAME = "limit_velocity"
class LimitVelocityFrame(tkinter.Frame):
    def __init__(self, parent, initial_config):
        super().__init__(parent)
        config = initial_config.get(LIMIT_VELOCITY_NAME, {})

        all_btn = tkinter.Button(self, text="-", bg="green", command=functools.partial(self._click, "xyzrpw"))
        all_btn.grid(row=0, column=0, columnspan=2, sticky="nsew")

        trans_btn = tkinter.Button(self, text="-", bg="green", command=functools.partial(self._click, "xyz"))
        trans_btn.grid(row=1, column=0, sticky="nsew")

        rot_btn = tkinter.Button(self, text="-", bg="green", command=functools.partial(self._click, "rpw"))
        rot_btn.grid(row=1, column=1, sticky="nsew")

        self._btns = {
            "xyzrpw": all_btn,
            "xyz": trans_btn,
            "rpw": rot_btn
        }

        for col, coords in enumerate(["xyz", "rpw"]):
            for row, coord in enumerate(coords):
                btn = tkinter.Button(self, text=coord, bg="green", command=functools.partial(self._click, coord))
                btn.grid(row=row+2, column=col, sticky="nsew")
                self._btns[coord] = btn

        self._status = {c: config.get("status", {}).get(c, True) for c in "xyzrpw"}  # should be equivalent to config["status"] but just make sure
        self._set_btn_status()

        # unfortunate hack to pass info to the robot even though we don't have access
        # really the config frame is not built for this... there is definitely a better way to do it
        self._limiter = command_queue.LimitVelocityFilter()


    def _set_btn_status(self):
        for coords, btn in self._btns.items():
            st = [self._status[c] for c in coords]
            if all(st):
                # all unlocked so mark as unlocked
                btn.configure(bg="green")
                if len(st) > 1:
                    btn.configure(text="-")
            elif any(st):
                # mix of locked and unlocked
                # always len() > 1 since mix
                btn.configure(bg=self.cget("bg"), text="?")
            else:
                # all locked
                btn.configure(bg="red")
                if len(st) > 1:
                    btn.configure(text="x")

    def _click(self, coords):
        # update current status
        if len(coords) == 1:
            # just toggle it
            self._status[coords] = not self._status[coords]
        else:
            # we have multiple so it's complicated
            # if any are unlocked, 
            any_unlocked = any((self._status[c] for c in coords))
            for c in coords:
                # if any unlocked, lock them, else unlock them
                self._status[c] = not any_unlocked
        
        # update button status
        self._set_btn_status()

        # update projection map
        self._limiter.set_projection(np.diag([self._status[c] for c in "xyzrpw"]).astype(float))

    def set_state(self, state):
        # just leave it enabled all the time
        pass

    def get_config(self):
        return {
            LIMIT_VELOCITY_NAME: {
                "status": dict(self._status),
                "limiter": self._limiter.name
            } 
        }

def get_limiter(config):
    if LIMIT_VELOCITY_NAME in config:
        name = config[LIMIT_VELOCITY_NAME].get("limiter", None)
        if name is not None:
            return command_queue.LimitVelocityFilter.load(name)
    return None


def main():
    import asyncio
    from geometry_msgs.msg import TwistStamped, Vector3
    import study_runner

    async def run(config, status_cb):
        limiter = get_limiter(config)
        limiter.registerCallback(print)
        cmd = TwistStamped()
        cmd.twist.linear = Vector3(1, 2, 3)
        cmd.twist.angular = Vector3(4, 5, 6)

        while True:
            limiter.process(cmd)
            await asyncio.sleep(1.)

    root = tkinter.Tk()
    runner = study_runner.StudyRunner(root, run)
    runner.add_config_frame(LimitVelocityFrame, "Velocity")
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()






