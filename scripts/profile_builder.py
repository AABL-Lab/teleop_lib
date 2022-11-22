#!/usr/bin/env python3

import queue
import rospy
from sensor_msgs.msg import Joy
import threading
import tkinter
import tkinter.filedialog
import tkinter.ttk
import yaml

from teleop_lib.msg import RobotCommand
import teleop_lib.input_profile
import teleop_lib.plugins

# find some less terrible way to do this??
AXIS_VALUES = teleop_lib.input_profile.TwistAxis.list() + [""]
BUTTON_VALUES = [ c for c in vars(RobotCommand) if c.isupper() ] + [""]

class AxisEditor:
    def __init__(self, parent, index, cb):
        self._index = index
        self._cb = cb

        self._labelvar = tkinter.StringVar()
        self._label = tkinter.ttk.Label(parent, textvariable=self._labelvar)
        self._label.grid(column=0, row=self._index)
        self.update(0)

        self._axisvar = tkinter.StringVar()
        self._selector = tkinter.ttk.Combobox(parent, values=AXIS_VALUES, state="readonly")
        self._selector.grid(column=1, row=self._index)
        self._selector.bind("<<ComboboxSelected>>", self._updated)

        self._scale = tkinter.Spinbox(parent, value=1, increment=0.1)
        self._scale.grid(column=2, row=self._index)
        self._scale.bind("<Key-Return>", self._updated)

        self._deadzone = tkinter.Spinbox(parent, value=0, increment=0.01)
        self._deadzone.grid(column=3, row=self._index)
        self._deadzone.bind("<Key-Return>", self._updated)

    ## definitely unnecessary haxx
    def after(self, *args):
        self._label.master.after(*args)

    def _updated(self, _):
        self._cb(self._index)

    def update(self, val):
        lbl = f"Axis {self._index}: {val:.04f}"
        if self._labelvar.get() != lbl:
            self._labelvar.set(lbl)
            return True
        else:
            return False
    
    def configure(self, **kwargs):
        self._label.configure(**kwargs)
        self._selector.configure(**kwargs)
        self._scale.configure(**kwargs)
        self._deadzone.configure(**kwargs)

    def get_config(self):
        return {
            "output": self._axisvar.get(),
            "scale": float(self._scale.get()),
            "deadzone": float(self._deadzone.get())
        }

class ButtonEditor:
    def __init__(self, parent, index, cb):
        self._index = index
        self._cb = cb

        self._labelvar = tkinter.StringVar()
        self._label = tkinter.ttk.Label(parent, textvariable=self._labelvar)
        self._label.grid(column=0, row=self._index)
        self.update(False)

        self._outputvar = tkinter.StringVar()
        self._selector = tkinter.ttk.Combobox(parent, values=BUTTON_VALUES, state="readonly")
        self._selector.grid(column=1, row=self._index)
        self._selector.bind("<<ComboboxSelected>>", self._updated)

    ## definitely unnecessary haxx
    def after(self, *args):
        self._label.master.after(*args)

    def _updated(self, _):
        self._cb(self._index)

    def update(self, val):
        state = "on" if val else "off"
        lbl = f"Button {self._index}: {state}"
        if self._labelvar.get() != lbl:
            self._labelvar.set(lbl)
            return True
        else:
            return False
    
    def configure(self, **kwargs):
        self._label.configure(**kwargs)
        self._selector.configure(**kwargs)

    def get_config(self):
        return {
            "output": self._outputvar.get()
        }

DEFAULT_BG = (0xd9, 0xd9, 0xd9)
HIGHLIGHT_BG = (0xF0, 0x80, 0x00)
HIGHLIGHT_RATE = 50 # ms
HIGHLIGHT_TIME = 2000 # total ms

def _update_highlight(item):
    if item._highlight_val < HIGHLIGHT_RATE/HIGHLIGHT_TIME:
        item._highlight_val = 0
    r = int(DEFAULT_BG[0]*(1-item._highlight_val) + HIGHLIGHT_BG[0]*item._highlight_val)
    g = int(DEFAULT_BG[1]*(1-item._highlight_val) + HIGHLIGHT_BG[1]*item._highlight_val)
    b = int(DEFAULT_BG[2]*(1-item._highlight_val) + HIGHLIGHT_BG[2]*item._highlight_val)
    item.configure(background=f"#{r:02x}{g:02x}{b:02x}")
    item._highlight_val -= HIGHLIGHT_RATE/HIGHLIGHT_TIME
    if item._highlight_val >= 0:
        item.after(HIGHLIGHT_RATE, lambda: _update_highlight(item))
    else:
        delattr(item, '_highlight_val')


def highlight(item):
    is_active = hasattr(item, "_highlight_val")
    item._highlight_val = 1
    if not is_active:
        item.after(HIGHLIGHT_RATE, lambda: _update_highlight(item))

class InputEditor(tkinter.Frame):
    def __init__(self, parent, editor_factory, n, cb):
        super().__init__(parent)

        self._cb = cb
        self._editors = []
        for i in range(n):
            self._editors.append(editor_factory(self, i, self._input_changed))

    def _input_changed(self, i):
        self._cb(i, self._editors[i].get_config())

    def get_config(self):
        return [ editor.get_config() for editor in self._editors ]

    def update_vals(self, vals):
        for i, val in enumerate(vals):
            if self._editors[i].update(val):
                highlight(self._editors[i])

    def __len__(self):
        return len(self._editors)


class ProfileBuilder(tkinter.Frame):
    def __init__(self, parent):
        super().__init__(parent)

        self._data_frame = tkinter.Frame(self)
        self._data_frame.pack(side=tkinter.TOP, fill=tkinter.BOTH, expand=True)

        self._label = tkinter.Label(self._data_frame, text="Waiting for first message...")
        self._label.pack()

        self._button_frame = tkinter.Frame(self)
        self._button_frame.pack(side=tkinter.BOTTOM, fill=tkinter.X, expand=True)

        self._plugin = None
        self._plugin_var = tkinter.StringVar
        self._plugin_selector = tkinter.ttk.Combobox(self._button_frame, values=teleop_lib.plugins.list_plugins()+[""], textvariable=self._plugin_var)
        self._plugin_selector.bind("<<ComboboxSelected>>", self._update_plugin)

        self._save_button = tkinter.Button(self._button_frame, text="Save", command=self._save)
        self._save_button.pack(side=tkinter.TOP)

        self._axes_editor = None
        self._button_editor = None

        self._ax_size = 0
        self._btn_size = 0

        self._input_config = {}
        self._input_profile = None

        self._building = False

        self._msg_queue = queue.Queue(maxsize=1) # drop old messages
        self._sub = rospy.Subscriber("joy", Joy, self._joy_cb, queue_size=1)
        self._poll_for_msgs()


    def build(self):
        if self._label is not None:
            self._label.destroy()

        self._axes_editor = InputEditor(self._data_frame, AxisEditor, self._ax_size, lambda v: self._config_changed("axes", v))
        self._button_editor = InputEditor(self._data_frame, ButtonEditor, self._btn_size, lambda v: self._config_changed("buttons", v))

        self._input_config = { 
            "axes": self._axes_editor.get_config(),
            "buttons": self._button_editor.get_config()
        }
        self._input_profile = teleop_lib.input_profile.build_profile(self._input_config)

        # display panels AFTER initial input profile update so we don't need to lock
        self._axes_editor.grid(row=0, column=0)
        self._button_editor.grid(row=0, column=1)

    def _save(self):
        fn = tkinter.filedialogue.asksaveasfilename(title="Save input profile configuration", defaultextension=".yaml")
        with open(fn, 'w') as f:
            # self._input_config can only be changed in the gui event loop
            # so no need to lock
            yaml.safe_dump(f, self._input_config)

    def _config_changed(self, input, index, config):
        self._input_config[input][index] = config
        self._input_profile = teleop_lib.input_profile.build_profile(self._input_config) 

    def _update_plugin(self, _):
        plugin_cls = teleop_lib.plugins.get_plugin(self._plugin_var.get())
        plugin = plugin_cls()

        with self._plugin_lock:
            self._plugin = plugin

    def _update_display(self, msg):
        self._axes_editor.update_vals(msg.axes)
        self._button_editor.update_vals(msg.buttons)

    def _joy_cb(self, msg):
        try:
            self._msg_queue.put_nowait(msg)
        except queue.Full:
            pass

    def _poll_for_msgs(self):
        try:
            msg = self._msg_queue.get_nowait()
        except queue.Empty:
            pass
        else:
            self._joy_event_cb(msg)
        self.after(100, self._poll_for_msgs)      

    def _joy_event_cb(self, msg):

        if not self._building:
            self._building = True
            # first init
            self._ax_size = len(msg.axes)
            self._btn_size = len(msg.buttons)
            self.build()
        else:
            if len(msg.axes) != self._ax_size or len(msg.buttons) != self._btn_size:
                rospy.logwarn(f"Different lengths detected for controller! Was {self._ax_size} axes, {self._btn_size} buttons; now {len(msg.axes)} axes, {len(msg.buttons)} buttons")
                # TODO: do we auto reconfigure? add a button to reconfigure? expect ppl to quit + rerun?
            else:
                self._update_display(msg)

        cmd = self._input_profile.process_input(msg)

        if self._plugin is not None:
            self._plugin.do_command(cmd)

if __name__ == "__main__":
    rospy.init_node("profile_builder", anonymous=True)
    root = tkinter.Tk()
    profile_builder = ProfileBuilder(root)

    ### HAX -- FOR TESTING ONLY
    profile_builder._plugin = teleop_lib.plugins.get_plugin("publisher")()


    # root.mainloop()
        


