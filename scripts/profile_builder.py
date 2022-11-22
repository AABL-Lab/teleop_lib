#!/usr/bin/env python3

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

    def _updated(self, _):
        self._cb(self._index)

    def update(self, val):
        lbl = "Axis {self._index}: {val:.04f}"
        if self._labelvar.get() != lbl:
            self._labelvar.set(lbl)
            return True
        else:
            return False
    
    def configure(self, style):
        self._label.configure(style=style)
        self._selector.configure(style=style)
        self._scale.configure(style=style)
        self._deadzone.configure(style=style)

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
    
    def configure(self, style):
        self._label.configure(style=style)
        self._selector.configure(style=style)

    def get_config(self):
        return {
            "output": self._outputvar.get()
        }

def highlight(item):
    pass

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
        self._plugin_lock = threading.Lock()

        self._save_button = tkinter.Button(self._button_frame, text="Save", command=self._save)
        self._save_button.pack(side=tkinter.TOP)

        self._axes_editor = None
        self._button_editor = None

        self._ax_size = 0
        self._btn_size = 0

        self._input_config = {}
        self._input_profile = None
        self._profile_lock = threading.Lock()

        self._sub = rospy.Subscriber("joy", Joy, self._joy_cb, queue_size=1)

    def build(self):
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
        new_profile = teleop_lib.input_profile.build_profile(self._input_config)
        with self._profile_lock:
            # variable assignment is probably atomic but doesn't seem to be guaranteed in the spec
            # so just use a lock for safety
            # hopefully (almost certainly) this won't lag the ui
            self._input_profile = new_profile  

    def _update_plugin(self, _):
        plugin_cls = teleop_lib.plugins.get_plugin(self._plugin_var.get())
        plugin = plugin_cls()

        with self._plugin_lock:
            self._plugin = plugin

    def _joy_cb(self, msg):
        if self._label is not None:
            # first init
            self._ax_size = len(msg.axes)
            self._btn_size = len(msg.buttons)
            self._label.destroy()
            self._label = None

            self.build()
        else:
            if len(msg.axes) != self._ax_size or len(msg.buttons) != self._btn_size:
                rospy.logwarn(f"Different lengths detected for controller! Was {self._ax_size} axes, {self._btn_size} buttons; now {len(msg.axes)} axes, {len(msg.buttons)} buttons")
                # TODO: do we auto reconfigure? add a button to reconfigure? expect ppl to quit + rerun?
            else:
                self._axes_editor.update_vals(msg.axes)
                self._button_editor.update_vals(msg.buttons)

        with self._profile_lock:
            cmd = self._input_profile.process_input(msg)

        plugin = None
        with self._plugin_lock:
            plugin = self._plugin
        if plugin is not None:
            plugin.do_command(cmd)

if __name__ == "__main__":
    rospy.init_node("profile_builder", anonymous=True)
    root = tkinter.Tk()
    profile_builder = ProfileBuilder(root)
    root.mainloop()
        


