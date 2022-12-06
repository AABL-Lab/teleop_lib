#!/usr/bin/env python3

import asyncio
import os
import study_runner
from study_runner.frames.logging import LoggingFrame, RunLogging
import teleop_lib.plugins
import tkinter
import tkinter.filedialog
import tkinter.ttk
import yaml


## https://stackoverflow.com/a/25023944
class DragDropListbox(tkinter.Listbox):
    """ A Tkinter listbox with drag'n'drop reordering of entries. """
    def __init__(self, master, **kw):
        kw['selectmode'] = tkinter.SINGLE
        super().__init__(master, kw)
        self.bind('<Button-1>', self.setCurrent)
        self.bind('<B1-Motion>', self.shiftSelection)
        self.bind("<BackSpace>", self._delete_current)
        self.bind("<Delete>", self._delete_current)
        self.curIndex = None

    def setCurrent(self, event):
        self.curIndex = self.nearest(event.y)

    def shiftSelection(self, event):
        i = self.nearest(event.y)
        if i < self.curIndex:
            x = self.get(i)
            self.delete(i)
            self.insert(i+1, x)
            self.curIndex = i
        elif i > self.curIndex:
            x = self.get(i)
            self.delete(i)
            self.insert(i-1, x)
            self.curIndex = i

    def _delete_current(self, evt):
        self.delete(self.curselection())


TELEOP_CONFIG_NAME = "teleop"
class TeleopConfigFrame(tkinter.Frame):
    def __init__(self, parent, initial_config):
        super().__init__(parent)
        config = initial_config.get(TELEOP_CONFIG_NAME, {})

        # Plugin to use
        self._plugin_label = tkinter.Label(self, text="Plugin:")
        self._plugin_label.grid(row=0, column=0, sticky="NW")

        self._plugin_var = tkinter.StringVar(value=config.get("plugin", ""))
        self._plugin_selector = tkinter.ttk.Combobox(self, values=teleop_lib.plugins.list_plugins(), state="readonly", textvariable=self._plugin_var)
        self._plugin_selector.grid(row=0, column=1, sticky="NW")

        self.rowconfigure(0, weight=0)

        # Modal control definition
        self._control_label = tkinter.Label(self, text="Control modes:")
        self._control_label.grid(row=1, column=0, columnspan=2, sticky="NW")

        self.rowconfigure(1, weight=0)

        self._loaded_modes = { c["filename"] : c for c in config.get("modes", []) }

        self._mode_var = tkinter.StringVar(value=[c["filename"] for c in config.get("modes", []) if "filename" in c])
        self._mode_list = DragDropListbox(self, height=4, listvariable=self._mode_var)
        self._mode_list.grid(row=2, column=0, columnspan=2, sticky="NS")
        self.rowconfigure(2, weight=1)
        
        self._load_button = tkinter.Button(self, text="Load mode file", command=self._load_mode)
        self._load_button.grid(row=3, column=1, sticky="NW")

    def _load_mode(self):
        fn = tkinter.filedialog.askopenfilename(filetypes=[("yaml", "*.yaml")])
        if fn is not None:
            try:
                with open(fn) as f:
                    mode = yaml.safe_load(f)
            except Exception as e:
                import traceback; traceback.print_exc()
            else:
                fn = os.path.basename(fn)
                mode["filename"] = fn
                self._loaded_modes[fn] = mode
                self._mode_list.insert(tkinter.END, fn)

    def set_state(self, state):
        self._plugin_selector.configure(state=state)
        self._mode_list.configure(state=state)
        self._load_button.configure(state=state)

    def get_config(self):
        return {
            TELEOP_CONFIG_NAME: {
                "plugin": self._plugin_var.get(),
                "modes": [ self._loaded_modes[m] for m in self._mode_list.get(0, tkinter.END) ]
            }
        }

async def run_teleop(config, status_cb):
    pass

def main():
    root = tkinter.Tk()
    runner = study_runner.StudyRunner(root, run_teleop)
    runner.add_config_frame(TeleopConfigFrame, "Teleoperation")
    logging_frame = runner.add_config_frame(LoggingFrame, "Logging")
    study_runner.runner.main(root)


if __name__ == "__main__":
    main()

