# teleop_lib
ROS library for translating controller input into robot arm control.


## Concepts
This library works by generalizing a command from an input device (joystick, controller, etc.) to a robot command. The abstraction used is generated from `RobotCommand.msg`.

An **input profile** is a function that takes in a `sensor_msgs/Joy` message and converts it into a `teleop_lib/RobotCommand` message. One way to do this, which is supported by the library, is to map some of the joystick axes to some of the axes in the `geometry_msgs/Twist` command, and to map some of the buttons to the `command` field. This behavior is supported by `teleop_lib.input_profile`. The class `FixedInputProfile` performs this transformation from a specified configuration dictionary; `build_profile` takes a generic dictionary with only string and numeric elements and provides an appropriate profile. Therefore, profiles can be loaded from YAML files; see examples in `data`.

A **plugin** is a class that consumes a `teleop_lib/RobotCommand` message. A simple plugin, which publishes the command to a topic, is available in `plugins/publisher.py`. Custom plugins can be built and registered using `import teleop_lib.plugins; teleop_lib.plugins.register_plugin(name, class)`.

## Usage

The `controller_interface` connects an input profile to a plugin.


This package also provides `profile_builder.py`, which creates a GUI (using Tkinter) to help create of these profiles as YAML files. The GUI listens on the `/joy` topic (remap it through `rosrun`) and when it receives a message, it displays options for mapping the axes and buttons pressed to axes and commands in `teleop_lib/RobotCommand`. The GUI highlights axes that have recently changed and enables selection of a registered plugin, so it can be used in situ to map the controls appropriately. Press Save to dump the current configuration as a YAML file which can be subsequently loaded in and converted via `build_profile`.

