#!/usr/bin/env python

import copy
from pynput.keyboard import Key, KeyCode, Listener
import rospy
import threading

from sensor_msgs.msg import Joy

class KeyState:
    def __init__(self, key_filt=None):
        self._lock = threading.Lock()
        self._key_filt = key_filt
        self.reset()

    def reset(self):
        with self._lock:
            if self._key_filt is None:
                self._state = {}
            else:
                self._state = { k: False for k in self._key_filt }

    def on_pressed(self, key):
        if self._key_filt is None or key in self._key_filt:
            with self._lock:
                self._state[key] = True
    
    def on_released(self, key):
        if self._key_filt is None:
            with self._lock:
                if key in self._state:
                    del(self._state[key])
                else:
                    print('warning: released key {} but not marked as pressed'.format(key))
        elif key in self._key_filt:
            with self._lock:
                if not self._state[key]:
                    print('warning: released key {} but not marked as pressed'.format(key))
                else:
                    self._state[key] = False

    def get_data(self):
        with self._lock:
            return copy.deepcopy(self._state)

KEYS_UP = { Key.up, KeyCode.from_char('w') }
KEYS_DOWN = { Key.down, KeyCode.from_char('s') }
KEYS_LEFT = { Key.left, KeyCode.from_char('a') }
KEYS_RIGHT = { Key.right, KeyCode.from_char('d') }
KEYS_BUTTON = { Key.space }

KEYS_USED = KEYS_UP | KEYS_DOWN | KEYS_LEFT | KEYS_RIGHT | KEYS_BUTTON
def get_message_from_key_state(state):
    x_axis = any(state[k] for k in KEYS_RIGHT) - any(state[k] for k in KEYS_LEFT)
    y_axis = any(state[k] for k in KEYS_UP) - any(state[k] for k in KEYS_DOWN)
    button = any(state[k] for k in KEYS_BUTTON)
    return Joy(
        axes = [x_axis, -y_axis, 0.],
        buttons = [button]
    )

def main():
    rospy.init_node('keyboard_to_joystick', anonymous=True)

    key_state = KeyState(KEYS_USED)
    listener = Listener(
        on_press=key_state.on_pressed,
        on_release=key_state.on_released)  
    listener.start()

    pub = rospy.Publisher('joy', Joy, queue_size=1)
    
    def publish_msg(_):
        pub.publish(get_message_from_key_state(key_state.get_data()))

    pub_rate = rospy.get_param('pub_rate', 100.)
    timer = rospy.Timer(rospy.Duration(1./pub_rate), publish_msg)

    print('Ready')
    rospy.spin()

    # clean up
    listener.stop()
    listener.join()

if __name__ == "__main__":
    main()



