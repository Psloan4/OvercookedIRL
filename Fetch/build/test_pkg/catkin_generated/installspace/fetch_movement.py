#!/usr/bin/env python3
import threading

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard  # pip install pynput

MOVE_BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}

SPEED = 2.0
TURN  = 3.0


def main():
    rospy.init_node('fetch_teleop_keyboard')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(60)

    print("""
Fetch keyboard teleop (pynput, no deadman)
-----------------------------------------
Controls:
    w : forward
    s : backward
    a : rotate left
    d : rotate right
    x or SPACE : stop

    CTRL+C in this terminal to quit.
""")

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    state = {
        "lin": 0.0,
        "ang": 0.0,
    }
    state_lock = threading.Lock()

    def stop_motion():
        with state_lock:
            state["lin"] = 0.0
            state["ang"] = 0.0

    def on_press(key):
        ch = None
        try:
            ch = key.char
        except AttributeError:
            pass

        if ch is not None:
            if ch in MOVE_BINDINGS:
                lin, ang = MOVE_BINDINGS[ch]
                with state_lock:
                    state["lin"] = lin * SPEED
                    state["ang"] = ang * TURN
            elif ch == 'x':
                stop_motion()
        else:
            if key == keyboard.Key.space:
                stop_motion()

    def on_release(key):
        ch = None
        try:
            ch = key.char
        except AttributeError:
            pass

        if (ch in MOVE_BINDINGS) or (ch == 'x') or (key == keyboard.Key.space):
            stop_motion()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while not rospy.is_shutdown():
            with state_lock:
                twist.linear.x = state["lin"]
                twist.angular.z = state["ang"]

            print(f"\rmovement_vec: ({twist.linear.x:.2f}, {twist.angular.z:.2f})   ",
                  end='', flush=True)

            pub.publish(twist)
            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        listener.stop()
        print("\nExiting teleop.")


if __name__ == '__main__':
    main()
