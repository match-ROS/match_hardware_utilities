#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import numpy as np
from copy import deepcopy


class JoyCmdVelRelay:
    def __init__(self):
        rospy.init_node("joy_cmd_vel_relay")

        # --- Parameter ---
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/mur620c/cmd_vel")
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")

        self.publish_rate = rospy.get_param("~publish_rate", 50.0)   # Hz
        self.joy_timeout = rospy.get_param("~joy_timeout", 10.5)      # s

        self.max_lin_acc = rospy.get_param("~max_lin_acc", 0.4)       # m/s²
        self.max_ang_acc = rospy.get_param("~max_ang_acc", 0.6)       # rad/s²

        # --- State ---
        self.last_joy_time = 0.0
        self.enabled = False

        self.cmd_target = Twist()     # letzter Joy-Sollwert
        self.cmd_current = Twist()    # tatsächlich gesendeter Wert
        self.cmd_old = Twist()        # vorheriger gesendeter Wert

        # --- ROS ---
        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        rospy.Subscriber(self.joy_topic, Joy, self.joy_cb)

        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate),
            self.timer_cb
        )

    # --------------------------------------------------

    def joy_cb(self, msg: Joy):
        self.last_joy_time = rospy.get_time()

        # -------- Enable-Sequenz --------
        lt = msg.axes[2]
        rt = msg.axes[5]

        if not self.enabled:
            if lt <= -0.99 and rt <= -0.99:
                rospy.loginfo("Controller ENABLED")
                self.enabled = True
            else:
                # optional: aktiv stoppen
                rospy.loginfo_throttle(5.0, "Controller disabled; waiting for both triggers to be pressed to enable.")
                #self.pub.publish(Twist())
                return


        # -----------------------------
        # Achsen-Indizes (Logitech F710, X-Mode!)
        # -----------------------------
        AX_LEFT_X   = 7
        AX_LEFT_Y   = 1
        AX_RIGHT_X  = 3

        AX_LT = 2   # linker Trigger  (-1 ungedrückt → +1 gedrückt)
        AX_RT = 5   # rechter Trigger

        AX_DPAD_X = 0
        AX_DPAD_Y = 1

        # -----------------------------
        # Geschwindigkeitsgrenzen
        # -----------------------------
        if not hasattr(self, "max_lin_speed"):
            self.max_lin_speed = 0.5   # m/s
            self.max_ang_speed = 1.0   # rad/s

        # -----------------------------
        # D-Pad: Limits anpassen
        # -----------------------------
        if msg.axes[AX_DPAD_Y] > 0.5:
            self.max_lin_speed += 0.1
        elif msg.axes[AX_DPAD_Y] < -0.5:
            self.max_lin_speed = max(0.0, self.max_lin_speed - 0.1)

        if msg.axes[AX_DPAD_X] > 0.5:
            self.max_ang_speed += 0.1
        elif msg.axes[AX_DPAD_X] < -0.5:
            self.max_ang_speed = max(0.0, self.max_ang_speed - 0.1)

        # -----------------------------
        # Angular.z: beide Joysticks X
        # -----------------------------
        ang = msg.axes[AX_LEFT_X] + msg.axes[AX_RIGHT_X]
        ang = max(-1.0, min(1.0, ang))
        self.cmd_target.angular.z = ang * self.max_ang_speed

        # -----------------------------
        # Linear.x: Trigger
        # -----------------------------
        # Trigger sind [-1, 1] → auf [0, 1] normieren
        rt = ( msg.axes[AX_RT]-1) * 0.5
        lt = (1.0 - msg.axes[AX_LT]) * 0.5

        lin = - rt - lt
        self.cmd_target.linear.x = lin * self.max_lin_speed


    # --------------------------------------------------

    def timer_cb(self, event):
        now = rospy.get_time()
        # erster Timer-Callback → noch kein dt
        if event.last_real is None:
            return

        dt = (event.current_real - event.last_real).to_sec()
        if dt <= 0.0:
            return

        # --- Timeout → Ziel = 0 ---
        if now - self.last_joy_time > self.joy_timeout:
            self.cmd_target.linear.x = 0.0
            self.cmd_target.angular.z = 0.0

        # --- Sanfte Annäherung (Rate Limiter) ---
        self.cmd_current.linear.x = self._ramp(
            self.cmd_current.linear.x,
            self.cmd_target.linear.x,
            self.max_lin_acc,
            dt
        )

        self.cmd_current.angular.z = self._ramp(
            self.cmd_current.angular.z,
            self.cmd_target.angular.z,
            self.max_ang_acc,
            dt
        )

        if self.cmd_current.linear.x == 0.0 and self.cmd_current.angular.z == 0.0 and self.cmd_old.linear.x == 0.0 and self.cmd_old.angular.z == 0.0:
            # Beide Werte sind 0, nichts ändern und nicht senden
            return
        self.pub.publish(self.cmd_current)
        self.cmd_old = deepcopy(self.cmd_current)
    # --------------------------------------------------

    @staticmethod
    def _ramp(current, target, max_acc, dt):
        delta = target - current
        max_delta = max_acc * dt
        delta = np.clip(delta, -max_delta, max_delta)
        return current + delta


if __name__ == "__main__":
    try:
        JoyCmdVelRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
