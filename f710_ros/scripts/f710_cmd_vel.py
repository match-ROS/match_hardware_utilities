#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class F710CmdVel:
    def __init__(self):
        rospy.init_node("f710_cmd_vel")

        self.pub = rospy.Publisher(
            "/mur620c/cmd_vel",
            Twist,
            queue_size=10
        )

        rospy.Subscriber("/joy", Joy, self.joy_cb)

        # Maximalgeschwindigkeiten
        self.max_lin = 0.2     # m/s
        self.max_ang = 0.4     # rad/s

        self.lin_step = 0.1
        self.ang_step = 0.1

        self.prev_dpad = [0.0, 0.0]

    def joy_cb(self, msg):
        twist = Twist()

        # -------- Rotation (beide Joysticks X) --------
        left_x  = msg.axes[0]
        right_x = msg.axes[3]

        # Summe, gegensinnig = Subtraktion automatisch
        rot = left_x + right_x
        twist.angular.z = -rot * self.max_ang
        # Minus: rechts → Uhrzeigersinn

        # -------- Translation (Trigger) --------
        rt = msg.axes[5]  # −1 gedrückt
        lt = msg.axes[2]

        lin = 0.0
        if rt < 0:
            lin += (-rt) * self.max_lin
        if lt < 0:
            lin -= (-lt) * self.max_lin

        twist.linear.x = lin

        # -------- D-Pad Geschwindigkeitsanpassung --------
        dpad_x = msg.axes[6]
        dpad_y = msg.axes[7]

        # nur Flanken
        if dpad_y > 0 and self.prev_dpad[1] <= 0:
            self.max_lin += self.lin_step
        if dpad_y < 0 and self.prev_dpad[1] >= 0:
            self.max_lin = max(0.0, self.max_lin - self.lin_step)

        if dpad_x > 0 and self.prev_dpad[0] <= 0:
            self.max_ang += self.ang_step
        if dpad_x < 0 and self.prev_dpad[0] >= 0:
            self.max_ang = max(0.0, self.max_ang - self.ang_step)

        self.prev_dpad = [dpad_x, dpad_y]

        self.pub.publish(twist)

if __name__ == "__main__":
    F710CmdVel()
    rospy.spin()
