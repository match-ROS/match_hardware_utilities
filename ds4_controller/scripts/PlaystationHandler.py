import rospy
from sensor_msgs.msg import Joy

class PlayStationHandler():
    # Button Mappings
    SQUARE = 0
    X = 1
    CIRCLE = 2
    TRIANGLE = 3
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    SHARE = 8
    OPTIONS = 9
    L3 = 10
    R3 = 11
    PS = 12
    TOUCHPAD = 13

    BUTTON_MAP = {
        SQUARE: "SQUARE",
        X: "X",
        CIRCLE: "CIRCLE",
        TRIANGLE: "TRIANGLE",
        L1: "L1",
        R1: "R1",
        L2: "L2",
        R2: "R2",
        SHARE: "SHARE",
        OPTIONS: "OPTIONS",
        L3: "L3",
        R3: "R3",
        PS: "PS",
        TOUCHPAD: "TOUCHPAD"
    }

    # Axis Mappings
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 5
    L2_AXIS = 3
    R2_AXIS = 4
    DPAD_X = 6
    DPAD_Y = 7

    def __init__(self):
        self._buttons = [0] * 14
        self._edges = [0] * 14
        self._axes = [0] * 8
        self.__sub_joy = rospy.Subscriber('/joy', Joy, self.__joy_callback__, queue_size=10)


    def __joy_callback__(self, msg):
        self._edges = [a and not b for a, b in zip(msg.buttons, self._buttons)]        
        self._buttons = msg.buttons
        self._axes = msg.axes

    def run(self):
        pass
