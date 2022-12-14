#!/usr/bin/env python3
import signal
import sys
import time
import os

import _clockkit
import rospy

rospy.init_node('clock_server')
config_file_path = rospy.get_param('~config_file_path', "1")

if config_file_path == "":
    rospy.logerr("No config file path provided")
    sys.exit(1)

sec_remaining = 5.0
terminate = sec_remaining > 0.0
if not terminate:
    sec_remaining = 1.0

_clockkit.ckInitialize(config_file_path)

# Trap ^C's SIGINT and pkill's SIGTERM,
# and pass the signal to clockkit.cpp's atexit(), which calls Clockkit.ckTerminate.
def signal_handler(sig, frame):
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

sec_remaining = float(5) if terminate else 1.0
while sec_remaining > 0.0:
    if _clockkit.ckInSync():
        offset = _clockkit.ckOffset()
        s = _clockkit.ckTimeAsString()
        t = _clockkit.ckTimeAsValue() # This might be 10 usec later than s.
        
        if int(offset) < 10:
            ns = 0
            num_str = repr(t)
            ns += int(num_str[-6])*1000000
            ns += int(num_str[-5])*100000
            ns += int(num_str[-4])*10000
            ns += int(num_str[-3])*1000
            ns += int(num_str[-2])*100
            ns += int(num_str[-1])*10
            bashCommand = "sudo date +%s.%N -s @" + str(int(t/1000000)) + "." + str(ns)
            os.system(bashCommand)
            break
        #print("offset:", _clockkit.ckOffset(), "\n", s)
    else:
        print("offset: OUT OF SYNC")
    time.sleep(0.1)
    if terminate:
        sec_remaining -= 0.1
    # print(sec_remaining, file=sys.stderr)

if terminate:
    # Also kill ckserver.
    _clockkit.ckKill()
else:
    _clockkit.ckTerminate()
