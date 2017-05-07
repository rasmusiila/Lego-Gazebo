import sys
import os
sys.path.append(os.path.abspath("/home/osboxes/lego_project_git/lego-project/python_scripts/ev3devmocka"))

import ev3devmocka.ev3 as ev3
import rospy
import time


c = ev3.ColorSensor('in3')

while True:
    print(c.color)


