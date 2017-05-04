import sys
import os
# sys.path.append(os.path.abspath("/home/osboxes/lego_project_git/lego-project/python_scripts/ev3devmocka"))

import ev3devmocka.ev3 as ev3
import rospy
import time


c = ev3.GyroSensor('In1')

time.sleep(5)

for i in range(0, 9):
    print(c.angle)
    time.sleep(0.3)

for i in range(0, 9):
    print(c.rate)
    time.sleep(1)

for i in range(0, 9):
    print(c.angle)
    time.sleep(0.3)


