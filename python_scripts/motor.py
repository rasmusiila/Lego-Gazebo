import sys
import os
sys.path.append(os.path.abspath("/home/osboxes/lego_project_git/lego-project/python_scripts/ev3devmocka"))

import ev3devmocka.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.LargeMotor('outA')
m1 = ev3.LargeMotor('outD')

m.speed_sp = 360
m.time_sp = 10000
m1.speed_sp = 360
m1.time_sp = 10000
m.run_timed()
m1.run_timed()
