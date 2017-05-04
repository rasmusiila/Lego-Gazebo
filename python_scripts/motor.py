import sys
import os
sys.path.append(os.path.abspath("/home/osboxes/lego_project_git/lego-project/python_scripts/ev3devmocka"))

import ev3devmocka.ev3 as ev3
import rospy
import time

print('bonjour')

m = ev3.LargeMotor('outA')
# m1 = ev3.LargeMotor('outD')
#
# print(m.max_speed)
#
m.speed_sp = 200
m.time_sp = 6000
# m.duty_cycle_sp = 100
#
m.run_forever()
time.sleep(1)
m.speed_sp = 600
m.run_forever()
time.sleep(4)
#
m.stop()
