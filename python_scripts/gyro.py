import sys
import os
# sys.path.append(os.path.abspath("/home/osboxes/lego_project_git/lego-project/python_scripts/ev3devmocka"))

import ev3devmocka.ev3 as ev3
import rospy
import time

m = ev3.LargeMotor('outA')
c = ev3.GyroSensor('In1')

m.speed_sp = 360
time.sleep(3)

t = time.time()
m.run_forever()
while True:
    print(c.angle)
    if c.angle >= 1800:
        print("OMG 10 spins!")
        print(time.time() - t)
        break

m.stop()
