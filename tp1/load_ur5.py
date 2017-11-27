import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import *

pkg = '/home/student/models/'
urdf = pkg + 'ur_description/urdf/ur5_gripper.urdf'
                     
robot = RobotWrapper(urdf,[pkg,])
robot.initDisplay(loadModel=True)

NQ = robot.model.nq
NV = robot.model.nv

q  = rand(NQ)
vq = zero(NV)
vq[3] = 1

robot.display(q)

from time import sleep
for i in range(10000):
    q += vq/100
    robot.display(q)
    sleep(.01)
