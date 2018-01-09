#!/usr/bin/ipython

import numpy as np
from robot import Robot
from scipy.optimize import fmin_bfgs, fmin_slsqp
import pinocchio as se3
from pinocchio.utils import *
import time
from FootSteps import *

robot = Robot()
i = 0

def cost(q):
	robot.q = q
	se3.forwardKinematics(robot.model, robot.data,q)
	#index = robot.indexEffector
	pu = robot.data.oMi[8] * robot.trans
	p = pu.translation
	pdes = se3.SE3(eye(3),np.matrix([-0.5+i*0.01,0. + 0.02*i,2+-1.-i*0.1])).translation
	return np.sqrt(np.square(p[0] - pdes[0]) + np.square(p[1] - pdes[1]) + np.square(p[2] - pdes[2]))

def constraint_eq(x):
     ''' Constraint x^3 = y '''
     return np.array([ x[0]**3-x[1] ])

def constraint_ineq(x):
     '''Constraint x>=2, y>=2'''
     return np.array([ x[0]-2,x[1]-2 ])

class CallbackLogger:
	def __init__(self):
		self.nfeval = 1
	def __call__(self,x):
		print '===CBK=== {0:4d}   {1: 3.6f}   {2: 3.6f}'.format(self.nfeval, x[0], x[1], cost(x))
		self.nfeval += 1


robot.display(robot.q0)
q = robot.q0

for i in range(0,20):
	# Optimize cost without any constraints in BFGS, with traces.
	try:
		xopt_bfgs = fmin_bfgs(cost, q, callback=CallbackLogger())
		print '\n *** Xopt in BFGS = ',xopt_bfgs,'\n\n\n\n'
	except:
		pass
	robot.display(robot.q)
	time.sleep(0.5)
for i in range(20,0,-1):
	# Optimize cost without any constraints in BFGS, with traces.
	try:
		xopt_bfgs = fmin_bfgs(cost, q, callback=CallbackLogger())
		print '\n *** Xopt in BFGS = ',xopt_bfgs,'\n\n\n\n'
	except:
		pass
	robot.display(robot.q)
	time.sleep(0.5)

'''
# Optimize cost without any constraints in CLSQ
xopt_lsq = fmin_slsqp(cost,[-1.0,1.0], iprint=2, full_output=1)
print '\n *** Xopt in LSQ = ',xopt_lsq,'\n\n\n\n'

# Optimize cost with equality and inequality constraints in CLSQ
xopt_clsq = fmin_slsqp(cost,[-1.0,1.0],
                       f_eqcons=constraint_eq, f_ieqcons=constraint_ineq,
                       iprint=2, full_output=1)
print '\n *** Xopt in c-lsq = ',xopt_clsq,'\n\n\n\n'
'''

