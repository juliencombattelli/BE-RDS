#!/usr/bin/ipython

from FootSteps import *
import matplotlib.pyplot as plt
import numpy as np

class ZmpRef (object):
    def __init__ (self, footsteps) :
        self.footsteps = footsteps
	self.XY = [0.0, 0.0]
    # Operator ()
    def __call__ (self, t):
	if(self.footsteps.getPhaseType(t) == 'right'):
		self.XY = footsteps.getLeftPosition(t)
	elif(self.footsteps.getPhaseType(t) == 'left'):
		self.XY = footsteps.getRightPosition(t)
	elif(footsteps.isDoubleFromLeftToRight(t) == True):
		self.XY = np.array(footsteps.getRightPosition(t)) + (np.array(footsteps.getLeftPosition(t)) - np.array(footsteps.getRightNextPosition(t))) / footsteps.getPhaseDuration(t) *  (footsteps.getPhaseDuration(t) - footsteps.getPhaseRemaining(t));
	else:
		self.XY = np.array(footsteps.getLeftPosition(t)) + (np.array(footsteps.getRightPosition(t)) - np.array(footsteps.getLeftNextPosition(t))) / footsteps.getPhaseDuration(t) *  (footsteps.getPhaseDuration(t) - footsteps.getPhaseRemaining(t));
        return self.XY

'''
footsteps.getPhaseType(.4)           # return 'left'
footsteps.getLeftPosition(0.4)       # return 0,0.1
footsteps.getLeftNextPosition(0.4)   # return 0.1,0.1
footsteps.getPhaseStart(0.4)         # return 0.3
footsteps.getPhaseDuration(0.4)      # return 0.7
footsteps.getPhaseRemaining(0.4)     # return 0.6
footsteps.isDoubleFromLeftToRight(0) # return False
footsteps.isDoubleFromLeftToRight(1) # return True
'''

footsteps = FootSteps( [0.0,-0.1] , [0.0,0.1] )
footsteps.addPhase( .3, 'none' )
footsteps.addPhase( .7, 'left' , [0.1,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.2,-0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [0.3,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.4,-0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'left' , [0.5,+0.1] )
footsteps.addPhase( .1, 'none' )
footsteps.addPhase( .7, 'right', [0.5,-0.1] )
footsteps.addPhase( .5, 'none' )

zmp = ZmpRef(footsteps)

t = np.arange(0.,5.,0.01)
x = np.arange(0.,5.,0.01)
y = np.arange(0.,5.,0.01)

for i in range(0,t.size):
	x[i], y[i] = zmp(t[i])

print t.size
print x.size
print y

plt.plot(t, x, 'r', t, y, 'b')
plt.show()

