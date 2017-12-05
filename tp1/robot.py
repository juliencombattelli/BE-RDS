#!/usr/bin/ipython

from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import gepetto.corbaserver
from display import Display
import time

class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot with 7DOF (shoulder=3 + elbow=1 + wrist=3).
    The configuration is nq=7. The velocity is the same.
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot, each element of the list being
    an object Visual (see above).

    See tp1.py for an example of use.
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = se3.Model.BuildEmptyModel()
        self.createLeg6DOF()
        self.data = self.model.createData()
        self.q0 = zero(6)
	self.q = self.q0
	self.trans = se3.SE3(eye(3),np.matrix([-0.7,0.,-.05]))

    def createLeg6DOF(self,rootId=0,prefix='',jointPlacement=None):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

	sphereSize = .15

        name               = prefix+"hip1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = jointPlacement if jointPlacement!=None else se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        #self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere1', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere1',jointId,se3.SE3.Identity()) )

        name               = prefix+"hip2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        #self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere2', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere2',jointId,se3.SE3.Identity()) )

        name               = prefix+"hip3"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRZ(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere3', sphereSize ,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere3',3,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'thigh', .1,.1,.5,color)
        self.visuals.append( Visual('world/'+prefix+'thigh',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-.5]))))

        name               = prefix+"knee"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-1.0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere4', sphereSize,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere4',jointId,se3.SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/'+prefix+'calf', .1,.1,.5,color)
        self.visuals.append( Visual('world/'+prefix+'calf',jointId,se3.SE3(eye(3),np.matrix([0.,0.,-.5]))))
	
        name               = prefix+"hankle1"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3(eye(3),np.matrix( [0,0,-1.0] ))
        jointId = self.model.addJoint(jointId,se3.JointModelRX(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        #self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere5', 0.3,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere5',jointId,se3.SE3.Identity()) )

        name               = prefix+"hankle2"
        jointName,bodyName = [name+"_joint",name+"_body"]
        jointPlacement     = se3.SE3.Identity()
        jointId = self.model.addJoint(jointId,se3.JointModelRY(),jointPlacement,jointName)
        self.model.appendBodyToJoint(jointId,se3.Inertia.Random(),se3.SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/'+prefix+'sphere6', sphereSize,colorred)
        self.visuals.append( Visual('world/'+prefix+'sphere6',jointId,se3.SE3.Identity()) )
	self.viewer.viewer.gui.addBox('world/'+prefix+'foot', .5,.1,.1,color)
        self.visuals.append( Visual('world/'+prefix+'foot',jointId,se3.SE3(eye(3),np.matrix([-.3,0.,0.]))))

    def display(self,q):
	self.q = q
        se3.forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()

'''
robot = Robot()
robot.display(robot.q0)
q = robot.q0

for j in range(0,10):
	for i in range(0, 90):
		q[1] = q[1] + np.radians(1)
		q[3] = q[3] - np.radians(1)
		robot.display(q)
		time.sleep(0.01)
	for i in range(0, 90):
		q[1] = q[1] - np.radians(1)
		q[3] = q[3] + np.radians(1)
		robot.display(q)
		time.sleep(0.01)
'''

