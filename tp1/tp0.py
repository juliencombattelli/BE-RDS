#!/usr/bin/ipython

# Typical header of a Python script using Pinocchio
from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as se3
import gepetto.corbaserver

# Example of creation of an object of class Display (implemented in the previous example, inside
#  a file 'display.py'). Do not forget to start Gepetto-viewer server in another terminal before
#  creating the client.
"""from display import Display

display = Display()

# Example of use of the class Display to create a box visual object.
boxid   = 147
name    = 'box' + str(boxid)
[w,h,d] = [1.0,1.0,1.0]
color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
display.viewer.gui.addBox('world/'+name, w,h,d,color)

# Example of use of the class Display to create a sphere visual object.
display.viewer.gui.addSphere('world/sphere', 1.0,color)

# Example of use of the class Display to create a cylinder visual object.
radius = 1.0
height = 1.0
display.viewer.gui.addCylinder('world/cylinder', radius,height,color)

# Example of use of the class display to place the previously-create object at random SE3 placements.
display.place("world/box147",se3.SE3.Random(),False)
display.place("world/sphere",se3.SE3.Random(),False)
display.place("world/cylinder",se3.SE3.Random())"""

# Example of creation of a simple robot model with one single revolute joint rotating around axis X.
model = se3.Model.BuildEmptyModel()
jointName          = "first_joint"                       # Name of joint.
jointPlacement     = se3.SE3.Random()                    # SE3 placement of the joint wrt chain init.
parent             = 0                                   # Index of the parent (0 is the universe).
jointModel         = se3.JointModelRX()                  # Type of the joint to be created.
jointId = model.addJoint(parent,jointModel,jointPlacement,jointName)
print('Model dimensions: {:d}, {:d}, {:d}'.format(model.nq,model.nv,model.nbodies))

bodyInertia        = se3.Inertia.Random()                # Body mass/center of mass/inertia
model.appendBodyToJoint(jointId, bodyInertia, body_placement = se3.SE3.Identity()) # The last
                                                         # argument is likely to always be Identity.

# Example of how to create a 'Data' object from a 'Model' object.
data = model.createData()


