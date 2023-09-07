#Luca Ciampaglia
#SME Soft Robotics Lab
#Case Western Reserve University

#CCoMa Multi-Segment Manipulator Example

import pybullet as p
import time
import pybullet_data
from CCoMa import Cable, CableSet, ContinuumManipulator, ActuatorMotor, generateURDF

# Set up the simulation
physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally

# Create the physical environment 
p.setGravity(0,0,-9.81) # creates gravity (not enabled by default)
planeId = p.loadURDF("plane.urdf") # loads the surface plane (not by default)


# Configure the GUI Windows

#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) # hides the interface panels, useful but also disables sliders
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0) # hide rgb buffer window
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0) # hide depth buffer window
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0) # hide segmentation mark window


# Create the Continuum Manipulator

startPos = [0,0,2] # Set a start position for the manipulator [xyz]
startOrientation = p.getQuaternionFromEuler([0,0,0]) # Set a start orientation for the manipulator [xyzw]

# Loads the continuum manipulator from urdf
# use flags p.URDF_USE_INERTIA_FROM_FILE if you want custom inertias or p.URDF_USE_SELF_COLLISION if you want self collision
bot = p.loadURDF("multi_segment_example_manipulator.urdf",startPos, startOrientation, 0, 1) 

numJoints = p.getNumJoints(bot)

# Generates the two sets of cables for actuation
cables1 = CableSet(-1,9,3,None,[0.5,0.0,0.7,0.6])
cables2 = CableSet(9,19,3,None,[0.0,0.0,1.0,0.6])
cables1.generateCables(0.04,0.0,1.0/20,0.0)
cables2.generateCables(0.04,0.0,1.0/20,0.0)

# Generates the user debug sliders for setting cable tensions
cables1.generateDebugSliders(400)
cables2.generateDebugSliders(400)

# Initializes the loaded URDF as a ContinuumManipulator object with the two sets of cables
manip = ContinuumManipulator(bot,[cables1,cables2],10.0,1.0/20,False,5,5.0)

# Set up GUI sliders for spring constant and simulation timestep size
spring = p.addUserDebugParameter("Spring", 1.0, 200.0, 50.0)
simspeed = p.addUserDebugParameter("Sim Speed", 1.0, 15.0, 1.0)

# Adjust the visuals of the manipulator
manip.showSegmentColors() # Uses the segment colors for links
manip.showCables() # Draws the cables on the screen

# Run the simulation
for i in range (1000000): # runs the simulation for 1000000 steps
    
    manip.setSpringConstant(p.readUserDebugParameter(spring)) # Updates the spring parameter of the manipulator from the slider value
    cables1.readDebugSliders() # Updates the cable tensions using the sliders
    cables2.readDebugSliders() # Updates the cable tensions using the sliders
    manip.stepSimulation() # Updates all forces in the manipulator

    p.stepSimulation() # Moves sim one step forward
    time.sleep(1/(p.readUserDebugParameter(simspeed)*240.)) # Rest 1/240th seconds


p.disconnect()
