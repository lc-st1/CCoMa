#Luca Ciampaglia
#SME Soft Robotics Lab
#Case Western Reserve University

#Soft Cable-Driven Continuum Manipulator Simulation
#Uses CCoMa

import pybullet as p
import time
import pybullet_data
from CCoMa import Cable, CableSet, ContinuumManipulator, ActuatorMotor

#set up the simulation environment

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
#p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) #hides the interface panels, useful but also disables sliders
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81) #creates gravity (not enabled by default)
planeId = p.loadURDF("plane.urdf") #loads the surface plane (not by default)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0) #hide rgb buffer window
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,0) #hide depth buffer window
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,0) #hide segmentation mark window

#spawn the manipulator

startPos = [0,0,2] #creates the start position [x,y,z]
startOrientation = p.getQuaternionFromEuler([0,0,0]) #creates the start orientation as a quaternion
bot = p.loadURDF("z_gen_test5.urdf",startPos, startOrientation, 0, 1) #loads the continuum manipulator from urdf, use p.URDF_USE_INERTIA_FROM_FILE if you want custom inertias or p.URDF_USE_SELF_COLLISION if you want self collision (not fixed yet)


numJoints = p.getNumJoints(bot)
cables1 = CableSet(-1,9,3,None,[0.5,0.0,0.7,0.6])
cables2 = CableSet(9,19,3,None,[0.0,0.0,1.0,0.6])
# cables1 = CableSet(-1,8,3,None,[0.5,0.0,0.7,0.6])

cables1.generateCables(0.04,0.0,1.0/20,0.0)
cables2.generateCables(0.04,0.0,1.0/20,0.0)
# cables1.generateCables(0.04,0.0,0.1,0.0)


sliders=cables1.generateDebugSliders(400)
cables2.generateDebugSliders(400)

manip = ContinuumManipulator(bot,[cables1,cables2],10.0,1.0/20,False,5,5.0)
# manip = ContinuumManipulator(bot,[cables1],10.0,0.1,False,5,1.0)



#set up simulation constants

#set up GUI sliders
spring = p.addUserDebugParameter("Spring", 1.0, 200.0, 50.0)
simspeed = p.addUserDebugParameter("Sim Speed", 1.0, 15.0, 1.0)
manip.showSegmentColors()
manip.showCables()

motors = []
for c in cables1.cables:
    motors.append(ActuatorMotor(manip,c,200,1.0))
#run the simulation 
for i in range (1000000): #runs the simulation for 1000000 steps
    
    manip.setSpringConstant(p.readUserDebugParameter(spring))
    cables1.readDebugSliders()
    cables2.readDebugSliders()
    # manip.setCableTensions(0,[p.readUserDebugParameter(sliders[0]),p.readUserDebugParameter(sliders[0]),p.readUserDebugParameter(sliders[0])])
    manip.stepSimulation()
    # for m in motors:
        # m.stepSimulation()
    for c in cables1.cables:
        print(str(c.length), "\t", str(c.tension))
    # for j in range(3):
        # motors[j].setMotorControl(p.TORQUE_CONTROL,p.readUserDebugParameter(sliders[j]))
    p.stepSimulation() #moves sim one step forward
    time.sleep(1/(p.readUserDebugParameter(simspeed)*240.)) #rest 1/240th seconds


p.disconnect()
