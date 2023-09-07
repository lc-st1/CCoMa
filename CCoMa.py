# Luca Ciampaglia
# SME Soft Robotics Lab
# Case Western Reserve University

# Cable-actuated Continuum Manipulator (CCoMa) Pybullet Extension Package
# Classes:
    # Cable
    # CableSet
    # ActuatorMotor
    # ActuatorPneumatic
    # ContinuumManipulator
# Functions: 
    # multiplyQuaternion()
    # vectorAngle()
    # invertQuaternion()
    # magnitude()
    # vecProject()
    # normalize()
    # quaternionFromAxisAngle()
    # applyRotation()
    # axisAngleFromQuaternion()
    # generateURDF()

# This is intended for use with Pybullet


import pybullet as p
import numpy
import math


# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# CLASSES:


class Cable:
    # a singular cable that has a tension, used in CableSet to affect continuum manipulators
    def __init__(self,
                 numJointsAffected, # total number of xyz joints affected
                 linkPositions, # the relative local position at each link affected
                 initialTension, # the initial tension of the cable
                 debugRenderColor # the RGB color of the cable if drawn for debug [R,G,B]          
                 ):
        self.numJointsAffected = numJointsAffected
        self.positions = linkPositions
        self.tension = initialTension
        self.debugRenderColor = debugRenderColor
        self.length = 0.0

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

class CableSet:
    # set of cable actuators with start and end link, cable positions, tensions, and overall segment coloration
    def __init__(self, 
                 startLink, # the cylindrical link immediately before the first affected xyz joint
                 endLink, # the cylindrical link immediately after the last affected xyz joint
                 numCables, # number of cables in the set
                 cables = None, # list of Cable objects contained in the set (optional, can be generated using generateCables())
                 segmentColor = [0.5,0.5,0.5,1.0] # what RGBA color to update affected links with
                 ):
        # link coordinates use [-1, 0, ..., numJoints-1]
        if startLink == -1:
            self.startLink = -1
        else:
            self.startLink = 3 * startLink
        self.endLink = 3*int(endLink)
        self.numCables = int(numCables)
        self.cables = cables
        self.segmentColor = segmentColor
        self.sliders = []
    
    
    # generate positions for each cable based on equal spacing around a circle
    def generateCables(self, 
                       radius, # xy distance from the cables to the center of each link
                       startAngle, # initial radian angle of cable 0 
                       linkLength, # length of manipulator links
                       twist # radian angle offset of cables between links
                       ):
        self.cables = []
        angleInc = 2 * math.pi / self.numCables
        for c in range(self.numCables):
            positions = []
            for p in range(self.endLink+1-self.startLink):
                linkpos = [radius * math.cos(startAngle + c * angleInc + p * twist),
                           radius * math.sin(startAngle + c * angleInc + p * twist),
                           linkLength
                           ]
                positions.append(linkpos)
            self.cables.append(Cable(self.endLink-self.startLink,positions,0.0, [0.0,0.0,0.0]))

    # generate pybullet sliders for debugging
    def generateDebugSliders(self, 
                        maxTension # maximum value of the slider
                        ):
        if(len(self.sliders)==0):
            for j in range(self.numCables):
                sliderText = "CABLE " + str(j) + " TENSION"
                self.sliders.append(p.addUserDebugParameter(sliderText, 0, maxTension, 0.0))
        else:
            print("There are already sliders")
        return self.sliders

    # automatically read tensions from sliders
    def readDebugSliders(self):
        if(len(self.sliders)>0):
            for j in range(self.numCables):
                self.cables[j].tension = p.readUserDebugParameter(self.sliders[j])
        else:
            print("No sliders to read")

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# (WORK IN PROGRESS)
class ActuatorMotor:
    # model of a rotary motor actuator, used to apply tension to cables with a torque
    def __init__(self, 
                manipulator, # associated ContinuumManipulator object
                cable, # cable affected by this actuator
                maxTorque, # maximum torque of the motor 
                wheelRadius, # radius of pulley wheel
                pidCoeff = [1.0,1.0,1.0] # optional PID coefficients (default work)
                ):
        self.manipulator = manipulator
        self.cable = cable
        self.maxTorque = maxTorque 
        self.wheelRadius = wheelRadius 
        self.pidCoeffs = pidCoeff
        self.actuationTorque = 0.0
        self.target = 0.0
        self.pid = [0.0,0.0,0.0]
        self.lastP = 0.0
        self.controlmode = p.TORQUE_CONTROL

    # outputs the amount of force for a given torque input, maxes out at the given max torque. Uses pybullet flags p.TORQUE_CONTROL and p.POSITION_CONTROL
    def setMotorControl(self, 
                        controlMode,
                        torque, # torque at which to drive motor (toruqe mode) or maximum allowed torque (pos/vel modes)
                        targetLength = None # target cable length (pos mode) 
                        ):
        
        if(controlMode == p.TORQUE_CONTROL):
            self.actuationTorque = torque
            self.target = None
            self.controlmode=p.TORQUE_CONTROL
        elif(controlMode == p.POSITION_CONTROL):
            if(targetLength == None):
                print("Target length not specified")
            self.target = targetLength
            self.controlmode=p.POSITION_CONTROL
            print("Positon control mode not implemented")
        else:
            print("Control mode not recognized, use p.TORQUE_CONTROL or p.POSITION_CONTROL")

    def stepSimulation(self):
        
        # Need to remember how to implement a digital PID controller
        # I dont really know how right now
        # This is supposed to model a servo pulling the cable to a given target angle, but this has to do that manually
        # This also doesnt seem necessary right now but idk
        
        if(self.controlmode == p.TORQUE_CONTROL):
            self.cable.tension = min(self.actuationTorque,self.maxTorque) / self.wheelRadius
        elif(self.controlmode == p.POSITION_CONTROL):
            self.pid[0] = self.target - self.cable.length
            self.pid[1] += self.pid[0]
            self.pid[2] = self.pid[0] - self.lastP

            self.cable.tension = min(self.pidCoeffs[0] * self.pid[0] + self.pidCoeffs[1] * self.pid[1] + self.pidCoeffs[2] * self.pid[2], self.maxTorque) / self.wheelRadius
            self.lastP = self.pid[0]
            
# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# (WORK IN PROGRESS)
class ActuatorPneumatic:
    # model of a pneumatic linear motor actuator, used to apply tension to cables with an angled force
    def __init__(self, 
                 manipulator, # associated ContinuumManipulator object
                 cable, # cable affected by this actuator
                 actuationArea, # area over which pressure is applied (m^2)
                 minPressure, # minimum model pressure (kPa)
                 maxPressure, # maximum model pressure (kPa)
                 minLength, # minimum model length (m)
                 maxLength # maximum model length (m)
                 ):
        self.manipulator = manipulator
        self.cable = cable
        self.actuationArea = actuationArea 
        self.minPressure = minPressure
        self.maxPressure = maxPressure 
        self.minLength = minLength 
        self.maxLength = maxLength 

        self.length = 0 # current expansion of the actuator, a function of current pressure and cable tension/length
        self.actuationPressure = minPressure # initial target pressure of the actuator
        


    def setMotorControl(self,
                        pressure # target actuation pressure
                        ):
        if(self.minPressure < pressure < self.maxPressure):
            self.actuationPressure = pressure
        elif(self.minPressure >= pressure):
            self.actuationPressure = self.minPressure
        elif(self.maxPressure >= pressure):
            self.actuationPressure = self.maxPressure


    def stepSimulation(self):
        # set the tension to a function of all current variables (acutator length, cable length, pressure)
        # Use a geometric model of the actuator to find the force from the given pressure and actuator state
        # There is some sort of triangle/trig function that would work
        # This would be highly dependent on actuator geometry though, it may be better to use an input for a custom function or something
        # Could just start with example origami actuator and work on generalizing it later
        cableLength = self.cable.length
        force = self.actuationArea*self.actuationPressure 
        tension = force * cableLength # This isnt a final function, needs to be replaced
        self.cable.tension = force
    
# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

class ContinuumManipulator: 
    # model of a continuum manipulator using spring oscillator forces and cable actuators
    def __init__(self, 
                modelId, # pybullet object ID of manipulator body
                cableSets, # list of all cable sets in manipulator
                springConstant, # hookean spring constant of the manipulator, determine from real model
                linkLength, # length of links that represent the body of the manipulator
                doSelfCollision = True, # enables self-collision, requires urdf flag p.URDF_USE_SELF_COLLISION
                maxLinkVelocity = 5.0, # maximum link velocity (fixes physics bugs, defaults to 5.0)
                twistConstant = 1.0 # spring constant on the z-axis rotation (twisting motion), can be specified if model has z-axis capabiltiies
                ):
        self.modelId = modelId 
        self.cableSets = cableSets 

        self.springConstant = springConstant
        self.linkLength = linkLength
        self.doSelfCollision = doSelfCollision 
        self.maxLinkVelocity = maxLinkVelocity

        self.drawnCables = [] # list of all the cable objects drawn

        self.twistConstant = twistConstant 


        self.numJoints = p.getNumJoints(self.modelId) # total number of joints in manipulator
        self.jointNumbers = numpy.arange(self.numJoints) # vector counting up from [0,...,numJoints-1]
        self.jointStates = p.getJointStates(self.modelId, self.jointNumbers) # vector of all joint states, in order
        self.linkStates = []
        self.jointForces = numpy.zeros(self.numJoints) # vector of all joint forces, in order
        self.jointTargets = numpy.zeros(self.numJoints)
        self.jointActuatorForces = numpy.zeros(self.numJoints)
        self.jointSegments=[]

        for j in range(self.numJoints):
            self.jointSegments.append([0])
        

        self.linkStates = [p.getBasePositionAndOrientation(self.modelId)]
        self.linkStates += p.getLinkStates(self.modelId, linkIndices=self.jointNumbers)

        self.updateSegments()

        # for s in self.cableSets:
        #    self.cableSetNetTorques.append(s.getNetLinkTorque())

        p.setJointMotorControlArray(self.modelId, jointIndices=self.jointNumbers, controlMode=p.POSITION_CONTROL, forces=self.jointForces) # set the maximum force at each joint to zero
        # p.setJointMotorControlArray(self.modelId, jointIndices=self.jointNumbers, controlMode=p.VELOCITY_CONTROL, forces=self.jointForces) # set the maximum force at each joint to zero

        if(self.doSelfCollision): # handles self collision link pair exclusion to nearby links, avoiding acutation and motion issues
            for linkA in range(-1,self.numJoints):
                for linkB in range(-1,self.numJoints):
                    if(linkA != linkB):
                        if((abs(linkB-linkA)<= 5)):
                            # print(str(linkA), " \t+\t", str(linkB), "\t:\t", str(abs(linkB-linkA)<= 4))
                            p.setCollisionFilterPair(bodyUniqueIdA=self.modelId, bodyUniqueIdB=self.modelId, linkIndexA=linkA, linkIndexB=linkB, enableCollision = 0)


    # updates all the joint forces with physics model and applied cable tensions
    def stepSimulation(self): 
        self.jointStates = p.getJointStates(self.modelId, self.jointNumbers)
        self.linkStates = [p.getBasePositionAndOrientation(self.modelId)]
        self.linkStates += p.getLinkStates(self.modelId, linkIndices=self.jointNumbers)

        for s in self.cableSets:
            for c in s.cables:
                c.length = 0.0
                c.length += self.linkLength
        
        for j in range(self.numJoints):
            l = math.floor(float(j)/3)

            netJointCableTorque = [0.0,0.0,0.0]

            for i in self.jointSegments[j]:
                for cable in self.cableSets[i].cables:
                    
                    lastLinkPos = cable.positions[3*l-(self.cableSets[i].startLink+1)]
                    currentLinkPos = cable.positions[3*l+1-(self.cableSets[i].startLink+1)]

                    lastLinkPosRelative = applyRotation([lastLinkPos[0], lastLinkPos[1], 0.0],self.linkStates[l][1])
                    currentLinkPosRelative = applyRotation([currentLinkPos[0], currentLinkPos[1], -currentLinkPos[2]],self.linkStates[l+1][1])

                    forceVectorRelative = [lastLinkPosRelative[0]-currentLinkPosRelative[0],
                                            lastLinkPosRelative[1]-currentLinkPosRelative[1],
                                            lastLinkPosRelative[2]-currentLinkPosRelative[2]]
                    cable.length += magnitude(forceVectorRelative)/3
                    
                    # ??????
                    # Need to add a way to properly estimate cable length, should use some sort of curve algorithm
                    # Idk why this needs '/3' to work and it really isnt very accurate
                    
                    forceVector = normalize(applyRotation(forceVectorRelative,invertQuaternion(self.linkStates[l+1][1])))
                    forceVector = scaleVector(cable.tension,forceVector)
                    
                    torqueVector=numpy.cross(forceVector,currentLinkPos)
                    netJointCableTorque=numpy.add(netJointCableTorque,torqueVector)

            jointSpringTorque = [abs(self.jointStates[3*l][0] * self.springConstant),
                                 abs(self.jointStates[3*l+1][0] * self.springConstant),
                                 abs(self.jointStates[3*l+2][0] * self.springConstant)]            

            targetAxis = normalize(netJointCableTorque)
            targetAngle = -magnitude(netJointCableTorque)/self.springConstant
            
            targetEuler = jointEulerFromQuaternion(quaternionFromAxisAngle(targetAxis,targetAngle))

            self.jointTargets[j] = targetEuler[j-3*l]
            self.jointForces[j] = abs(jointSpringTorque[j-3*l]+netJointCableTorque[j-3*l])

        
        p.setJointMotorControlArray(self.modelId, self.jointNumbers, controlMode=p.POSITION_CONTROL, targetPositions=self.jointTargets, forces=self.jointForces)




    # updates the list of joint affecting segments
    def updateSegments(self):
        for j in range(self.numJoints):
            self.jointSegments[j].clear()
            for s in range(len(self.cableSets)):
                if(self.cableSets[s].startLink < j <= self.cableSets[s].endLink):
                    self.jointSegments[j].append(s)

    # add a new cable set to the manipulator
    def addNewCableSet(self, 
                       newCableSet
                       ):
        self.cableSets.append(newCableSet)
        self.cableSetNetTorques.clear()
        # for s in self.cableSets:
            # self.cableSetNetTorques.append(s.getNetLinkTorque)
    
    # set new spring constant
    def setSpringConstant(self, 
                          newSpringConstant
                          ): 
        self.springConstant = newSpringConstant

    # set new cable tensions for cable set cableSetIndex (directly sets the forces, for actuator function use updateCableForces)
    def setCableTensions(self, 
                         cableSetIndex, 
                         newCableTensions
                         ):
        for i in range(len(newCableTensions)):        
            self.cableSets[cableSetIndex].cables[i].tension = newCableTensions[i]

    # show the cables
    def showCables(self):
        if(len(self.drawnCables)==0):
            for s in self.cableSets:
                for cable in s.cables:
                    for l in range(s.startLink,s.endLink+1):
                        if((l+1)%3 == 0):
                            if(l==-1):
                                prevp = [cable.positions[l+1-(s.startLink+1)][0],
                                         cable.positions[l+1-(s.startLink+1)][1],
                                         cable.positions[l+1-(s.startLink+1)][2]-0.5*self.linkLength]
                            else:
                                prevp = [cable.positions[l-(s.startLink+1)][0],
                                         cable.positions[l-(s.startLink+1)][1],
                                         cable.positions[l-(s.startLink+1)][2]-0.5*self.linkLength]
                                
                            currp = [cable.positions[l+1-(s.startLink+1)][0],
                                     cable.positions[l+1-(s.startLink+1)][1],
                                     -cable.positions[l+1-(s.startLink+1)][2]+0.5*self.linkLength]
                            self.drawnCables.append(p.addUserDebugLine(lineFromXYZ=prevp,
                                                                    lineToXYZ=currp,
                                                                    lineColorRGB=cable.debugRenderColor,
                                                                    lineWidth=1,
                                                                    parentObjectUniqueId=self.modelId,
                                                                    parentLinkIndex=l))
        else:
            print("Cables are already shown")

    # hide the cables
    def hideCables(self):
        if(len(self.drawnCables)!=0):
            for d in self.drawnCables:
                p.removeUserDebugItem(d)
            self.drawnCables.clear()
        else:
            print("Cables are not currently shown")

    # show the segment colors
    def showSegmentColors(self):
        for c in self.cableSets: # color each set of segments differently
            for l in range(c.startLink, c.endLink+1):
                p.changeVisualShape(self.modelId, linkIndex=l, rgbaColor=c.segmentColor)

    # hide the segment colors
    def hideSegmentColors(self, 
                          modelColor=[0.5,0.5,0.5,1.0]
                          ):
        for l in range(-1, self.numJoints):
            p.changeVisualShape(self.modelId, linkIndex=l, rgbaColor=modelColor)
    
    # makes the model visible
    def showModel(self):
        self.hideSegmentColors()

    # makes the model invisible
    def hideModel(self):
        self.hideSegmentCOlors(modelColor=[1.0,1.0,1.0,0.0])

    # move a soft body using this setup? (p.createSoftBodyAnchor(int softBodyUniqueId, int nodeIndex, int bodyUniqueId, int linkIndex, const double bodyFramePosition[3]);
    def addSoftBody(self, 
                    softBodyID
                    ):
        # do something here idk
        print("addSoftBody not finished")

    # find the mass matrix of the manipulator in the current state
    def getMassMatrix(self):
        jointAngles=[]
        for j in range(self.numJoints):
                jointAngles.append(self.jointStates[j][0])
        return p.calculateMassMatrix(self.modelId,jointAngles)
    
# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# FUNCTIONS:


# quaternion left multiplication q0q1 = q2
def multiplyQuaternion(q0, # left quaternion [xyzw]
                       q1 # right quaternion [xyzw]
                       ):
    w0 = float(q0[3])
    x0 = float(q0[0])
    y0 = float(q0[1])
    z0 = float(q0[2])

    w1 = float(q1[3])
    x1 = float(q1[0])
    y1 = float(q1[1])
    z1 = float(q1[2])

    w2 = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1 
    x2 = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    y2 = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    z2 = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    q2 = [x2,y2,z2,w2]
    return q2

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def scaleVector(s,
              v0
              ):
    v1=[]
    for i in range(len(v0)):
        v1.append(s*v0[i])
    return v1

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def vectorAngle(v0,
                v1
                ):
    if(magnitude(v0) > 0.0 and magnitude(v1) > 0.0):
        v0n = normalize(v0)
        v1n = normalize(v1)
        # angle = math.acos((dotProd(v0n,v1n))/magnitude(v0)*magnitude(v1))
        angle = math.acos(numpy.dot(v0n,v1n))
    else:
        angle=0.0
    return angle

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# quaternnion inverting q'
def invertQuaternion(q):
    return [-q[0],-q[1],-q[2],q[3]]

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def magnitude(vec):
    mag = 0.0
    for v in vec:
        mag += pow(v,2)
    return pow(mag,0.5)

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def vecProject(v0, # vector to be projected [xyz]
               v1 # vector projected onto [xyz]
               ):
    if(math.isclose(magnitude(v0),0.0)):
        return [0.0,0.0,0.0]
    elif (math.isclose(magnitude(v1),0.0)):
        return [0.0,0.0,0.0]
    else:
        mag = numpy.dot(v0,v1)/magnitude(v1)
        return [mag*v0[0],mag*v0[1],mag*v0[2]]

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def normalize(v):
    vnorm = []
    if(magnitude(v)>0.0):
        mag = magnitude(v)
        for i in v:
            vnorm.append(i / mag)
        return vnorm
    else:
        return numpy.zeros(len(v))

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# given an axis and angle, produce a quaternion describing a rotation of angle about axis
def quaternionFromAxisAngle(axis, # axis of rotation [xyz]
                            angle # radians rotated
                            ):
    rad = angle/2
    quat0 = [math.sin(rad)*axis[0],math.sin(rad)*axis[1],math.sin(rad)*axis[2],math.cos(rad)]
    return quat0

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

def applyRotation(v, # vector to be rotated around the origin [xyz]
                  q # quaternion rotation to apply [xyzw]
                  ):
    qv = [v[0],v[1],v[2],0]
    qv = multiplyQuaternion(q,qv)
    qv = multiplyQuaternion(qv,invertQuaternion(q))
    return [qv[0],qv[1],qv[2]]

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# reverse of quaternionFromAxisAngle(), returns an axis of rotation and radian angle from a given quaternion
def axisAngleFromQuaternion(quat0):
    q = normalize(quat0)
    angle = math.acos(q[3])
    if(math.isclose(angle,0.0)):
        axis = [0.0,0.0,0.0]
    else:
        # scaleFac = 1 / pow(1-pow(q[3],2),0.5)
        # axis = [scaleFac * q[0], scaleFac * q[1], scaleFac * q[2]]
        axis = normalize([q[0],q[1],q[2]])
    return [axis,2.0*angle]

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# Based on https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def jointEulerFromQuaternion(q):
        
        # Convert a quaternion into euler angles (roll, pitch, yaw)
        # roll is rotation around x in radians (counterclockwise)
        # pitch is rotation around y in radians (counterclockwise)
        # yaw is rotation around z in radians (counterclockwise)
        
        x=q[0]
        y=q[1]
        z=q[2]
        w=q[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = -math.atan2(t3, t4) # I dont know why but negating the yaw fixes everything
     
        return [roll_x, pitch_y, yaw_z] # in radians

# -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------# 

# Generates a URDF file describing a continuum manipulator using combined xyz joints
def generateURDF(
                fileName,      # name of the URDF file, resulting file will be "filename.urdf"
                robotName,      # name of the robot as specified within the URDF file
                radius,         # radius of the manipulator body
                numLinks,       # number of moveable links (after the base link)
                length,     # length of each link
                mass,       # mass of each link
                automaticSegments = False, # Use length/mass for each link or for entire robot, divided evenly between links (false is for segments, true for entire robot)
                linkInertia = [0.0,0.0,0.0],    # inertia tensor of each link, in [ixx,iyy,izz] format (can be [0,0,0] if model will not use inertia from file)
                damping=0.1     # joint damping of each link, values around 0.05-0.15 work well (tune based on manipulator dynamics)
                ):
    
    if(automaticSegments):
        linkLength = length/(numLinks+1)
        linkMass = mass/(numLinks+1)
    elif(not automaticSegments):
        linkLength = length
        linkMass = mass
    
    name = fileName + ".urdf" # generate the file name with appended .urdf
    with open(name, 'w') as f: # open and clear the file (makes file if it does not already exist)
        initLine = '<robot name=\"' + robotName + '\">\n\n'  # start the urdf XML
        f.write(initLine)
        f.close()

    with open(name, 'a') as f: # reopen the created file in append mode to create the robot definition
        f.write('\t <link name=\"link_-1\">\n') # generates the base link special at xyz=0.0 0.0 0.0

        f.write('\t\t<inertial>\n')
        f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
        f.write(str(round(-0.5*linkLength,4)))
        # f.write('0.0')
        f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
        f.write('\t\t\t<mass value=\"')
        f.write(str(linkMass))
        f.write('\"/>\n')
        inertiaLine = '\t\t\t<inertia '+ 'ixx=\"' + str(linkInertia[0]) + '\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"' + str(linkInertia[1]) + '\" iyz=\"0.0\" izz=\"' +str(linkInertia[2]) + '\"/>\n'
        f.write(inertiaLine)
        f.write('\t\t</inertial>\n')

        f.write('\t\t<visual name=\"\">\n')
        f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
        f.write(str(round(-0.5*linkLength,4)))
        f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
        f.write('\t\t\t<geometry>\n')
        f.write('\t\t\t\t<cylinder radius=\"')
        f.write(str(radius))
        f.write('\" length=\"')
        f.write(str(linkLength))
        f.write('\"/>\n')
        f.write('\t\t\t</geometry>\n')
        f.write('\t\t\t<material name=\"\">\n')
        f.write('\t\t\t\t<color rgba=\"0.6 0.6 0.6 1.0\"/>\n')
        f.write('\t\t\t\t<texture filename=\"\"/>\n')
        f.write('\t\t\t</material>\n')
        f.write('\t\t</visual>\n')

        f.write('\t\t<collision>\n')
        f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
        f.write(str(round(-0.5*linkLength,4)))
        f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
        f.write('\t\t\t<geometry>\n')
        f.write('\t\t\t\t<cylinder radius=\"')
        f.write(str(radius))
        f.write('\" length=\"')
        f.write(str(linkLength))
        f.write('\"/>\n')
        f.write('\t\t\t</geometry>\n')
        f.write('\t\t</collision>\n')

        f.write('\t</link>\n\n')

        axes = ['x','y','z']
        axisvecs = ['1.0 0.0 0.0', '0.0 1.0 0.0', '0.0 0.0 1.0']

        for l in range(numLinks): # generates all the remaining links and calculates position values automatically
            for j in range(2):
                f.write('\t<link name=\"link_')
                f.write(str(l))
                f.write('_')
                f.write(axes[j])
                f.write('\">\n')

                f.write('\t\t<inertial>\n')
                f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
                f.write('0.0')
                f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')

                f.write('\t\t\t<mass value=\"0.00001\"/>\n')
                f.write('\t\t\t<inertia ixx=\"0.0000001\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"0.0000001\" iyz=\"0.0\" izz=\"0.0000001\"/>\n')
                f.write('\t\t</inertial>\n')

                f.write('\t\t<visual name=\"\">\n')
                f.write('\t\t\t<origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\"/>\n')
                f.write('\t\t\t<geometry>\n')
                f.write('\t\t\t\t<sphere radius=\"')
                f.write(str(radius))
                f.write('\"/>\n')
                f.write('\t\t\t</geometry>\n')
                f.write('\t\t\t<material name=\"\">\n')
                f.write('\t\t\t\t<color rgba=\"0.6 0.6 0.6 1.0\"/>\n')
                f.write('\t\t\t\t<texture filename=\"\"/>\n')
                f.write('\t\t\t</material>\n')
                f.write('\t\t</visual>\n')

                f.write('\t\t<collision>\n')
                f.write('\t\t\t<origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\"/>\n')
                f.write('\t\t\t<geometry>\n')
                f.write('\t\t\t\t<sphere radius=\"')
                f.write(str(radius))
                f.write('\"/>\n')
                f.write('\t\t\t</geometry>\n')
                f.write('\t\t</collision>\n')

                f.write('\t</link>\n\n')



            f.write('\t<link name=\"link_')
            f.write(str(l))
            f.write('\">\n')

            f.write('\t\t<inertial>\n')
            f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
            f.write(str(round(-0.5*linkLength,4)))
            # f.write('0.0')
            f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
            f.write('\t\t\t<mass value=\"')
            f.write(str(linkMass))
            f.write('\"/>\n')
            inertiaLine = '\t\t\t<inertia '+ 'ixx=\"' + str(linkInertia[0]) + '\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"' + str(linkInertia[1]) + '\" iyz=\"0.0\" izz=\"' +str(linkInertia[2]) + '\"/>\n'
            f.write(inertiaLine)
            f.write('\t\t</inertial>\n')

            f.write('\t\t<visual name=\"\">\n')
            f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
            f.write(str(round(-0.5*linkLength,4)))
            f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
            f.write('\t\t\t<geometry>\n')
            f.write('\t\t\t\t<cylinder radius=\"')
            f.write(str(radius))
            f.write('\" length=\"')
            f.write(str(linkLength))
            f.write('\"/>\n')
            f.write('\t\t\t</geometry>\n')
            f.write('\t\t\t<material name=\"\">\n')
            f.write('\t\t\t\t<color rgba=\"0.6 0.6 0.6 1.0\"/>\n')
            f.write('\t\t\t\t<texture filename=\"\"/>\n')
            f.write('\t\t\t</material>\n')
            f.write('\t\t</visual>\n')

            f.write('\t\t<collision>\n')
            f.write('\t\t\t<origin xyz=\"0.0 0.0 ')
            f.write(str(round(-0.5*linkLength,4)))
            f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')
            f.write('\t\t\t<geometry>\n')
            f.write('\t\t\t\t<cylinder radius=\"')
            f.write(str(radius))
            f.write('\" length=\"')
            f.write(str(linkLength))
            f.write('\"/>\n')
            f.write('\t\t\t</geometry>\n')
            f.write('\t\t</collision>\n')
            f.write('\t</link>\n\n')

        for j in range(numLinks): # generates all joints and calculates positions automatically
            
            for i in range(3):
                f.write('\t<joint name=\"joint_')
                f.write(str(j))
                f.write('_')
                f.write(axes[i])
                f.write('\" type=\"')
                if(i<2):
                    f.write('revolute')
                else:
                    f.write('continuous') 
                f.write('\">\n')

                f.write('\t\t<origin xyz=\"0.0 0.0 ')
                if(i<1):
                    f.write(str(round(-1*linkLength,4)))
                else:
                    f.write('0.0')
                f.write('\" rpy=\"0.0 0.0 0.0\"/>\n')

                f.write('\t\t<parent link=\"link_')
                if(i==0):
                    f.write(str(j-1))
                else:
                    f.write(str(j))
                    f.write('_')
                    f.write(axes[i-1])
                f.write('\"/>\n')

                f.write('\t\t<child link=\"link_')
                f.write(str(j))
                if(i<2):
                    f.write('_')
                    f.write(axes[i])
                f.write('\"/>\n')


                f.write('\t\t<axis xyz=\"')
                f.write(axisvecs[i])
                f.write('\"/>\n')
                if(i<2):
                    f.write('\t\t<limit lower=\"-1.57\" upper=\"1.57\" effort=\"1000\" velocity=\"20\"/>\n')
                f.write('\t\t<dynamics damping=\"')
                f.write(str(damping))
                # f.write('\" friction=\"0.04\"/>\n')
                f.write('\"/>\n')
                f.write('\t</joint>\n\n')

        f.write('</robot>') # finish the urdf XML and close the file
        f.close()