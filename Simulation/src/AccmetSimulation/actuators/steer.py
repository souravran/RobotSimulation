import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.core import blenderapi
from morse.helpers.components import add_data, add_property
from morse.core import mathutils
from morse.builder import bpymorse
import math
import bisect


class Steer(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Steer"
    _short_desc = "move the wagon on the track"


    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)
        self._target_count = 0 # dummy internal variable, for testing purposes
        logger.info('Component initialized')
        self.scene                  = blenderapi.scene()
        self.wagon                  = self.scene.objects[self.robot_parent.name()]
        self.nav                    = 0
        self.navCellPoseX           = -73.31683 ## assigned with start cell X-Coordinate
        self.navCellPoseY           = -54.60022 ## assigned with start cell Y-Coordinate 
        self.navCellPoseZ           = 15.73236 ## assigned with the Z-Coordinate of the navigation plane, it is always constant
        self.crossProductThreshold  = 0.26 ## threshold value for considering the right number of wagon pose in cross product
        self.initFlag               = False ## used for setting the robot to stop motion once at the start cell [0,0] during initialization
        self.switchFlag             = False
        self.lrSwitch               = False
        self.naviMeshCellToGridCell = 2.234  ## conversion factor for changing navigation mesh cell length to grid cell length, at application level grid cells are being used
        self.targetLengthStore      = 0.0
        self.currentRelDist         = 1.0 ## current and lastRelDist is needed to identify if there is a direction change in order to activate navCompStore  
        self.lastRelDist            = 1.0
        self.robotDirectionChanged  = False
        self.navStore               = {}
        self.navCompeStore          = {}
        self.InitNavMesh()
        self.ComplementaryNavMesh()


    ## from rail coordinate, setting the target position in blender coordinate in (x,y,z)
    def SetTargetBlenderCoordinate(self, relativeDist):
        previousCell    = None
        targetCell      = None
        cellLength      = 0
        lastCellLength  = 0
        lengthFactor    = 0
        navCellPoseX    = 0
        navCellPoseY    = 0

        #targetLength = self.GetWagonRailCoordinate() + (relativeDist)
        if (self.robotDirectionChanged  and (not self.switchFlag)):
        	name = self.navCompeStore[self.nav.name][0]
        	self.nav = bpymorse.get_object(name)
        absRelativeDist = abs(relativeDist)
        #self.GetWagonRailCoordinateModified()	
        #targetLength = self.targetLengthStore + absRelativeDist
        targetLength = self.GetWagonRailCoordinateModified() + absRelativeDist
        #targetLength = self.GetWagonRailCoordinate() + (absRelativeDist)
        #targetLength = 5.0
        print("\n[target distance :",targetLength,"]")
        print("[relative distance :",absRelativeDist,"]\n", self.targetLengthStore, self.nav.name)
        shortNavMesh = True
        
        for poly in self.nav.data.polygons:
            nextCellLocalCoord = mathutils.Vector(poly.center)
            nextCell = self.nav.matrix_world * nextCellLocalCoord
            print("Next Cell ", nextCell[0], nextCell[1])
            if previousCell:
                targetCell = nextCell
                
                previousCell -= nextCell
                cellLength = cellLength + previousCell.length
                #print("Cell Lengths : ", cellLength, previousCell.length," \n")
                if (cellLength > targetLength) and (not self.lrSwitch):
                	shortNavMesh = False
                	lastCellLength = previousCell.length
                	navCellPoseX = float("{0:.4f}".format(targetCell[0]))
                	navCellPoseY = float("{0:.4f}".format(targetCell[1]))
                	cellLength = cellLength - lastCellLength
                	break
            previousCell = nextCell
        if shortNavMesh:
        	if self.lrSwitch: # this flag is make sure that the target is placed at the end of all the switch nav mesh
        		navCellPoseX = float("{0:.4f}".format(targetCell[0]))
        		navCellPoseY = float("{0:.4f}".format(targetCell[1]))
        	else:
        		navCellPoseX = float("{0:.4f}".format(targetCell[0]))
        		navCellPoseY = float("{0:.4f}".format(targetCell[1]))
        	print(" The nav mesh was short, setting the target at the end of mesh !!")

        #print("Relative distance ", navCellPoseX, navCellPoseY)
        ## calculate the offset distance, which is the distance between the last cell and target length
        self.targetLengthStore = self.targetLengthStore + (absRelativeDist)
        offsetLength = targetLength - cellLength
        self.switchFlag = False
        self.lrSwitch = False
        if lastCellLength != 0:
            lengthFactor = offsetLength/lastCellLength

        navCelltargetPoseX = ((1 - lengthFactor) * navCellPoseX) + (lengthFactor * nextCell[0])
        navCelltargetPoseY = ((1 - lengthFactor) * navCellPoseY) + (lengthFactor * nextCell[1]) 
        if relativeDist > 0:
            self.wagon.actuators['GoForward'].target.position = [navCellPoseX, navCellPoseY, self.navCellPoseZ]
            #print("Relative distance has been set", navCellPoseX, navCellPoseY)
        else:
            self.wagon.actuators['GoBackward'].target.position = [navCellPoseX, navCellPoseY, self.navCellPoseZ]

    def GetWagonRailCoordinateModified(self):
    	cellLength      = 0
    	currentPose     = self.GetWagonBlenderCoordinate()
    	currentPoseX    = float("{0:.3f}".format(currentPose[0]))
    	currentPoseY    = float("{0:.3f}".format(currentPose[1]))
    	distDict        = [(100000, 1000000)]
    	for poly in self.nav.data.polygons:
    		nextCellLocalCoord = mathutils.Vector(poly.center)
    		nextCell = self.nav.matrix_world * nextCellLocalCoord
    		linearDist = math.hypot(currentPose[0] - nextCell[0], currentPose[1] - nextCell[1])
    		cellLength = cellLength + 0.626
    		bisect.insort_right(distDict, (linearDist, cellLength))
    	print(distDict[0][1], self.nav.name)
    	return distDict[0][1]


    ## getting the rail coordinate, which is the absolute distance of wagon from the start of the track
    def GetWagonRailCoordinate(self):
        previousCell    = None
        cellLength      = 0
        currentPose     = self.GetWagonBlenderCoordinate()
        currentPoseX    = float("{0:.3f}".format(currentPose[0]))
        currentPoseY    = float("{0:.3f}".format(currentPose[1]))

        print("Current Pose and nav name : ", currentPoseX, currentPoseY, self.nav.name) 

        for poly in self.nav.data.polygons:
            nextCellLocalCoord = mathutils.Vector(poly.center)
            nextCell = self.nav.matrix_world * nextCellLocalCoord
            #print("Nav Poly Pose ", float("{0:.3f}".format(nextCell[0])), float("{0:.3f}".format(nextCell[1])))         
            if previousCell:
                startCellX      = float("{0:.3f}".format(previousCell[0]))
                startCellY      = float("{0:.3f}".format(previousCell[1]))
                endCellX        = float("{0:.3f}".format(nextCell[0]))
                endCellY        = float("{0:.3f}".format(nextCell[1]))

                diffPoseX       = float("{0:.3f}".format(currentPoseX - startCellX))
                diffPoseY       = float("{0:.3f}".format(currentPoseY - startCellY))
                diffCellX       = float("{0:.3f}".format(endCellX - startCellX))
                diffCellY       = float("{0:.3f}".format(endCellY - startCellY))
                crossProduct    = diffPoseX * diffCellY - diffPoseY * diffCellX
                #crossProduct    = abs(crossProduct)
                #print("   Start and end cell  :", startCellX, startCellY, endCellX, endCellY)
                #print("Cross Product, diff pose, diff cell length : ", crossProduct, diffPoseX, diffPoseY, diffCellX, diffCellY)
                if crossProduct == 0 or crossProduct < self.crossProductThreshold :
                    #print("Diff cell X and Y : ", diffCellX, diffCellY)
                    if (abs(diffCellX) >= abs(diffCellY)):
                    	if diffCellX > 0:
                    		if (startCellX <= currentPoseX and currentPoseX <= endCellX) or (endCellX <= currentPoseX and currentPoseX <= startCellX):
                    		    #print("Break Cell X", startCellX, endCellX)
                    		    break;
                    else:
                    	if diffCellY > 0:
                    		if (startCellY <= currentPoseY and currentPoseY <= endCellY) or (endCellY <= currentPoseY and currentPoseY <= endCellY):
                    		    #print("Break cell Y", startCellY, endCellY)
                    		    break;
                previousCell -= nextCell
                ## each Cell length in the Navigation-Mesh is 0.6265316078016844 with resepect to the current simulation scene scale  
                cellLength = cellLength + previousCell.length
                #print("Calculated cell length :", cellLength, previousCell.length)
            previousCell = nextCell

        #print("Current Length", cellLength)

        ## now calculate the distance between the center of last cell till the current pose
        #print("Previous Cell", previousCell[0], previousCell[1])
        #print("Previous cell Length before", previousCell.length)
        previousCell -= currentPose
        #print("New previous cell", previousCell[0], previousCell[1])
        #print("Previous cell Length after", previousCell.length)
        cellLength = cellLength + previousCell.length
        return cellLength


    ## get the wagon position from blender in (x,y,z) 
    def GetWagonBlenderCoordinate(self):
        return self.wagon.position

    def ComplementaryNavMesh(self):
    	self.navCompeStore["route_1_2"]    = ["route_2_1"]
    	self.navCompeStore["route_2_1"]    = ["route_1_2"]
    	self.navCompeStore["route_2_9"]    = ["route_9_2"]
    	self.navCompeStore["route_9_2"]    = ["route_2_9"]
    	self.navCompeStore["route_3_9"]    = ["route_9_3"]
    	self.navCompeStore["route_9_3"]    = ["route_3_9"]
    	self.navCompeStore["route_2_4"]    = ["route_4_2"]
    	self.navCompeStore["route_4_2"]    = ["route_2_4"]
    	self.navCompeStore["route_3_4"]    = ["route_4_3"]
    	self.navCompeStore["route_4_3"]    = ["route_3_4"]
    	self.navCompeStore["route_3_8"]    = ["route_8_3"]
    	self.navCompeStore["route_8_3"]    = ["route_3_8"]
    	self.navCompeStore["route_8_6"]    = ["route_6_8"]
    	self.navCompeStore["route_6_8"]    = ["route_8_6"]
    	self.navCompeStore["route_1_5"]    = ["route_5_1"]
    	self.navCompeStore["route_5_1"]    = ["route_1_5"]
    	self.navCompeStore["route_4_6"]    = ["route_6_4"]
    	self.navCompeStore["route_6_4"]    = ["route_4_6"]
    	self.navCompeStore["route_5_7"]    = ["route_7_5"]
    	self.navCompeStore["route_7_5"]    = ["route_5_7"]
    	self.navCompeStore["route_6_7"]    = ["route_7_6"]
    	self.navCompeStore["route_7_6"]    = ["route_6_7"]
    	self.navCompeStore["route_4_5"]    = ["route_5_4"]
    	self.navCompeStore["route_5_4"]    = ["route_4_5"]


    def InitNavMesh(self):
    	self.navStore["route_init"]  = ["route_1_mXY", "", "route_1_mXX"]
    	self.navStore["route_1_2"]   = ["", "route_2_mYX", "route_2_mYY"]
    	self.navStore["route_1_5"]   = ["route_5_mXY", "", "route_5_mXX"]
    	self.navStore["route_2_1"]   = ["route_1_YX", "route_1_YmX", ""]
    	self.navStore["route_2_4"]   = ["route_4_mXY", "route_4_mXmY", "route_4_mXX"]
    	self.navStore["route_2_9"]   = ["", "route_9_mYX", ""]
    	self.navStore["route_3_9"]   = ["route_9_XmY", "", ""]
    	self.navStore["route_3_8"]   = ["", "route_8_mXmY", ""]
    	self.navStore["route_3_4"]   = ["route_4_YX", "route_4_YmX", "route_4_YmY"]
    	self.navStore["route_4_2"]   = ["route_2_XmY", "route_2_XY", ""]
    	self.navStore["route_4_5"]   = ["route_5_YX", "route_5_YmX", ""]
    	self.navStore["route_4_6"]   = ["route_6_mXY", "route_6_mXmY", ""]
    	self.navStore["route_4_3"]   = ["route_3_mYmX", "route_3_mYX", ""]
    	self.navStore["route_5_1"]   = ["", "route_1_XY", "route_1_XmX"]
    	self.navStore["route_5_4"]   = ["route_4_mYmX", "route_4_mYX", "route_4_mYY"]
    	self.navStore["route_5_7"]   = ["route_7_mXY", "", ""]
    	self.navStore["route_7_5"]   = ["", "route_5_XY", "route_5_XmX"]
    	self.navStore["route_7_6"]   = ["route_6_mYmX", "", "route_6_mYY"]
    	self.navStore["route_6_4"]   = ["route_4_XmY", "route_4_XY", "route_4_XmX"]
    	self.navStore["route_8_3"]   = ["route_3_XmY", "", "route_3_XmX"]
    	self.navStore["route_6_7"]   = ["", "route_7_YmX", ""]
    	self.navStore["route_6_8"]   = ["route_8_mYmX", "", ""]
    	self.navStore["route_8_6"]   = ["", "route_6_YmX", "route_6_YmY"]
    	self.navStore["route_9_2"]   = ["route_2_YX", "", "route_2_YmY"]
    	self.navStore["route_9_3"]   = ["", "route_3_mXmY", "route_3_mXX"]
    	self.navStore["route_1_mXX"] = ["", "", "route_1_5"]
    	self.navStore["route_1_mXY"] = ["", "", "route_1_2"]
    	self.navStore["route_1_YmX"] = ["", "", "route_init"]
    	self.navStore["route_1_XmX"] = ["", "", "route_init"]
    	self.navStore["route_1_XY"]  = ["", "", "route_1_2"]
    	self.navStore["route_1_YX"]  = ["", "", "route_1_5"]
    	self.navStore["route_2_mYY"] = ["", "", "route_2_9"]
    	self.navStore["route_2_YmY"] = ["", "", "route_2_1"]
    	self.navStore["route_2_mYX"] = ["", "", "route_2_4"]
    	self.navStore["route_2_XmY"] = ["", "", "route_2_1"]
    	self.navStore["route_2_YX"]  = ["", "", "route_2_4"]
    	self.navStore["route_2_XY"]  = ["", "", "route_2_9"]
    	self.navStore["route_3_mXX"] = ["", "", "route_3_8"]
    	self.navStore["route_3_XmX"] = ["", "", "route_3_9"]
    	self.navStore["route_3_mXmY"] = ["", "", "route_3_4"]
    	self.navStore["route_3_mYmX"] = ["", "", "route_3_9"]
    	self.navStore["route_3_mYX"] = ["", "", "route_3_8"]
    	self.navStore["route_3_XmY"] = ["", "", "route_3_4"]
    	self.navStore["route_4_YmY"] = ["", "", "route_4_6"]
    	self.navStore["route_4_mYY"] = ["", "", "route_4_3"]
    	self.navStore["route_4_mXX"] = ["", "", "route_4_6"]
    	self.navStore["route_4_XmX"] = ["", "", "route_4_2"]
    	self.navStore["route_4_mXmY"] = ["", "", "route_4_5"]
    	self.navStore["route_4_mYmX"] = ["", "", "route_4_2"]
    	self.navStore["route_4_mXY"] = ["", "", "route_4_3"]
    	self.navStore["route_4_YmX"] = ["", "", "route_4_2"]
    	self.navStore["route_4_YX"] = ["", "", "route_4_6"]
    	self.navStore["route_4_XY"] = ["", "", "route_4_3"]
    	self.navStore["route_4_XmY"] = ["", "", "route_4_5"]
    	self.navStore["route_4_mYX"] = ["", "", "route_4_6"]
    	self.navStore["route_5_mXX"] = ["", "", "route_5_7"]
    	self.navStore["route_5_XmX"] = ["", "", "route_5_1"]
    	self.navStore["route_5_mXY"] = ["", "", "route_5_4"]
    	self.navStore["route_5_YmX"] = ["", "", "route_5_1"]
    	self.navStore["route_5_YX"] = ["", "", "route_5_7"]
    	self.navStore["route_5_XY"] = ["", "", "route_5_4"]
    	self.navStore["route_6_mYY"] = ["", "", "route_6_8"]
    	self.navStore["route_6_YmY"] = ["", "", "route_6_7"]
    	self.navStore["route_6_mYmX"] = ["", "", "route_6_4"]
    	self.navStore["route_6_mXmY"] = ["", "", "route_6_7"]
    	self.navStore["route_6_mXY"] = ["", "", "route_6_8"]
    	self.navStore["route_6_YmX"] = ["", "", "route_6_4"]
    	self.navStore["route_7_mXY"] = ["", "", "route_7_6"]
    	self.navStore["route_7_YmX"] = ["", "", "route_7_5"]
    	self.navStore["route_8_mXmY"] = ["", "", "route_8_6"]
    	self.navStore["route_8_mYmX"] = ["", "", "route_8_3"]
    	self.navStore["route_9_mYX"] = ["", "", "route_9_3"]
    	self.navStore["route_9_XmY"] = ["", "", "route_9_2"]

    	




    def SetInitNavMesh(self):
    	self.nav = bpymorse.get_object("route_init")


    def SwitchNavMesh(self, dir):
    	navNameStr = self.nav.name
    	self.lrSwitch = False
    	if navNameStr:
    		if (dir == 1):
    			name = self.navStore[navNameStr][0]
    			if name:
    				self.nav = bpymorse.get_object(name)
    				self.wagon.actuators['GoForward'].navmesh = name
    				self.wagon.actuators['GoBackward'].navmesh = name
    				#return 1
    			#else:
    				#return 0
    		if (dir == 2):
    			name = self.navStore[navNameStr][1]
    			if name:
    				self.nav = bpymorse.get_object(name)
    				self.wagon.actuators['GoForward'].navmesh = name
    				self.wagon.actuators['GoBackward'].navmesh = name
    				#return 1
    			#else:
    				#return 0
    		if (dir == 3):
    			name = self.navStore[navNameStr][2]
    			if name:
    				self.nav = bpymorse.get_object(name)
    				self.wagon.actuators['GoForward'].navmesh = name
    				self.wagon.actuators['GoBackward'].navmesh = name
    				#return 1
    			#else:
    				#return 0
    	#else:
    		#return 0



    ## setting the game property to finally make the wagon move
    def Move(self, relativeDist, velocity):
        self.Stop()
        if relativeDist > 0:
            self.wagon['GamePropForward'] = 1
        else:
            self.wagon['GamePropBackward'] = 1



##########################################   Steer Module Services   ######################################################
    @service
    def MoveRelative(self, relativeDist, velocity):
        """ This service is for making the wagon move relative 
            distance from wagon current position with the given 
            velocity.
        """
        self.initFlag = False # setting the init falg to false because after Init only Move can be fired, not setting this in trackpose because once the wagon is in start location it should keep reading 0 for TrackPose
        #self.SetNavMesh(2)
        self.currentRelDist = relativeDist
        if (relativeDist == 1.5) or (relativeDist == -1.5):
        	self.lrSwitch = True
        self.relativeDist = relativeDist*7.2
        if (self.currentRelDist > 0) and (self.lastRelDist > 0):
        	self.robotDirectionChanged = False;
        elif (self.currentRelDist < 0) and (self.lastRelDist < 0):
        	self.robotDirectionChanged = False;
        else:
        	self.robotDirectionChanged = True;
        self.lastRelDist = self.currentRelDist
        self.SetTargetBlenderCoordinate(self.relativeDist)
        self.Move(self.relativeDist, velocity)


    @service
    def Initialize(self):
        """ This service is for making the wagon to initialize at 
            the start of the track.
        """
        self.SetInitNavMesh()
        self.navCellPoseX = -73.31683 ## assigning with the start cell X-Coordinate
        self.navCellPoseY = -54.60022 ## assigning with the start cell Y-Coordinate
        self.Stop()
        self.initFlag = True
        self.wagon.actuators['GoForward'].target.position = [-73.31683, -54.60022, self.navCellPoseZ]
        self.Move(1,0) ## for wagon initialization, Move() does not need relative distance and velocity


    @service
    def TrackPose(self):
        """ This service is for tracking the wagon rail-coordinate.
            This will return a double value of the rail-coordinate.
        """
        poseValue = 0.0
        ## sometime even if the robot gets to start location [0,0], a slightest movement backward gives the erroneous rail-coordinate
        ## manually sending a stop command once the robot on the [0,0] location, which will ideally stop the robot at start reference location
        #if ((math.floor(poseValue) == 0) and (self.initFlag == True)):
        if (self.initFlag == True):
        	#self.Stop()
        	poseValue = self.GetWagonBlenderCoordinate()
        	if (((abs(float("{0:.3f}".format(poseValue[0]))) - abs(self.navCellPoseX)) < 0.31) and ((float("{0:.3f}".format(poseValue[1])) - abs(self.navCellPoseY)) < 0.1)):
        		poseValue = 0.0
        	else:
        		poseValue = self.GetWagonRailCoordinate()
        else:
        	poseValue = self.GetWagonRailCoordinate()
        return "[ " + str(poseValue/self.naviMeshCellToGridCell) + " ]"

    @service
    def Switch(self, direction):
    	""" This service is for making the wagon change 
    	    direction relative to its current position.
    	    1.0=left, 2.0=right, 3.0=forward and 4.0=backward
    	"""
    	self.switchFlag = True
    	self.targetLengthStore = 0.0
    	return self.SwitchNavMesh(direction)


    @service    
    def Stop(self):
        """ This service is for stopping the wagon.
        """
        self.wagon['GamePropForward'] = 0
        self.wagon['GamePropBackward'] = 0


    def default_action(self):
        """ Main loop of the actuator.
        """
        vel = 0.0
        robot = self.robot_parent
        #vel = self.local_data['velocity']
