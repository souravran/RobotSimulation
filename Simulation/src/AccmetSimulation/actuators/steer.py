import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.core import blenderapi
from morse.helpers.components import add_data, add_property
from morse.core import mathutils
from morse.builder import bpymorse
import math


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
        self.navCellPoseX           = -26.5592 ## assigned with start cell X-Coordinate
        self.navCellPoseY           = -16.55265 ## assigned with start cell Y-Coordinate 
        self.navCellPoseZ           = 11.83072 ## assigned with the Z-Coordinate of the navigation plane, it is always constant
        self.crossProductThreshold  = 0.26 ## threshold value for considering the right number of wagon pose in cross product
        self.initFlag               = False ## used for setting the robot to stop motion once at the start cell [0,0] during initialization
        self.naviMeshCellToGridCell = 2.234  ## conversion factor for changing navigation mesh cell length to grid cell length, at application level grid cells are being used


    ## from rail coordinate, setting the target position in blender coordinate in (x,y,z)
    def SetTargetBlenderCoordinate(self, relativeDist):
        previousCell    = None
        targetCell      = None
        cellLength      = 0
        lastCellLength  = 0
        lengthFactor    = 0
        navCellPoseX    = 0
        navCellPoseY    = 0

        targetLength = self.GetWagonRailCoordinate() + (relativeDist)
        #targetLength = 20.0
        print("\n[target distance :",targetLength,"]")
        print("[relative distance :",relativeDist,"]\n")
        
        for poly in self.nav.data.polygons:
            nextCellLocalCoord = mathutils.Vector(poly.center)
            nextCell = self.nav.matrix_world * nextCellLocalCoord
            #print("Next Cell ", nextCell[0], nextCell[1])
            if previousCell:
                targetCell = nextCell
                
                previousCell -= nextCell
                cellLength = cellLength + previousCell.length
                if cellLength > targetLength:
                    lastCellLength = previousCell.length
                    navCellPoseX = float("{0:.4f}".format(targetCell[0]))
                    navCellPoseY = float("{0:.4f}".format(targetCell[1]))
                    cellLength = cellLength - lastCellLength
                    break
            previousCell = nextCell

        print("Relative distance ", navCellPoseX, navCellPoseY)
        ## calculate the offset distance, which is the distance between the last cell and target length
        offsetLength = targetLength - cellLength
        if lastCellLength != 0:
            lengthFactor = offsetLength/lastCellLength

        navCelltargetPoseX = ((1 - lengthFactor) * navCellPoseX) + (lengthFactor * nextCell[0])
        navCelltargetPoseY = ((1 - lengthFactor) * navCellPoseY) + (lengthFactor * nextCell[1]) 
        if relativeDist > 0:
            self.wagon.actuators['GoForward'].target.position = [navCellPoseX, navCellPoseY, self.navCellPoseZ]
            print("Relative distance has been set", navCellPoseX, navCellPoseY)
        else:
            self.wagon.actuators['GoBackward'].target.position = [self.navCellPoseX, self.navCellPoseY, self.navCellPoseZ]


    ## getting the rail coordinate, which is the absolute distance of wagon from the start of the track
    def GetWagonRailCoordinate(self):
        previousCell    = None
        cellLength      = 0
        currentPose     = self.GetWagonBlenderCoordinate()
        currentPoseX    = float("{0:.3f}".format(currentPose[0]))
        currentPoseY    = float("{0:.3f}".format(currentPose[1]))

        print("Current Pose ", currentPoseX, currentPoseY) 

        for poly in self.nav.data.polygons:
            nextCellLocalCoord = mathutils.Vector(poly.center)
            nextCell = self.nav.matrix_world * nextCellLocalCoord        
            if previousCell:
                startCellX      = float("{0:.3f}".format(previousCell[0]))
                startCellY      = float("{0:.3f}".format(previousCell[1]))
                endCellX        = float("{0:.3f}".format(nextCell[0]))
                endCellY        = float("{0:.3f}".format(nextCell[1]))

                diffPoseX       = float("{0:.3f}".format(currentPoseX - startCellX))
                diffPoseY       = float("{0:.3f}".format(currentPoseY - startCellY))
                diffCellX       = float("{0:.3f}".format(endCellX - startCellX))
                diffCellY       = float("{0:.3f}".format(endCellY - startCellY))
                crossProduct    = abs(diffPoseX * diffCellY - diffPoseY * diffCellX)
                #crossProduct    = abs(crossProduct)
                print("   Start and end cell  :", startCellX, startCellY, endCellX, endCellY)
                print("Cross Product, diff pose, diff cell length : ", crossProduct, diffPoseX, diffPoseY, diffCellX, diffCellY)
                if crossProduct == 0 or crossProduct < self.crossProductThreshold :
                	#print("Diff cell X and Y : ", diffCellX, diffCellY)
                	if (abs(diffCellX) >= abs(diffCellY)):
                		if diffCellX > 0:
                			if (startCellX <= currentPoseX and currentPoseX <= endCellX) or (endCellX <= currentPoseX and currentPoseX <= startCellX):
                				print("Break Cell X", startCellX, endCellX)
                				break;
                	else:
                		if diffCellY > 0:
                			if (startCellY <= currentPoseY and currentPoseY <= endCellY) or (endCellY <= currentPoseY and currentPoseY <= endCellY):
                				print("Break cell Y", startCellY, endCellY)
                				break;
                previousCell -= nextCell
                ## each Cell length in the Navigation-Mesh is 0.6265316078016844 with resepect to the current simulation scene scale  
                cellLength = cellLength + previousCell.length
                #print("Calculated cell length :", cellLength, previousCell.length)
            previousCell = nextCell

        print("Current Length", cellLength)

        ## now calculate the distance between the center of last cell till the current pose
        print("Previous Cell", previousCell[0], previousCell[1])
        print("Previous cell Length before", previousCell.length)
        previousCell -= currentPose
        print("New previous cell", previousCell[0], previousCell[1])
        print("Previous cell Length after", previousCell.length)
        cellLength = cellLength + previousCell.length
        return cellLength


    ## get the wagon position from blender in (x,y,z) 
    def GetWagonBlenderCoordinate(self):
        return self.wagon.position

    def SetNavMesh(self, case):
        if (case == 1) :
        	#self.wagon.actuators['GoForward'].navmesh = "routeInit"
        	self.nav = bpymorse.get_object("routeInit")
        if (case == 2) :
        	self.nav = bpymorse.get_object("route")
        	self.wagon.actuators['GoForward'].navmesh = "route"
        	self.wagon.actuators['GoBackward'].navmesh = "route"
        	#self.nav = bpymorse.get_object("route")



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
        self.SetNavMesh(2)
        self.relativeDist = relativeDist
        self.SetTargetBlenderCoordinate(self.relativeDist)
        self.Move(self.relativeDist, velocity)


    @service
    def Initialize(self):
        """ This service is for making the wagon to initialize at 
            the start of the track.
        """
        self.SetNavMesh(1)
        self.navCellPoseX = -26.5592 ## assigning with the start cell X-Coordinate
        self.navCellPoseY = -16.55265 ## assigning with the start cell Y-Coordinate
        self.Stop()
        self.initFlag = True
        self.wagon.actuators['GoForward'].target.position = [-26.5592, -16.55265, self.navCellPoseZ]
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
