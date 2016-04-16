from morse.builder import *
from AccmetSimulation.builder.actuators import Steer

class Wagon(GroundRobot):
    def __init__(self, name = None, debug = True):
        Robot.__init__(self, 'AccmetSimulation/robots/wagon.blend', name)
        self.properties(classpath = "AccmetSimulation.robots.wagon.Wagon")
        self.steer = Steer()
        self.append(self.steer)