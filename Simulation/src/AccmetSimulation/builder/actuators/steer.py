from morse.builder.creator import ActuatorCreator

class Steer(ActuatorCreator):
    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name, \
                                 "AccmetSimulation.actuators.steer.Steer",\
                                 "steer")

