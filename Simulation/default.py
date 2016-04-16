#! /usr/bin/env morseexec

""" Basic MORSE simulation scene for <AccmetSimulation> environment
"""
import os
from morse.builder import *
from AccmetSimulation.builder.robots import Wagon

## add a new wagon robot in the simulation
wagon = Wagon()
#Dala = Wagon()
#Roxy = Wagon()
#Roxy.translate(6, 7, 0)

## add the socket interface to MORSE wagon
wagon.add_default_interface('socket')

## get the path of the AccMet-Lab scene blend file
simulation_directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
scene_file = '/Models/scene.blend'
path = simulation_directory+scene_file


## set 'fastmode' to True to switch to wireframe mode
env = Environment(path, fastmode = False)
env.set_camera_location([38.0, 6.1, 31.0])
env.set_camera_rotation([0.98, 0, 1.50])