#! /usr/bin/env python3
"""
Test client for the <AccmetSimulation> simulation environment.

This simple program shows how to control a robot from Python.

For real applications, you may want to rely on a full middleware,
like ROS (www.ros.org).
"""


import sys
import math
#from morse.testing.testing import MorseTestCase
from pymorse import Morse
from morse.core import blenderapi



try:
    from pymorse import Morse
except ImportError:
    print("you need first to install pymorse, the Python bindings for MORSE!")
    sys.exit(1)

from morse.core.exceptions import MorseBuilderNoComponentError

bpy = None

try:
    import bpy
except ImportError:
    print("WARNING: MORSE is running outside Blender! (no bpy)")


if bpy:
	print("Test!")


# print("Use ASD to control the robot")

# with Morse('localhost', 4001) as simu:

#   motion = simu.wagon.motion
#   pose = simu.wagon.pose
#steer = simu.robot.steer
#scene.objects




  # v = 0.0
  # w = 0.0

  # while True:
  #     key = input("WASD?")

  #     if key.lower() == "w":
  #     #    v += 0.1
  #     elif key.lower() == "s":
  #     #    v -= 0.1
  #          steer.Stop()
  #     elif key.lower() == "a":
  #     #    w += 0.1
  #          steer.MoveBackward()
  #     elif key.lower() == "d":
  #     #    w -= 0.1
  #          steer.MoveForward()
  #     else:
  #         continue

  #     # here, we call 'get' on the pose sensor: this is a blocking
  #     # call. Check pymorse documentation for alternatives, including
  #     # asynchronous stream subscription.
  #     print("The robot is currently at: %s" % pose.get())

      #motion.publish({"v": v, "w": w})
