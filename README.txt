AccMet Simulation README
========================
:Author: Avantys Engineering GmbH & CoKG
:Date:   July 22nd, 2015


What is AccMet Simulation?
--------------------------

AccMet is focused on developing an in-house facility  thats will utilize
a rail system with multiple robotic wagons mounted on it. For iteratively
handling "Navigation" task for wagons, and the system prototyping, the 
entire work has been developed in 3D simulated environment using MORSE,
that uses Blender 3D modeling tool. Virtual prototyping involves:

* Simulation of all important aspects of a prototype;
* Facility environment design;
* Defining kinematics, dynamics and physics for wagons; 
* The possibility to run multiple physics engines at the same time;
 

Installing MORSE
------------------

MORSE is available for Ubuntu >= 13.04. You can install the package 
morse-simulator with your favorite software manager:

 $ sudo apt-get install morse-simulator

You can also install the Python bindings:

 $ sudo apt-get install python3-morse-simulator


Running Accmet Simulation
-------------------------

If you have successfully installed the above packages then, checkout 
the AccmetSimulation changes from git-repository. Provided is the 
run-sim.sh script which checks for the correct MORSE installation, 
setup, and finally starts the AccmetSimulation. One can run the 
run-sim.sh script from any directory with:

 $ ./run-sim.sh
 
This will create the basic simulation environment which includes:

* Create directory AccmetSimulation with wagon and basic builder script; 
* Imports a basic scene to the current simulation;
* Enable the wagon to be controlled by keyboard using "A/D" 
  key for forward/backward movement and "S" for stop; 


Interacting with Accmet Simulation
----------------------------------
Once the MORSE Accmet simulation has started up, the simulation
could be interacted and the wagon behavior could be controlled
via Telnet connection. While the simulation is running, open another
terminal window, and type:

 $ telnet localhost 4001
 
This will connect to the running simulation through port 4001

Now the wagon could be seen placed on the track. So to control
the navigation, the wagon has to be initialize first, hence 
the wagon initialization could be done with following:

  $ ID wagon.steer Initialize
  
This should return a 'ID SUCCESS' and the wagon should move to
the start of the track.

The wagon current position is the absolute distance of the wagon
from the start of the track. Current position of the wagon can 
be obtained by using:

  $ ID wagon.steer TrackPose
  
This should return a 'ID SUCCESS' along with printing the
current wagon position.

The wagon relative movement can be accomplished by providing
any relative distance value and velocity. The telnet command 
looks like the following:

  $ ID wagon.steer MoveRelative [40.0, 20.0]
  
This should return a 'ID SUCCESS', also prints the wagon current
absolute location, the target distance and will make the wagon 
move for 40.0 units on the track, relative to its current position 
and with 20.0 units of speed.

