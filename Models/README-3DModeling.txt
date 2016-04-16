AccMet-Scene Modeling README
============================
:Author: Avantys Engineering GmbH & CoKG
:Date:   August 21st, 2015


AccMet-Lab Scene
----------------

AccMet_lab scene consists of different blender models which are
put together while rendering the scene in the simulation. Various
blend file models which are used in the scene can be found in the 
'Models' folder.


Extending the Track System
--------------------------
The track system consist of two components. One is the track model,
which is the 3D model of the real-world track and a navigation mesh
which is a plane of exact same shape as the surface of the track. 
The navigation mesh, however is not visible in the simulation and 
sits just on top of the 3D model of the track. 

3D model of the track is segmented as a straight-track and a curve-
track models. The navigation mesh is also segmented in the same way.


  * For extending the 3D track model do the following steps:

    1) Ensure that the models is in Object Mode in Blender.
    2) Go to File options and click on Append.
    3) Locate the straight/curve track segment blend file.
    4) Click on the blend file and then go to Object folder.
    5) And then, finally select the track Object.

    
  * After the blend file is successfully appended, now it need to 
    merge the existing 3D models of the track with the new track
    segment. For that do the following:

    1) Scale the new track segment as same as the existing track.
    2) Align and place the new track segment close to the existing 
       track without overlapping the faces.
    3) Now, click on the exiting track, and select the newly added 
       track (by pressing Shift-key).
    4) While in the Object Mode, type Ctrl-J and this merges the 
       newly added track with the existing one.

      
  * For extending the navigation mesh do the following steps:

    1) Append the straight/curve navigation mesh as it is mentioned
       above for the track 3D model.
    2) As said above, place the newly added navigation mesh as
       close as possible to the existing navigation mesh, without
       overlapping.
    3) While in the Object Mode, type Ctrl-J and this merges the 
       newly added navigation mesh with the existing one. 

  
  * This operation sometime results into the navigation mesh losing 
    its properties. To retain the navigation mesh properties, do the
    following:
 
    1) Select the navigation mesh and open the Properties panel  
       usually on the right-hand side of the blender window.
    2) Click on the Physics button (usually the last button).
    3) It can be seen that the Physics Type selected is Navigation
       Mesh. Below that, click on the NavMesh Reset Index Values.
      

