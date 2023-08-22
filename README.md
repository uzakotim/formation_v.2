# Non-linear Optimal Perception-Driven Multi-drone Control
My Master Thesis diploma work. The package of ROS(noetic) nodes, that enable drones to detect colored plates and form a triangular shape around them.


USER GUIDE:
- press r for recording/not_recording centroid between drones
- use w,a,s,d for moving formation in 4 directions 
- press c to enable/disable automatic searching
- press 1 for moving/stopping uav1
- press 2 for moving/stopping uav2
- press 3 for moving/stopping uav3
- press m to switch mode: searching/form_formation
- press 9 to decrease radius
- press 0 to increase radius
- press i to switch ignore mode
- when ignore is on: drones do not separate 
- when ignore is off: each drone stops at observations
- press q to exit

WARNINGS:
!Make sure that drones communicate and the centroid has been recorded!
!Be aware to turn IGNORE ON when returning to searching mode!
!Be aware to turn RECORDING_CENTROID OFF BEFORE setting MOVING ON!


       
