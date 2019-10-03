4-LINK ROBOT SIMULATOR
Simulates a 4-link/3-joint robot to demonstrate key robotic concepts for manipulation and locomotion whilst avoiding obstacles.
Inverse kinematics, Kalman filtering, Q-learning, & simulated annealing are dealt with separately.

AUTHORS
David Hardman (dsh46@cam.ac.uk), University of Cambridge, September 2019
Fumiya Iida, University of Cambridge, September 2018

CREATED USING MATLAB 2019a

RUNNING
Unzip contents of simulator.zip then double click on SIMULATOR.mlapp in main directory


KEY VARIABLES

phiVec: 3x1. Contains the angles in radians of the 3 joints, as ordered in Labels.png.

lVec: 6x1. Contains the lengths of the links and feet, as ordered in Labels.png.

mVec: 9x1. Contains the relative masses of the links, feet, & servos, as ordered in Labels.png.

fixed: Integer value of 1 or 2: determines which foot is flat on the ground.

pos1: x,y coordinates of intersection of link 1 and foot 1.

pos2: x,y coordinates of intersection of link 4 and foot 2.

rMat: 3x7. Coordinates of link/foot intersections, calculated from phiVec,lVec,pos1/pos2 using rmat1cal/rmat2calc.

footsep: Minimum distance that these 2 points can be apart without collisions involving the feet.

corners: 2xn. Rectangular obstacles entered in [[bottomleft1] [topright1] [bottomleft2] [topright2]...] x;y format.

n: Number of possible positions of each servo during Q-Learning/Simulated Annealing discretisation.


KEY FUNCTIONS AND SCRIPTS

StandardStandingParameters: Initialisation script. Sets key variables.

centreofmass: Given the component masses and arrangement, returns coordinates of centre of mass.

fixed1/fixed2: Draws/re-draws robot position with fixed foot. Colours background red for any collisions/instabilities.

obstaclecollide: Checks for collision of robot with obstacles, returning Boolean (1 indicates collision).

selfcollide: Returns 1 if any links intersect.

stability: Checks whether robot is stable on the foot set as 'fixed', returning Boolean of instability.

throughground: Returns 1 when any of the links are (partially) below ground level. Second output is an array of these joints.



MANUAL STRIDE
Hard-coded robot walking.

	-Calls StandardStartingParameters.m
	-Calls manualstride.m, which performs 5 hard-coded steps

INVERSE KINEMATICS
Inverse kinematics with one foot fixed - end point is entered by user.

	-Calls StandardStartingParameters.m
	-Calls inversekinematicsdemo.m, which calls JacInv1.m or JacInv2.m depending on which foot is fixed 

KALMAN FILTERING
Moves sv3, using accelerometer to intermediately calculate the orientation of link 1 and compare with that expected.
See KalmanSetup.png for accelerometer and servo connections.

	-Calls StandardStartingParameters.m
	-Runs kalmanapp.mlapp
	-BUTTONS:
		-Initialise Servos calls Initialise3servos.m, requiring connected Arduino
		-Calibrate Accelerometers calls acccalibrate.m, returning global calibration settings
		-Alternatively, Use Standard Accelerometer Values sets values defined in mlapp buttonpushed function
		-Go calls kalmanfilterdemo.m: filtering parameters set within script

Q-LEARNING
Q Learning with a foot fixed to ground. Desired start/finish angles are entered and path between them is learned.

	-Runs qlearningsetup.mlapp
		-Defaults defined in StandardStartingParameters.m, and in mlapp buttonpushed function
		-Go calls coarsegeneralqlearn1.m or coarsegeneralqlearn2.m, passing settings as inputs

SIMULATED ANNEALING
Aims to find most efficient valid path between start & end phiVecs entered.

	-Runs simannsetup.mlapp
		-Defaults defined in StandardStartingParameters.m, and in mlapp buttonpushed function
		-When Go pushed:
			-Calls generalsimulatedannealsetup.m, passing settings as inputs. Returns statespace of
			valid states, and the indices of the start/end states
			-Calls simulatedannealing.m, which optimises and returns a path between these states
			-Calls performpath.m, which animates this path

WALKING USING SIMULATED ANNEALING
Uses simulated annealing to walk forwards & deal with obstacles.

	-Runs simannwalksetup.mlapp
		-Defaults defined in StandardStartingParameters.m, and in mlapp buttonpushed function
		-When Go pushed:
			-Coverts starting phiVec to closest 'state' in the base-n discretised statespace
			-Calls simulannwalk.m, passing settings as inputs