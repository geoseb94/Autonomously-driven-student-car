# Autonomously-driven-student-car


= Drive the Maze 2 Team 4 =

== Team Structure ==

 ||'''Name'''                ||'''Matriculation Number'''||'''   Email                    '''||'''Development Task'''         ||'''Management Task'''         ||
 ||George Sebastian||762610||george.sebastian@stud.h-da.de  ||Core algorithm engineer for route estimator and path planning. Simulation and validation of core algorithms ||Project Manager||
 ||Athira Ramakrishna||762598||athira.ramakrishna@stud.h-da.de ||Core algorithm engineer for path planning and completion criteria. Simulation and validation of core algorithms ||System Architect||
 ||Sai Parimi||762583||stsapari@stud.h-da.de ||Scanning and fusion, camera SW. Point stabilization and static rotation of car ||Test Manager||
 ||Pavan Kamath Ammembal||762512||pavan.ammembal@stud.h-da.de||Scanning and fusion, camera SW. Development of color sequencer and map visualizer tools ||Integrator||
 ||Jishnu Murali Thampan||762574||jishnu.thampan@stud.h-da.de||Core algorithm engineer for route estimator and sequence pre-processing. Development of simulation tool for core algorithms ||Requirements Engineer||


== Workpackage Distribution ==

[[Image(WPD.PNG)]]

== Meeting Minutes ==

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/10_Projectmanagement/10_Minutes]

== Requirements ==

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/20_Requirements/SYS_SRS_Maze2.doc]

== Feature Plan ==

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/10_Projectmanagement/20_PlanSchedule/02_PM_FP_Project.xls]

== SVN ==

All project related documents are placed in the SVN link below:

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4]

== Release Folder ==

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/40_Implementation/04_Release]


== Project Video Link ==

SVN Link: 

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/10_Projectmanagement/40_Presentation/04_Race_The_Maze_II_FinalVideo.mp4]


Youtube Link: 

[https://www.youtube.com/watch?v=XJ5ApJoVQBA]


== 1 System Architecture ==

Autonomous driving as a future technology seems to revolutionize mobility. Autonomous driving promises many benefits: improved safety, reduced congestion and lower stress for car occupants, among others.

          The system in general is an autonomously driven car with the help of a Lidar sensor. The lidar sensor in the car continuously monitors for the obstacles in the course of the car movement. The car now has a camera which continuously looks for obstacles in it vicinity and reports the obstacle information such as the color and position of the beacon to the Aurix controller. The car can navigate to the desired obstacles with the integrated information from the camera and the lidar sensor. The autonomous mode algorithm is implemented in the new runnable which is scheduled cyclically once the user chooses the autonomous mode. The whole implementation resides in the application part which also interacts with the PSOC to get the input Lidar values. The system also interacts with the decoder values which gives the information about the current position of the car. 

Functionality in scope
•	Beacon Identification in the maze
•	Maze traversal according to the input sequence(beacon color) provided by the user
•	All previous modes such as manual and autonomous modes are retained along with this new feature

The maze is categorized into cells. First, The car does a Scanning around the maze taking object shadowing into account. Next, based on the input sequence and Scanned data, The Algorithm calculates set of waypoints for the car to traverse. The Waypoints are then executed seqeuntially to reach the destination. 

[[Image(HL_Design.jpg)]]


=== 1.1  List of new features implemented on top of existing system ===

* Customised Tools for Simulation, Input sequence transmission and Map Visualisation.
* Efficient Algorithm to generate waypoints
* Camera and LIDAR fusion to scan the maze

== 2 Components and key features ==

 ||'''Sl.NO'''||'''Key Features'''||'''Key Role'''||
 ||1||Aurix-TC297||Maze2 Application runs on core 0 of AurixTC297 where the Lidar and Camera data are analyzed to identify the colored object location and drive through the maze as per the sequence. Camera data and LIDAR data received from 2 different PSOC5LP via CAN; Communicates to the Remote Control and Sequence Processing Tool via XBee Provides speed data to the engine via CAN.||
 ||2||PSOC5LP||2 PSOC5LP are used Each of it receives the data from LIDAR and Camera respectively via UART and sends it to the AurixTC297 via CAN. Data is sent every 100ms via CAN||
 ||3||OpenMV Cam M7||RGB image format is used in the application to identify the colors and data is sent via UART to PSOC5LP||
 ||4||RPLIDAR A2||Used to detect the presence of the object and data is sent to the PSOC via UART||
 ||5||XBee PRO RF Module ||Selection of Autonomous or Manual Mode is done via the Remote Tool and sent through XBee Input of the color Sequence for the Maze2 traverse is also sent via XBee||


== 3 Class Diagram ==

[[Image(class_diagram.png)]]

'''Algorithm'''  Path Planning algorithm class which generates the path information and provides this to the Car Control class

'''Carcontrol''' Responsible for the car movement, rotation and calibration related functions. This class is controlled my the Race class which upon getting the inputs from the Path Planner, passes all the necessary info to this class.

'''Race:'''  Interfaces the path planner class and Car control class. This class consists of the central state machine which initiates and controls the overall system functionality. Errors reported by the individual classes are are captured here and reported to the visualizer tool via the logging framework. This class consists of a reference to the global maze object and the maze information is updated to this reference by the individual classes such as Car control and Path planner

'''Test'''  Test class to simulate the race

'''Maze'''  Represents the global maze object

'''Queue'''  Template class that implements a FIFO Queue

'''CObstacleData'''  Represents the maze obstacles

'''CNode'''  Represents a Node in the tree data structure 


== 4 State Machine ==

[[Image(SM_1.png)]]

=== 4.1 Brief regarding each state ===

'''IS_WAITING_FOR_INPUT:''' State to wait for the valid input sequence from the Sequence tool before the car could traverse the maze. 

'''IS_INIT2ORG:''' State where the car is moved towards the origin when placed anywhere in the maze.

'''IS_SCANNING:''' State where the maze is scanned to identify the position of the colored beacons.

'''IS_SEARCHING:''' State where the validity check is done based on the input sequence and avaialable Beacons in the maze from the Scanning state.

'''IS_POPULATING_WPS:''' State where the route is populated for the maze to move towards each destination 

'''IS_DRIVING:''' State where the car moves towards the destination through the waypoints that has been filled in the IS_POPULATING_WPS state

'''IS_APPROACHING:''' State where the car performs the completion Criteria Approach 2 of sending the message as the car approaches near the destination.

'''IS_STOPPING:''' State where the check if all destinations have been covered is done, if yes it remains in this STOP state else goes back to IS_POPULATING_WPS state and continues with the next destination

'''IS_ERROR:''' State where Error from anyother states have been handled


== 5 Customized tools ==

=== 5.1 MAZE02 Colour Selector and Map Visualiser Tool ===


[[Image(Tool_1.PNG)]]


'''Goal'''
1. To provide colour sequence of objects for the Maze traversal 
2. To get a visual representation of the location of Car within the maze
3. To use as a diagnostic Console


[[Image(TOOL_2.PNG)]]


COM Port selector helps in selection of the ZIGBEE serial dongle. Every click on the drop-down refreshes the selector. This is an important step to start using the tool. 


[[Image(TOOL_3.png)]]


Color Picker enables user to pick the sequence that needs to sent to the Aurix. Appropriate colour should be selected by pressing the + button beside the specific colour.  (as shown in Fig 3)

After selection of colours, Send sequence button is sent. 


[[Image(TOOL_4.png)]]



Map Visualization feature can be enabled by clicking the Map > Connect Map Data.

This opens an expanded window that shows a matrix view of co-ordinates.


[[Image(TOOL_5.png)]]



The Maze is divided into 7x7 matrix with left bottom (Green) considered as origin. 

Khaki colour shows the location of the Car within the Maze.
Red, Blue and Green shows colour of the specific object.

Pressing clear console erases the console and the map.


[[Image(TOOL_6.png)]]


'''Tool, Source code and Manual are placed at:'''

 
Source: [http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/40_Implementation/04_Release/03_Colour_Seq_Map_Visualiser_Source]


Tool Document: [http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/10_Projectmanagement/50_Documentation/06_Colour Selector and Map Visualiser Tool.docx]


=== 5.2 MAZE-Simulation tool : Simulation of Core Algorithms ===


Consists of: 
 * Maze grids
 * Color picker
 * Obstacle Info file                                                     
 * Input sequence grids
 * Send button
 * Input sequence file

[[Image(SIM_1.jpg)]]


'''Screenshot of the Tool:''' 

[[Image(SIM_2.png)]]

'''Tool, Source code and Manual are placed at:'''

 
Source: [http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/40_Implementation/04_Release/02_Simulation_Source]


Tool Manual: [http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/10_Projectmanagement/50_Documentation/07_MAZE02_simulation-manual.docx]

== 6 Scanning and Object Fusion ==

'''Goal''' – To ascertain the co-ordinates and color of the object within the maze.

Illustration for Co-ordinate estimation: 
Car reaches Origin after a run of Init to origin component, As shown in figure1. Here we start traveresal 
in the Degree 90 (Right) direction, Until an object is detected. (Fig 2).
 

[[Image(SCAN_1.png)]]



[[Image(SCAN_2.jpg)]]


       Fig 2: Car moving in the right (90) direction


As the car moves, parallely LIDAR data is analysed if the 0-deg Direction less than 300. When this condition is satified, 2 actions take place:
* Distance from left, Direction 360 degree, is measured. Let us take this distance as X and front, Degree 0, as Y.
* Colour is extracted from Camera data packet. This colour would be ideally match with colour of the object that is infront of the car.
X-Coordinate = X/ (unit distance of each square ie. 80cm)

Y-Coordinate = Y/ (unit distance of each square ie. 80cm)


[[Image(SCAN_3.jpg)]]


Widest layout is chosen as the possible colour of the object

[[Image(SCAN_4.PNG)]]


== 7 Input Sequence Pre-Processing ==

The input provided by the user is pre-processed in order to :

* Eliminate all invalid entries
* Eliminate any discontinuity in the sequence
	     For ex. The sequence could be [[span(style=color: #FF0000, RED )]], [[span(style=color: #FF0000, RED )]], [[span(style=color: #808080, INVALID*)]], [[span(style=color: #0000FF, BLUE)]]
                  Here there is a discontinuity at position 3.
*  Initial filtering eliminates the need of additional validity checks 	    		

	

 [[span(style=color: #808080, INVALID*)]] indicates any obstacle color which is out of the scope of the game


Input    - [[span(style=color: #FF0000, RED )]], [[span(style=color: #00FF00, GREEN)]], [[span(style=color: #808080, INVALID*)]], [[span(style=color: #0000FF, BLUE)]]

Output - [[span(style=color: #FF0000, RED )]], [[span(style=color: #00FF00, GREEN)]], [[span(style=color: #0000FF, BLUE)]]

Input    - [[span(style=color: #00FF00, GREEN)]], [[span(style=color: #0000FF, BLUE)]], [[span(style=color: #FF0000, RED )]], [[span(style=color: #808080, INVALID*)]], [[span(style=color: #808080, INVALID*)]]

Output - [[span(style=color: #00FF00, GREEN)]], [[span(style=color: #0000FF, BLUE)]], [[span(style=color: #FF0000, RED )]]

Input    - [[span(style=color: #808080, INVALID*)]], [[span(style=color: #808080, INVALID*)]], [[span(style=color: #FF0000, RED )]], [[span(style=color: #808080, INVALID*)]], [[span(style=color: #808080, INVALID*)]]

Output - [[span(style=color: #FF0000, RED )]]


== 8 Route estimator algorithm ==


'''Aim''' – There could be multiple possible routes satisfying the pre-processed input color sequence and the colored obstacles present in the maze. The route estimator algorithm finds out the most efficient and shortest route among the all possible routes.

[[Image(RE_1.jpg)]]

'''Problem statement''' - Suppose the maze consist of three red, one green and one blue obstacles, and an input sequence of red,  red, red, green, blue  is fed to the system. The position and color of all the obstacles are identified after the scanning phase. The input sequence is pre-processed for error handling. There are six possible routes satisfying the given input sequence starting from the initial position of the car as shown in the below figure. The numbers in the gird represent the order in which each grid is approached. The route estimator should find out the most efficient route among the all possible routes. 

[[Image(RE_2.jpg)]]

'''Solution''' - A tree data structure is maintained as a collection of nodes (starting at a root node), where each node is a data structure consisting of a value, together with a list of references to nodes (the "children"), with the constraints that no reference is duplicated along each branch, and none points to the root. These constraints make sure that each obstacle is traversed only once during the race.
The root node is the starting position of the car in the maze. The rest of the nodes are filled as per the pre-processed input sequence and the colored obstacles in the maze. Each node stores the complete object information as well as the distance/ number of steps from its parent node using BFS algorithm. Each branch in the tree data structure represents the possible routes satisfying the input sequence.
The route estimator algorithm sums up the total distance along all the branches, and selects the branch with minimum number of steps as the most efficient route. In case of multiple efficient routes, the order of input obstacle is considered.

[[Image(RE_3.jpg)]]


== 9 Path Planning  algorithm == 

'''Aim''' – There could be multiple possible paths starting from the source node to the destination node in the maze. The path planning algorithm finds out the most efficient and shortest path among all the possible paths without hitting the obstacles.

[[Image(PP_1.jpg)]]

'''Problem statement''' - Suppose the maze consist of two red, one green and one blue obstacles.  Let the source node be at (0,0) and destination node be at (1,3). The path planning algorithm should find out the most efficient and shortest path from source to destination node without hitting any of the obstacles as shown in the below figure.

[[Image(PP_2.png)]]

'''Solution''' – A modified form of Breadth First Search algorithm is used to estimate the shortest path between the source and destination. Starting from the source node, its immediate neighbors excluding the obstacle nodes are stored in a queue data structure. This process of storing immediate neighbors is continued until the destination node is found. Finally the shortest path is estimated by backtracking from the destination node up to the source node. 

[[Image(PP_3.png)]]
[[Image(PP_4.png)]]

'''Integration of sequence pre-processing, route estimator and path planning algorithms'''

[[Image(INT_PP_RE.png)]]

== 10 Completion Criteria == 

=== 10.1 Approach 1 ===
 Once the car reaches the penultimate position in the given route, it looks out for the nearest obstacle in the direction of the destination and completes a 360 movement around the destination object which is the colored beacon in our case.

[[Image(CC_1.PNG)]]

Considering the scenario as in the above figure, if the Route is 1 to 7 points in the maze
1. As the Car moves from Starting point to the destination, the Car stops at point 6 (penultimate) and as it knows the next moving direction should be towards point 7 as the beacon is in that grid, it checks for the nearest obstacle (beacon) within the angles 225 and 315 and gets the distance and angle of the beacon with respect to the Car.
2. Checks if the angle of the beacon with respect to the car is less than 270 and greater than 225 then Car moves backwards so as to align with the beacon and then starts moving to towards the beacon(which is the case illustrated in the above figure). If the angle of the beacon with respect to the car is greater than 270 and less than 315 then car moves forward to align with the obstacle and then moves towards it.
Car follows the similar strategy with respect to to the other angles and moves right/left or forward/backward to align with the beacon.
3. Once it reaches closer to the beacon it does 360 around the beacon and proceed with the next route.
Formula used for the example illustrated above:
if(A_Ldr_Obs>= 225 && A_Ldr_Obs <=270 )
then the beacon is behind  with respect to to the car and car has to move backwards with X distance to align with the beacon
X=D_ Ldr_Obs*sin((270- A_Ldr_Obs) * 3.14/ 180)	
if(A_Ldr_Obs> 270 && A_Ldr_Obs <=315 )
then the beacon is ahead the car and car has to move forward by X distance to align with the beacon
X=D_ Ldr_Obs*sin((270- A_Ldr_Obs) * 3.14/ 180)
where ;
A_Ldr_Obs is the Angle of the nearest obstacle in the given range 
D_Ldr_Obs is the Distance of the nearest obstacle in the given range.
X is the distance to be travelled to align with the beacon
Once the X distance has been traversed it moves closer to the beacon and completes the 360 rotation around it.
Limitation:
If there are other beacons which is not of our interest within the range then car would assume that to be the point of interest and move towards it, which is not the desired behavior.
This could be avoided by checking the color of the nearest obstacle identified matches with the our point of interest beacon then proceed with the aligning and the rest of the 360, however due to time constraints this part could not be implemented.
As Illustrated below green is the desired beacon , however since blue is closer to the car it assumes it to be the point of interest and aligns towards it which is not the desired behavior.

[[Image(CC_2.PNG)]]


=== 10.2 Approach 2 ===

The car reaches the penultimate position and sends the message about the direction and color of the beacon to the Input Sequence tool via Xbee.
Example : As the car reaches point 6 it sends out a message to the tool via XBee that the destination is on the right/left/front/back and also the color of the beacon.
(Currently this method is available in the Software)

[[Image(CC_3.PNG)]]



== 11 Quality == 


== 11.1 System Test Specification == 

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/50_Quality/20_Tests/03_TST_TSR_Project.xls]

== 11.2 Test cases using Simulation Tool ==

[http://project.eit.h-da.de/studentcar/studentcar/20_Projects/470_Maze_II/Team_4/50_Quality/20_Tests/02_TST_TC_With_Sim_Tool.docx]





