# Autonomously-driven-student-car

Autonomous driving as a future technology seems to revolutionize mobility. Autonomous driving promises many benefits: improved safety, reduced congestion and lower stress for car occupants, among others.

The system in general is an autonomously driven car with the help of 2D Lidar and camera sensor. The lidar sensor in the car continuously monitors for the obstacles in the course of the car movement. The camera which continuously looks for obstacles in it's vicinity and reports the obstacle information such as the color and position of the beacon to the Aurix controller. The car can navigate to the desired obstacles with the integrated information from the camera and the lidar sensor. The autonomous mode algorithm is implemented in the new runnable which is scheduled cyclically once the user chooses the autonomous mode. The whole implementation resides in the application part which also interacts with the PSOC to get the input Lidar values. The system also interacts with the decoder values which gives the information about the current position of the car.

Functionality in scope

- Beacon Identification in the maze
- Maze traversal according to the input sequence(beacon color) provided by the user
- All previous modes such as manual and autonomous modes are retained along with this new feature

The maze is categorized into cells. First, The car does a Scanning around the maze taking object shadowing into account. Next, based on the input sequence and Scanned data, The Algorithm calculates set of waypoints for the car to traverse. The Waypoints are then executed seqeuntially to reach the destination.


TODO : add image here
