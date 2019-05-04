Goals


The goal of this project was to create an indoor delivery robot that could be used to deliver messages and items from one point in the office to another point. In order to create this robot, the ROS navigation stack is implemented. ROS - robot operating system -is an open source framework that has many useful libraries and packages for robotics. It also allows the connection of multiple robotics systems to one backend. In order to develop the robot further, a decent understanding of ROS is needed. Here are some tutorials to help:
http://wiki.ros.org/ROS/Introduction
http://wiki.ros.org/ROS/Tutorials
http://www.clearpathrobotics.com/assets/guides/husky/
http://www.theconstructsim.com/ros-for-beginners/
 
The navigation stack is a set up that allows control of the robot localization, obstacle avoidance, and path planning. In order to map, use the hector slam mapping package, which allows use of LiDAR, IMU, odometry, or any combination of the three. 


This stack can be seen below:






Thus far, the robot is able to move, map, and localize with LiDAR alone. The next steps are to set up path planning and obstacle avoidance. The IMU should also be set up on the robot in order to get a better map, and better localization.




The delivery bot in its latest state (as of 04/26/2019)

Software Systems


How to set it up on a new system:
1. On a new tinker board S ( https://www.asus.com/us/Single-Board-Computer/Tinker-Board-S/ ) (or any other board) flashed with linux, install ros-kinetic-desktop-full
    * http://wiki.ros.org/kinetic/Installation/Ubuntu
    * Install the laser-scan-tools module
2. Clone the Khazanah delivery-bot-platform github repo( https://github.com/KhazanahAmericasInc/delivery-bot-platform ) to the system
3. Start up the ROS system by source the setup.bash file for ROS kinetic (see the ROS tutorials)
4. Create the package using catkin_create_package, and then run catkin_make


How to drive the bot around:
1. To manually drive the robot around, ssh into the tinker board on 3 terminals
2. Type out sros to source the setup file for ROS. This allows you to access the ROS navigation system. Do this each time you open up a new terminal, or add it to the .bashrc file to make it permanent
3. On the first terminal type: roscore
    * This starts the ROS backend that allows for communication between ROS nodes
4. Second terminal: rosrun motor_driver motor_driver.py
    * This sets up the serial communication between the tinker board and the arduino for controlling the motors
5. Third terminal: rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    * This allows you to use your keyboard to control the robot - follow the instructions on the screen to control the bot
    * This publishes velocity commands that are subscribed to by the motor driver


How to map using the robot
1. Ssh into the tinker board on 2 terminals (tinker@10.8.9.169, password: tinker)
    * In order to view the output you should use a remote desktop or ssh -X 
2. Type roslaunch rplidar_ros rplidar.launch on one terminal. 
    * This should start up ros master, and start getting input from the lidar
    * Note: you can press tab to autocomplete just like in linux
3. Second terminal: roslaunch hector_slam_launch tutorial.launch
    * This will start the mapping program and bring up rviz for you to visualize the mapping
    * Move the robot around a bit slowly to see the map clear up (see above on how to move it around)
4. To save the map open up a new terminal and ssh into the tinker board. Type: rosrun map_server map_saver
    * This will save a map in the current directory in the tinker board as map.yaml and map.gpm
    * This map can be used later for localization


How to localize
Note: each of the following steps are launched in separate terminals
1. First launch the lidar as before: roslaunch rplidar_ros rplidar.launch
2. Then run: rosrun db_navigation tf.launch
    * launches the message transformation frame tree that is used by most ros nodes
3. Next open the map up: rosrun map_server map_server map.yaml
    *  Serves the map that the localization uses for finding out where it is using adaptive monte carlo localization
    * The map.yaml can be replaced by any file in the current directory
4. roslaunch db_navigation odom.launch 
    * Runs laser_scan_matcher, which mimics odometry using the laser scans
    * Can be removed if odometry data is collected 
5. roslaunch db_navigation localization.launch
    * launches amcl for the robot
6. You can visualize all of these on rviz by adding laser scans, the pose array from amcl, and the map
    * rosrun rviz rviz starts rviz


Mechanical Systems


Motors
* The motors and wheels were part of a chassis kit that also comes with motor mounts:  https://www.amazon.com/Obstacles-Crossing-Robot-Smart-Chassis/dp/B078JZ7R42/ref=sr_1_fkmrnull_3?keywords=4WD+Robot+Chassis+Kit+Smart+Off-Road+Car+Kit+Robot+Car+Aluminum+Alloy+Chassis&qid=1556237620&s=gateway&sr=8-3-fkmrnull
* Data sheet:


* The motor driver used to power the motors is the adafruit v2.3 arduino motor shield
    * Allows for up to 3A of current draw for short time
    * Fine for most applications within the office
    * Communicates with the tinker board via the serial port on the arduino 
    * Can be replaced with perhaps a high current polou motor driver in the future
* In order to reduce friction between the wheels and the floor they are covered in duct tape
    * They should be replaced with better wheels in the future 
        * https://www.amazon.com/90-10mm-Black-Robot-Wheels/dp/B00T3MQG7M/ref=sr_1_4?keywords=robot+wheels&qid=1556318300&s=gateway&sr=8-4
        * https://www.amazon.com/Rubber-Omnidirectional-Wheels-Component-Accessory/dp/B07MPC989Y/ref=sr_1_14?keywords=robot+wheels&qid=1556318300&s=gateway&sr=8-14
    * The previous wheels did not allow good control on carpeted areas
Chassis
* The very bottom base plate of the chassis is made from 12" x 18" 16-gauge steel plate from Lowe's
    * Holes were drilled for the motor mountings 3.25 inches from the ends and some to mount the acrylic top plates
* The top acrylic mounting plates were laser cut from a large acrylic plate bought from Lowe's 
    * Laser cutting files: 
                     
* In order to hold the bottom base plate and top mounting plate together, M3 standoffs were used
    * There is an entire set in the tool box drawer, and more can be bought on Amazon if necessary
* The LiDAR is mounted with a few nuts and M2.5 screws




Electrical Systems


LiDAR
* The chosen 2D LiDAR/ 2D Laser scanner is the RPlidar from adafruit *link
* The LiDAR communicates using a UART to USB converter 
* Earlier, the LiDAR was tested with a laptop running Linux 
    * Using python directly with the liDAR the most points received on the laptop was about 200 points per scan
* On the tinker board, it could only get about 60-80 points on python
    * Due to this, the choice was made to switch from python to C++ and ROS (allows over 200 points on the tinker board per scan)
* An attempt was also made to get more points using an Arduino MEGA connected to the LiDAR via serial communication, and then communicating to the Tinker Board
    * This failed because of serial communication speed limits between the tinker board and the arduino
* All the code for the attempts where the comm was slower can be found in the github repo under the Lidar_test folder


Power Systems
* The motors are powered by a 5200 mah 50C lithium ion battery
    * The battery is connected to a hobbyking power distribution board meant to distribute power to drone motors
    * This board is connected to a step up voltage converter/ regulator that boosts the power from the 5V output of the PDB to the 9V needed for the motors
    * The battery also connects to a voltage monitor while operating that will make a buzzing sound if the voltage is going low
    * This should be unplugged and connected to the charger when 
* The tinker board is powered by a 20,000 mah power bank
    * The power bank has an output that is connected directly to the tinker board and one to a step up voltage regulator that keeps the voltage at 5.3 V
    * This is done because as the power bank discharges, the voltage drops which can cause the tinker board to power off
* There are 2 power systems on the robot because when the motors draw too much current from the Li-Ion battery, it sometimes causes the tinker board to power off and the power bank does not have enough current for the motors
* The LiDAR is powered from a USB port on the tinker board

How to turn on

1. Plug in the yellow connector of the Li-Ion into the yellow connector on the PCB on the robot. This should turn on the lights on the Arduino.

2. Plug in the two usb cables next to the power bank into the power bank. This should turn on the tinker board.

3. In order to check the status of the Li-ion plug in the Li-ion charging input into the 7-segment LED.

Lessons learned:
* Power Systems:
    * Initially, 2 weaker LiPo batteries were being used to power the tinker and the motors
    * These were not good enough as they did not allow nearly enough current draw for the motors (mainly)
    * The necessary current for the motors was between 6 and 20 amps (normal usage)
    * The supplied current earlier was about 4 A, which was not enough
    * I.e. lesson: always make sure your batteries can provide enough current for your motors
* LiDAR:
    * Initially, as mentioned, I tried to use python to read the data from the LiDAR via a third party library
    * This did not work out very well and the lesson I learned here is that for fast communication between software and hardware it is always best to use a lower level language such as C++
    * Most of the code w/ the python experiments are in the lidar_test folder
* Chassis:
    * Originally, a smaller chassis was used, approximately 8x10"
    * This original chassis would have worked, although likely would have been less stable as more layers would have to be built up to contain all the hardware
    * The larger chassis allowed for a better, more stable, and more contained design


Overview of File Structure:
The following is the structure of the files inside the workspace (catkin_ws/src):


This is all inside the src folder inside catkin_ws. This source folder contains most of the code for the project. The db_navigation folder consists of the code needed to enable communication between the ROS nodes using the TF tree. The hector_slam package consists of the code needed to do mapping for the robot. The rplidar_ros folder contains the necessary files to communicate with the lidar. Finally the motor_driver folder contains the files needed to communicate with the motors. 


Bill of Materials:






Other References


https://github.com/robopeak/rplidar_ros/wiki/How-to-use-rplidar
https://husarion.com/
https://github.com/mktk1117/six_wheel_robot/wiki/Make-a-ROS-package-to-communicate-with-Arduino-to-control-motors
https://www.dfrobot.com/blog-867.html
*There are a few more links saved on chromium on the tinker board


References for ROS packages used:
http://wiki.ros.org/laser_scan_matcher
http://wiki.ros.org/amcl
http://wiki.ros.org/hector_slam
http://wiki.ros.org/rplidar
http://wiki.ros.org/tf
