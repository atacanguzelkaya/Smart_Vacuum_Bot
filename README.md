# Smart_Vacuum_Bot
In this project, Istanbul Bilgi University Faculty of Engineering and Natural Sciences Mechatronics Engineering students developed an innovative robotic vacuuming system. The system consists of artificial intelligence. Sweeps dust and dirt from floor surfaces, including corners of a room, under cabinets or behind tables and other furniture. The robot calculates an efficient way to vacuum dust and dirt from parts of the room using the Robot Operating System (ROS), based on the map information transmitted by Lidar and the distance sensor. The vacuum system is programmed using ROS under Linux Ubuntu. Path planning algorithms allow them to navigate around obstacles. The autonomous robot is mainly designed for cleaning areas such as hotel rooms, home rooms.

# PROTOTYPE MATERIALS
• Raspberry PI 4 Model B

• Arduino Uno

• Step Motors

• DC Motor with Encoders

• Motor Drivers

• Vacuum Dc Motor

• Lidar

• Li-Po Battery

# PI OS AND MELODIC ON RASPBERRY PI 4
Download the image of Pi Os with Raspberry Pi Imager and Flash the microSD card using. Attach the monitor and keyboard to the board. Insert the microSD card into your Raspberry Pi 4. Power it up. 
# Setup ROS Repositories
First install repository key:

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

Now, make sure your Debian package index is up-to-date:

$ sudo apt-get update

$ sudo apt-get upgrade

# Initializing rosdep
$ sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
Initializing rosdep

$ sudo rosdep init

$ rosdep update

Now, we will download and build ROS Melodic.

# Create a catkin Workspace
In order to build the core packages, you will need a catkin workspace. Create one now:

$ mkdir -p ~/ros_catkin_ws

$ cd ~/ros_catkin_ws

Next we will want to fetch the core packages so we can build them. We will use wstool for this. Select the wstool command for the particular variant you want to install:
Desktop: ROS, rqt, rviz, and robot-generic libraries

$ rosinstall_generator desktop --rosdistro melodic --deps --wet-only --tar > melodic-desktop-wet.rosinstall

$ wstool init src melodic-desktop-wet.rosinstall

# Resolve Dependencies
Before you can build your catkin workspace, you need to make sure that you have all the required dependencies. We use the rosdep tool for this.
Resolving Dependencies with rosdep
The dependencies should be resolved by running rosdep:

$ cd ~/ros_catkin_ws

$ rosdep install -y --from-paths src --ignore-src --rosdistro melodic

This will look at all of the packages in the src directory and find all of the dependencies they have. Then it will recursively install the dependencies.
Invoke catkin_make_isolated:

$ sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic

Now ROS should be installed! Remember to source the new installation:

$ source /opt/ros/melodic/setup.bash

Or optionally source the setup.bash in the ~/.bashrc, so that ROS environment variables are automatically added to your bash session every time a new shell is launched:

$ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

At this point, we are ready to start the work. The first thing is to create work space to build your packages to run on the robot:

$mkdir -p ~/catkin_ws/src

$cd ~/catkin_ws/

$catkin_make

$echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc 

# ARDUINO IDE AND ROSSERIAL
# Install Arduino IDE
If we check the architecture of a Raspberry Pi 4, we can see that the Model B Rev 1.2 is working on 32 bits. This information is needed to download the proper software version of the Arduino IDE.

Step 1: Download the 32 bits version for Linux ARM.

This step may require some minutes to download the file. Navigate to the Download directory and unzip the archive.

Step 2: Open a terminal and navigate to the Download directory, then go to Arduino directory and run the command:

$sudo mv arduino-1.8.19 /opt/arduino-1.8.19

$cd ~

$cd /opt/arduino-1.8.19

$sudo ./install.sh

At this point, you have finished the installation of the Arduino IDE on Raspberry Pi. The next step is to install rosserial.

# Install Rosserial from Source
Step 1: Clone rosserial from the GitHub repository:

$cd catkin_ws/src

$git clone https://github.com/ros-drivers/rosserial.git

Step 2: Generate the rosserial_msgs needed for communication:

$cd ~/catkin_ws/

$catkin_make

Step 3: Make the library files in the catkin_ws/devel/lib directory

$catkin_make install

Step 4: We need to source a setup.bash file in every single terminal that we will run ROS commands. These scripts control many important ROS environment variables and non-ROS environment variables.

$cd ~

$source ~/catkin_ws/install/setup.bash

Step 5: This command is to import the rosserial library into Arduino IDE.

$cd Arduino/libraries

$rosrun rosserial_arduino make_libraries.py .

With all of the above steps, we finish the installation of the Arduino IDE and the rosserial library.

![Vacuum_Bot_Final_View](https://github.com/atacanguzelkaya/Smart_Vacuum_Bot/blob/c1a60c6aa183407fc15d0f8778912aeb5c57d0e1/vacuum_bot/Charts%20and%20Pictures/Vacuum_Bot.png)

# REFERANCES

$ Dragos Calin, April 15, 2020, https://www.intorobotics.com/how-to-install-ros-melodic-rosserial-and-more-on-raspberry-pi-4-raspbian-buster/

$ A guest post by Liang Bao, Chengqi Lv, from Ecovacs Robotics AI Department, January 07, 2020, https://blog.tensorflow.org/2020/01/ecovacs-robotics-ai-robotic-vacuum.html

$ Michael Rose, 2017, https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

$ Tully Foote, March 25, 2020, http://wiki.ros.org/melodic/Installation/Ubuntu
