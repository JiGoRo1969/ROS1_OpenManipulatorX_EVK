
# Demonstration of ROS 2 communication based on the Micro-XRCE-DDS-Client implementation for Renesas RZ/A2M

日本語版は[こちら](./README.jp.md)

# Demonstration Overview
This demonstration implements the robot arm operation by MoveIt!, using the evaluation board SEMB-1451/1452 equipped with Renesas RZ/A2M MPU on which the Micro-XRCE-DDS-Client implementation runs.

# 1. Demonstration environment
The items listed below are needed for the demonstration.

### [Hardware]  
(1) SEMB-1451/1452 board (1set)  
The target board on which the created image runs.

SEMB-1451/1452 is provided by Shimafuji in Japan.
  
JPN : http://www.shimafuji.co.jp/products/1767
  
ENG : http://www.shimafuji.co.jp/en/products/1522


(2) Windows PC  
Used to write the firmware to the target SEMB-1451/1452 board.  

(3) Linux PC  
Used to run the ROS/ROS 2 environment.  
We have confirmed this demonstration works correctly in Ubuntu 18.04 LTS.  

(4) J-Link, USB cable  
Used to write the firmware.  

(5) Ethernet HUB  
Used to make a local network between Linux and the SEMB-1451/1452.  

(6) Two Ethernet cables  
Used to make a local network between Linux and the SEMB-1451/1452.  

(7) OpenMANIPULATOR-X  
OpenMANIPULATOR-X is the robot arm of ROBOTIS ("Robot Arm").  

In this demonstration, devices (1) through (7) described above need to be connected as shown below:  
```
                         +---------------+                 +--------------+
       +-----------------+     J-Link    |                 |  Linux PC *2 |
       |                 +-------+-------+                 +------+-------+
       | USB                     | JTAG                           | Ethernet
       |                         |                                | 192.168.2.101
+------+-------+         +-------+--------+    Ethernet    +------+-------+
|  Windows PC  +---------+ SEMB-1451/1452 +----------------+ Ethernet HUB |
+--------------+  UART   +-------+--------+  192.168.2.52  +--------------+
                (SCIFA4)         |
                   *1            |      UART(SCIFA2)*3     +--------------+
                                 +-------------------------+  Robot Arm   |
                                                           +--------------+
```

*1. Connect USB serial cable to CN17 [baud rate: 115200].  
*2. Connect Linux to local network only. Do not connect Linux to global network (internet). Disable IPv6 setting in Linux.  
*3. Make connections as below. Make sure pin 1 is unconnected.  
- GND (pin 4, closest to silkscreen printing of "CN15")  
- 12V (pin 3)  
- Signal (pin 2)  

### [Software]  

#### Linux PC
It is necessary to install the software listed below on Linux PC.  

(1) ROS (Melodic)  
Regarding the installation process, please refer to the following web page.  
http://wiki.ros.org/melodic/Installation/Ubuntu

(2) ROS 2 (Crystal Clemmys)  
Regarding the installation process, please refer to the following web page.  
https://index.ros.org/doc/ros2/Installation/Crystal/Linux-Install-Debians/  

(3) Micro XRCE-DDS-Agent  
Regarding the installation process, please follow the instructions in chapter [2. Build Micro XRCE-DDS-Agent]. 
The version used for this demonstration is v1.1.0. 

(4) App to control robot model  
A ZIP archive named "ROS1_OpenManipulatorX_EVK-master.zip"(download from this repository). How to use this archive is described in section [4.1 Extract source code].
Download this archive from the URL shown below. It must be placed in the home directory.  

https://github.com/JiGoRo1969/ROS1_OpenManipulatorX_EVK/archive/master.zip

#### Windows PC
It is necessary to install the software listed below on Windows PC.  

(1) e2studio  
Regarding the installation process, please refer to documents on the web page below.  
We have confirmed that e2studio 7.5.0 works correctly.  
https://www.renesas.com/us/en/products/software-tools/tools/ide/e2studio.html

(2) FreeRTOS source code  
A ZIP archive named "/ROS2_RZA2M_MoveIt1-master.zip". How to use this archive is described in section [5.1 Import project]  
Download this archive from the URL shown below. It must be placed in home directory.  

https://github.com/JiGoRo1969/ROS2_RZA2M_MoveIt1/archive/master.zip


# 2. Building the Micro XRCE-DDS-Agent  

## 2.1. Get the source code  
Input the commands below into a terminal.  
```
$ cd ~
$ git clone -b v1.1.0 https://github.com/JiGoRo1969/Micro-XRCE-DDS-Agent
$ cd Micro-XRCE-DDS-Agent
$ git submodule update --init --recursive
```


## 2.2. Install spdlog
To build the Micro XRCE-DDS-Agent, create the development environment as shown below.
```
$ cd ~/Micro-XRCE-DDS-Agent
$ pushd thirdparty/spdlog/
$ mkdir build; cd build
$ cmake -DSPDLOG_BUILD_EXAMPLES=OFF -DSPDLOG_BUILD_BENCH=OFF -DSPDLOG_BUILD_TESTS=OFF ..
$ sudo make install
$ popd
```

## 2.3. Install CLI11
To build the Micro XRCE-DDS-Agent, create the development environment as shown below.
```
$ cd ~/Micro-XRCE-DDS-Agent
$ pushd thirdparty/CLI11/
$ mkdir build; cd build
$ cmake -DCLI11_TESTING=OFF -DCLI11_EXAMPLES=OFF ..
$ sudo make install
$ popd
```

## 2.4. Build and Install
Input the commands below into a terminal.  
```
$ cd ~/Micro-XRCE-DDS-Agent
$ mkdir build; cd build
$ cmake ..
$ make
$ sudo make install
$ sudo ldconfig /usr/local/lib
```

# 3. Steps to construct the development environment on Linux  

## 3.1. Install packages related to MoveIt!
After you have installed ROS(melodic), run the commands below.  
```
$ sudo apt install ros-melodic-ros-control ros-melodic-ros-controllers
$ sudo apt install ros-melodic-industrial-trajectory-filters
$ sudo apt install ros-melodic-moveit
```
After you have run the commands above, initialize rosdep.    

## 3.2. Install ROS1-Bridge
Input the commands below into a terminal.  
```
$ sudo apt install ros-crystal-ros1-bridge
```

# 4. Building the app to control the robot model

## 4.1. Extract source code
To extract ROS1_OpenManipulatorX_EVK-master.zip which has already been downloaded to the home directory,  
input the commands below into a terminal.  
```
$ source /opt/ros/melodic/setup.bash
$ cd ~
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ unzip ~/ROS1_OpenManipulatorX_EVK-master.zip
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```

## 4.2. Build application
Input the commands below into a terminal.  
```
$ cd ~/catkin_ws/
$ catkin_make
```

# 5. Steps to build the firmware on the Windows PC

## 5.1. Import Project
After launching e2studio, import ROS2_RZA2M_MoveIt1-master.zip via "File -> Import".  
In this dialog, only the project "ROS2_RZA2M_MoveIt1-master.zip_expanded\ROS2_RZA2M_MoveIt1-master\demos\renesas\rza2m-ebk\e2studio" must be checked.  

**"ROS2_RZA2M_MoveIt1-master.zip" needs to be put onto the root directory such as "C:\".**

## 5.2. Modify project settings and build
Perform the steps listed below.  
(1) Right click on the project explorer and select "properties".  
(2) Select "Settings" for C/C++ build, and there select "Cross ARM C++ Compiler" as "Preprocessor".  
(3) Modify "Defined Symbol" as below.  

```
USING_UXR
USING_FIXED_IPADDR
DDS_TRANSPORT=DDS_TRANSPORT_UDP
CLIENT_STREAMS=CLIENT_STREAMS_RELIABLE
```

Save modifications and start the build.    

## 5.3. Write firmware to target board
Perform the steps listed below.   
(1) Connect J-Link to PC via USB before turning on the target.  
(2) Turn on the target, and then select "Run" -> "Debug" in e2studio.   
* Select "Renesas GDB Hardware Debugging", and press "OK"  
* Select "J-Link ARM", and press "OK" 
* Select "R7S921053", and press "OK"  

(3) After writing to the target has finished, exit e2studio and disconnect J-Link from the target.    

# 6. Steps to setup the environment for controlling the robot arm

## 6.1. Linux side

**make sure that .bashrc does not call "source /opt/ros/\<package\>/setup.bash" and does not call "source ~/catkin_ws/devel/setup.bash".**  

Open four terminals (A-D) and run the commands below.  

[Terminal A]  
```
$ MicroXRCEAgent udp --port 2020 --discovery
```

[Terminal B]  
```
$ source /opt/ros/crystal/setup.bash
$ source /opt/ros/melodic/setup.bash
$ ros2 run ros1_bridge dynamic_bridge
```

[Terminal C]  
```
$ source /opt/ros/melodic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch open_manipulator_evk_moveit demo.launch
```

[Terminal D]  
When it is necessary to coordinate with Gazebo, run the commands below.  
```
$ source /opt/ros/melodic/setup.bash
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch open_manipulator_evk_gazebo open_manipulator_gazebo.launch
```

When terminal B launches the following error messages may appear. 
There is, however, no problem with continuing operation.  
```
[ERROR] [1567752565.159206520]: [registerPublisher] Failed to contact master at [localhost:11311]. Retrying…
```

```
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
```

## 6.2. SEMB-1451/1452 side
Please turn on power.  

## 6.3. How to check whether it has launched correctly or not  
These messages below will be displayed on terminal A in Linux.
```
[1565169949.224506] info     | DiscoveryServerLinux.cpp | init                     | running...             | Port: 7400
[1565169949.224969] info     | UDPServerLinux.cpp | init                     | running...             | port: 2020
[1565170101.307103] info     | Root.cpp           | create_client            | create                 | client_key: 0xAAAABBBB, session_id: 0x81
[1565170101.307270] info     | UDPServerBase.cpp  | on_create_client         | session established    | client_key: 0xAAAABBBB, address: 192.168.2.52:8983  → Having client_key with 0xAAAABBBB, publisher is connected.
[1565170106.425678] info     | Root.cpp           | create_client            | create                 | client_key: 0xCCCCDDDD, session_id: 0x81
[1565170106.425794] info     | UDPServerBase.cpp  | on_create_client         | session established    | client_key: 0xCCCCDDDD, address: 192.168.2.52:33168　→ Having client_key with 0xCCCCDDDD, subscriber is connected.
```
In addition, you can confirm the the ROS 1 <=> ROS 2 bridge operation by opening another terminal and inputting the commands below.  

```
$ source /opt/ros/crystal/setup.bash
$ ros2 topic list
/parameter_events
/open_manipulator_evk/demo_arm_srv_pos_down  → subscriber topic name
/open_manipulator_evk/demo_arm_srv_pos_up  → publisher topic name
/rosout
```

# 7. How to operate

## 7.1. Operation method for MoveIt!  
By using MoveIt!, which was launched on terminal C, you can control the robot arm's posture. 
To control the robot arm, please follow the steps below.

(1) Enable the checkbox "Allow Approx IK Solutions" in the tab "MotionPlanning - Planning"  
 → Then, you can drag the interactive marker (blue sphere) in the robot model to specify the goal posture

(2) After setting the goal posture, click the button "Plan and Execute" to move the robot arm 

(3) If the communication between MoveIt! and the robot arm is established, the robot arm will change its posture to the goal posture

\[Caution\] Don't choose any postures in which any part of the ARM is beneath the plane on which the arm is fixated.

\[Note\] Dragging the interactive marker too much may cause MoveIt! (rviz) to abort or freeze. 
In that case, perform the the steps below to restart all terminals.

(1) Press Ctrl+C on all the terminals (A-D), and be sure that all the processes in each terminal have terminated.  

(2) Redo the steps as outlined in [6.1. Linux side] in each terminal.

# 8 About the ROS topics

## 8.1. System structure
This system structure is as shown below.

```
                   +-----------------+            +-----------------+
                   |     MovevIt!    |            |     Gazebo      |
                   +-----------------+            +-----------------+
                       |        ↑                          ↑
                    1. |        | 2.                       |
                       ↓        |                          |
               +------------------------+         7.       |
               | arm_controller_adaptor |------------------+
               +------------------------+
                       |        ↑
                    3. |        | 4.
                       ↓        |
                  +-----------------+
                  |   ros1_bridge   |
                  +-----------------+
                       |        ↑
                    5. |        | 6.
                       ↓        |
                  +-----------------+              +-----------------+
                  |  XRCE DDS Agent |              |    Robot Arm    |
                  +-----------------+              +-----------------+
                       |        ↑                      ↑         |
                       |        |                      |         |
                       ↓        |                      |         ↓
                  +-----------------+              +-----------------+
                  | XRCE DDS Client |←------------→| ROS 2 Listener  |
                  +-----------------+              +-----------------+

```
</br>

## 8.2. Topic details
The table below contains information regarding the topics from No.1 through 7, as described in the section on the system structure [8.1] above.

| No. | Topic name | message type | content |
|:----------:|:-----------|:------------|:------------|
| 1. | /open_manipulator_evk<br>/open_manipulator_evk_joint_controller<br>/follow_joint_trajectory/goal | control_msgs<br>/FollowJointTrajectoryActionGoal | Sequence of goals in a plan sent by MoveIt! (position, speed, acceleration, current). |  
| 2. |  /open_manipulator_evk<br>/open_manipulator_evk_joint_controller<br>/follow_joint_trajectory/feedback | control_msgs<br>/FollowJointTrajectoryActionFeedback  | Status of each servo sent by ROS 2 Listener. |  
| 3. | /open_manipulator_evk<br>/demo_arm_srv_pos_down | std_msgs/Float64MultiArray | Servo status for each goal in a plan sent by MoveIt!. This status is converted from control_msgs/FollowJointTrajectoryActionGoal to std_msgs/Float64MultiArray, being added the time stamp of each goal. |  
| 4. | /open_manipulator_evk<br>/demo_arm_srv_pos_up | std_msgs/Float64MultiArray | Forwarded message of No.6 below across ros1_bridge. |  
| 5. | /open_manipulator_evk<br>/demo_arm_srv_pos_down | std_msgs/Float64MultiArray | Forwarded message of No.3 above across ros1_bridge. (ROS 2 topic) |  
| 6. | /open_manipulator_evk<br>/demo_arm_srv_pos_up | std_msgs/Float64MultiArray | Servo status publised by the ROS 2 Listener. (ROS 2 topic) |  
| 7. | /open_manipulator_evk<br>/joint*_position/command | std_msgs/Float64 | Servo status forwarded from No.4 above by arm_controller_adaptor.  |  


## 8.3. Data structure of std_msgs/Float64MultiArray
The data structure of std_msgs/Float64MultiArray is shown below.  

| IDX | content | note |
|:----------:|:-----------|:------------|
| 0   | time for Servo position         | unit: nano sec//planning time on MoveIt!  
| 1   | servo 1 angular coordinates     | unit: radian|  
| ... | ...        | ...|  
| 5   | servo 5 angular coordinates     | unit: radian|  
| 6   | servo 1 angular velocity        | unit: radian/s|  
| ... | ...        | ...|  
| 10  | servo 5 angular velocity        | unit: radian/s|  
| 11  | servo 1 angular acceleration    | unit: radian/s^2|  
| ... | ...        | ...|  
| 15  | servo 5 angular acceleration    | unit: radian/s^2|  
| 16  | servo 1 power consumption       | unit: %（unused）|  
| ... | ...        | ...|  
| 20  | servo 5 power consumption       | unit: %（unused）|  

</br>

# Restriction

* Controlling the Gripper via MoveIt! is not supported.  
  Currently, arm_controller_adaptor will always output default values to the Robot Arm.  
----
