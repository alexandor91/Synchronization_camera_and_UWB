# Synchronization_camera_and_UWB
This project is established for my assistant work  at TUM Navigation Lab, based on camera and UWB preception
===
Hardware requirements
====
1. Bumblebee2 BB2-08S2C
via FireWire-1394 standard
2. Decawave range finder with one anchor and onr tag
via SerialPort

===
Compile
===
1. cd catkin_ws
2. delete build, devel
3. Change all the absolute pathes within the source code according to your local path: /*/catkin_ws/src/filter_synchronizer1/src/synchronizer1.cpp
4. chmod +x catkin_ws/src/decawave_driver/src/dist_read.py
5. catkin_make
6. cd /*/catkin_ws/src/left_image_data/ 
   empty the folder manually
   cd /*/catkin_ws/src/right_image_data/ 
   empty the folder manually

===
Set-Up
===
7. cd catkin_ws/devel
8. source setup.bash
9. sudo chmod 666 /dev/ttyACM*: the enumeration should be the same as it is displayed.
check accordingly the port number in the launch file <param name="port" value="/dev/ttyACM0" type="string"/> in catkin_ws/src/read_dw_camera_dist.launch 
or
self.dwPort = rospy.get_param('~port','dev/ttyACM0') in catkin_ws/src/decawave_driver/src/dist_read.py
10. cd catkin_ws
11. Terminal1: roslaunch read_dw_camera_dist.launch
12. Termianl2: rosrun filter_synchronizer1 synchronizer_node1 /*/catkin_ws/src/filter_synchronizer1/src/bumblebee2.yaml
or 
./run.sh

===
Denotation
===
1. On Terminal1 the "image 0" corresponding to left camera image, "image 1" to right camera image, followed by the counter for the image pair, and absolute ros time, in between there would be "UW time" referred to the data from range finder followed by ros time and distance in m.
2. On Terminal2, "Ranging measurement [m]"should be printed out.

===
Storage
===
######Note the storing rate could be set:
cd /*/catkin_ws/src/filter_synchronizer1/src/, in synchronizer1.cpp "recording_image_rate" default as 2, which means the storage is implemented once every two images. Change it accordingly to  your application.

1. cd /*/catkin_ws/src/left_image_data/, the left images are stored here
2. cd /*/catkin_ws/src/right_image_data/, the right images are stored here
3. cd /*/catkin_ws/src/distance_data/, the distances are stored in the range.txt file
4. cd /*/catkin_ws/src/time_stamp/, the timestamps are stored in the time_stamp.txt file
---
Demo
---
![demo of the system](perception_and_wodstation.jpg)
<br />&emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp; &emsp; &emsp;  &emsp;  &emsp;&emsp; &emsp;  &emsp;  &emsp;Overall nodes flow<br />

