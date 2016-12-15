Set-up Documentation for Snake Robot, using Raspberry Pi 3 (Ubuntu Mate) and Operator PC (Ubuntu)

RASPBERRY PI:

New terminal
$ ifconfig
get IP address (currently 192.168.15.109)
$ ssh <IP-address>
enter password -> rasppi3.
$ roscore

New terminal
$ cd /tmp
$ mkdir stream
$ cd ~/snake_robot
$ g++ picam_stream.cpp -o picam_stream -I/usr/local/include -lraspicam -lopencv_core -lopencv_highgui -lraspicam_cv
$ ./picam_stream

New terminal
$ cd mjpg-streamer-code-182/mjpg-streamer
$ LD_LIBRARY_PATH=./ ./mjpg_streamer -i "input_file.so -f /tmp/stream -n pic.jpg" -o "output_http.so -w ./www"

New terminal
$ cd ~/arduino-1.6.13
$ ./arduino
Open file 'Snake_Motion_Control.ino'
Upload to Arbotix

New terminal
$ ssh <IP-address>
enter password -> rasppi3.
$ export ROS_IP=<IP-address>
$ export ROS_MASTER_URI=http://<IP-address>:11311
$ cd ~/catkin_ws
$ catkin_make
$ rosrun snake_robot key_sub

OPERATOR COMPUTER:

Open web browser
http://<RASP-PI-IP-address>:8080/stream.html

New terminal
$ rospack find key_teleop
if the key_teleop package is install, skip to ifconfig
if not installed:
$ printenv | grep ROS
Note ROS distro, eg Kinetic, Indigo, Jade
$ sudo apt-get install ros-<distro>-key-teleop
$ ifconfig
get IP address (currently 192.168.15.104)
$ ssh <IP-address>
enter password -> student
$ export ROS_IP=<IP-address>
$ export ROS_MASTER_URI=http://<RASP-PI-IP-address>:11311
$ rosrun key_teleop key_teleop.py 

use arrow keys on PC to control snake robot
