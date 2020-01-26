### Introduction
1. We have two NICO robots: `A` and `B`
2. Each robot can have `LEFT` or `RIGHT` position. We use the robot's perspective,
not a human. `LEFT` position means that a robot on the right side of a commander
3. See markings on the floor for placing robots and a table correctly
4. Each robot runs on its own computer. You need `sudo` access
5. `LEFT` robot runs on `LEFT` computer (operator's perspective)

### Pre-check
1. Make sure that both robots are connected to the power source. Motors in a relaxed
state produce noise, you can here it. One NICO needs two power sockets and 
the other only one.
2. Connect all USBs and audio cables to PCs.
    1. Check if faces are lighted up
    2. Check if speakers on. Check the sound settings on a PC.

### Set-up
1. `git clone https://git.informatik.uni-hamburg.de/wtm-robots-and-equipment/NICO-software.git`
2. `cd NICO-software/api`
3. `source NICO-setup.bash`
4. `cd NICO-software/api/src`
5. `git clone https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev.git`
6. `cd NICO-software/api`
7. `catkin_make`
8. `source devel/setup.bash`
9. `sudo chmod 777 /dev/ttyACM*`
10. `sudo chmod 777 /dev/ttyUSB*`

### Start services (example):
1. `rosrun nicopose poseService.py --label A --position LEFT`
2. `rosrun nicopose fexSub.py --label B --position RIGHT`