# PHRI1920_dev

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
    
### Initial Set-up
#### A general set-up process for two computers
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

#### A set-up for `LEFT` computer
1. On this computer run 4 services and one subscriber
    1. pose service
    2. face expression subscriber
    3. speech synthesis service
    4. speech recognition service
    5. vision service for counting cubes
2.  For each service/subscriber a new terminal is needed. The set-up process for 
each terminal:
    1. `export ROS_MASTER_URI=http://wtmpc23:11311/`
    2. `source ~/.NICO/bin/activate`
    3. `cd NICO-software/api`
    4. `source devel/setup.bash`
3. Start services:
    1. `rosrun nicopose poseService.py --label A --position LEFT`
    2. `rosrun nicopose fexSub.py --label A --position LEFT`
    3. `rosrun speech SpeechSynthesisStub.py A`
    4. `rosrun speech SpeechRecognitionStub.py`
    5. `rosrun vision count_resources.py`
    
#### A set-up for `RIGHT` computer
1. On this computer run 4 services and one subscriber
    1. pose service
    2. face expression subscriber
    3. speech synthesis service
    4. state machine
2.  For each service/subscriber a new terminal is needed. The set-up process for 
each terminal:
    1. `export ROS_MASTER_URI=http://wtmpc23:11311/`
    2. `source ~/.NICO/bin/activate`
    3. `cd NICO-software/api`
    4. `source devel/setup.bash`
3. Start services:
    1. `rosrun nicopose poseService.py --label B --position RIGHT`
    2. `rosrun nicopose fexSub.py --label B --position RIGHT`
    3. `rosrun speech SpeechSynthesisStub.py B`
    5. `rosrun state_machine StateMachine.py --scene 0 --entry 0`

### Ros launch scripts
You can use launch scripts to avoid a lot of typing, which is described in the previous section.
1. Launch nicopose `roslaunch nicopose pose.launch llabel:=A lnvc:=M`
2. Launch logging: `rostopic echo /rosout | grep msg`

# Experiment set-up. 
### Note, you need to change lnvc parameter according to your setup. Start terminal. In each terminal session execute the required commands:
1. Computer LEFT:
    1. Launch nicopose `roslaunch nicopose pose.launch llabel:=A lnvc:=M`
    2. Launch logging: `rostopic echo /rosout | grep msg`
    3. Execute `rosrun vision count_resources.py`
2. Computer RIGHT:
    1. Launch nicopose `roslaunch nicopose pose.launch llabel:=B lnvc:=S`
    2. Execute `rosrun speech SpeechRecognitionStub.py`
    3. Execute `rosservice call speech_recognition calibrate`
3. Computer iCub. Note, you must run `cd /data/home/hri/phri1920` and `source setup.bash in each session`
    1. Execute  `roscore`
    2. Execute `roslaunch state_machine lab_pc.launch` before participant enters
    3. Execute `rosrun state_machine StateMachine.py` when robots are supposed to start talking. 
       - optional parameters are e.g. :  --scene=0 --entry=0
    4. You can set the lights manually, if needed: `rosservice call /light_control "setting: 'cyan'"`


NB: 
In case of face expression is not recognized for B (right PC)
open a terminal
1. check which ttyACM "ls /dev/ttyACM" and tab
2. go to fexSub.py change specifically line 37, change to respective ttyACM

#### Links

[PHRI1920 Wiki](https://www2.informatik.uni-hamburg.de/wtm/wtm/wtmwiki/index.php/Project_Human_Robot_Interaction_2019-20_Set-up)
