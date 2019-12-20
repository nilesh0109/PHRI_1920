# Scripts for creating movements. Motion team: Lara, Vadym, Christoper

## Believe movement (preliminary). 
See also a video. Take into consideration that V-REP simulation does not use the 
actual robot (e.g. V-REP simulated NICO has 3 fingers, a real NICO 4). Therefore,
adjustments will be done directly on the robot. But the main movements remain.

## My name movement (preliminary)
See video (https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/blob/motion/motion/my_name_movement.mp4) . The actual robot has a new wrist than a V-REP simulation. Therefore,
movement will be adjusted directly on a robot.

## Please use Slack channel 'motion' to discuss the movements.

## How to run the motion scripts
```
first terminal
1. roscore

2nd run the motion node
1. source ~/NICO-software/api/NICO-setup.bash
2. cd ~/NICO-software/api
3. copy scripts from github origin/motion to ~/NICO-software/api/src/nicoros/scripts
4. chmod +x {copied files}
5. catkin_make
6. source ~/NICO-software/api/devel/setup.sh (you dont need this if you already put on your bashrc)

3rd terminal (run the motion node)
1. source ~/.NICO/bin activate
2. sudo chmod 777 /dev/ttyACM*
3. sudo chmod 777 /dev/ttyUSB*
4. rosrun nicoros Motion.py -m 'NICO-software/json/nico_humanoid_upper_rh7d.json'

4th terminal (run the face expression node)
1. source ~/.NICO/bin activate
4. rosrun nicoros FaceExpression.py

another terminal (run the movement script)
1. source ~/.NICO/bin activate
2. rosrun nicoros {the movement script}
```
