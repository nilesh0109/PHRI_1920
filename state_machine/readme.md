# Protocol
- start projector setup:
  - press 1 on the dvi switch to switch to windows
  - (re)start the blackbox program and load the (second)most recent xml 	calibration (the one with xml in its name, NOT the bin one)
  - press 2 on the dvi switch to go back to linux

- For EACH terminal/tab you open, do:

```
cd /data/home/hri/phri1920
source setup.bash
```

- open roscore in one terminal:

```
roscore
```

## for each experiment
- make sure the blackout button on the light control panel is enabled (yellow led is flickering)
- (in a new terminal) start lab_pc services by:

```
roslaunch state_machine lab_pc.launch
```

RESTART AFTER EACH EXPERIMENT!!!

- these services should now be available (rosservice list):
  - /S/speech_synthesis
  - /cube_counting
  - /light_control
  - /play_video

## start the experiment
- make sure all the services are running
  - restart lab_pc.launch and motion services before each experiment!
- run the state machine (this immediately starts the robots introducing themselves):

```
rosrun state_machine StateMachine.py
```

## Wizard of Oz Stuff
To start in a different scene, change scene and entry (see printout TODO update printout):

```
rosrun state_machine StateMachine.py --scene=0 --entry=0
```

If you need to manually set the lights, use (other options are 'alarm' and 'blackout'):
```
rosservice call /light_control "setting: 'cyan'"
```