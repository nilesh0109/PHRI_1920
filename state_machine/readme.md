- press 1 on the dvi switch
- blackbox program. restart the program and load the most recent xml calibration (the one with xml in its name, NOT the bin one)
- press 2 on the dvi switch
- for each terminal/tab used (tabs should still be open unless pc was shut down):
```
cd /data/home/hri/phri1920
source setup.bash
```
## for each experiment
- make sure the blackout button on the light control panel is enabled (yellow led is flickering)
```
roslaunch state_machine lab_pc.launch
```
RESTART AFTER EACH EXPERIMENT!!!

## start the experiment by
```
rosrun state_machine StateMachine.py
```
## Wizard of Oz Stuff

There are some fallback states which require experimenter input. A GUI will pop up on the screen if input is necessary.

To start in a different scene, change scene and entry (see printout TODO update printout):

```
rosrun state_machine StateMachine.py --scene=0 --entry=0
```

If you need to manually set the lights, use (other options are 'alarm' and 'blackout'):
```
rosservice call /light_control "setting: 'cyan'"
```