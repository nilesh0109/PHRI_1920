## Running the state machine

Make sure all other services are running, then use:

```
rosrun state_machine StateMachine.py
```

If you want to skip parts of the script, you can define a new entry point (refer to printed script for ids) e.g.:

```
rosrun state_machine StateMachine.py --scene=2 --entry=4
```
