# PHRI1920_dev

This repository is for anything that is code and directly tied to development.

Launch Vision service
 
    rosrun vision count_resources.py robotA
    rosrun vision count_resources.py robotB

We can call service with any robot name A or B. Both will be called together

Service call

    rosservice call /robotA/count_objects participant_number
    - participant_number is integer starting from 1

    rosservice call /robotA/check_empty 0
    - 0 is blocked for service to check if the table in front of the robots is empty or not


* In count_resources.py --> check_empty()
    
   - The automatic timeout is set for 30s can be changed accordingly


* if the same participant_number is provided again then the folder will be created with a time stamp prefix with that participant id
## Links ##

[PHRI1920 Wiki](https://www2.informatik.uni-hamburg.de/WTM/wtm/wtmwiki/index.php/Category:Project_Human_Robot_Interaction_2019-20)

## Command line instructions ##
Git global setup

    git config --global user.name "<your name>"
    git config --global user.email "<your email address"



