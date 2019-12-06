#!/usr/bin/env python

import rospy
import smach
import yaml
from os.path import dirname, abspath
from smach_ros import ServiceState
from beginner_tutorials.srv import AddTwoInts,AddTwoIntsResponse

class SceneManager(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['say_next_sentence','ressource_allocation','scenes_finished'], output_keys=["speaker", "audio"])
        self.scene = 0
        self.scenes = ["scene1.yml"]
        self.load_dialog_script()

    def execute(self, userdata):
        rospy.loginfo('Executing state SCENE')
        if self.dialog < len(self.script):
            userdata.speaker = self.script[self.dialog]["speaker"]
            userdata.audio = self.script[self.dialog]["audio"]
            self.dialog += 1
            return 'say_next_sentence'
        elif self.scene < len(self.scenes):
            self.scene += 1
            if self.scene < len(self.scenes):
              self.load_dialog_script()
            return 'ressource_allocation'
        else:
            return 'scenes_finished'

    def load_dialog_script(self):
      self.dialog = 0
      with open(dirname(abspath(__file__)) + "/" + self.scenes[self.scene], "r") as f:
        self.script = yaml.safe_load(f)
        

class Speak(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['speech_done'], input_keys=["speaker", "audio"])

    def execute(self, userdata):
        rospy.loginfo('Executing state SPEAK')
        rospy.loginfo(userdata.speaker)
        rospy.loginfo(userdata.audio)
        return 'speech_done'
        



# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['experiment_ended', 'preempted', 'aborted'])
    sm.userdata.speaker = ""
    sm.userdata.audio = ""
    sm.userdata.a = 1
    sm.userdata.b = 1

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SCENE', SceneManager(), 
                               transitions={'say_next_sentence':'SPEAK', 
                                            'ressource_allocation':'ADD_TWO_INTS',
                                            'scenes_finished':'experiment_ended'})
        smach.StateMachine.add('SPEAK', Speak(), 
                               transitions={'speech_done':'SCENE'})
                             
        # Callback for service response  
        def add_two_result_cb(userdata, response):
            rospy.loginfo("Result: %i" % response.sum)
            return 'succeeded'
        smach.StateMachine.add('ADD_TWO_INTS',
                           ServiceState('/add_two_ints',
                                        AddTwoInts,
                                        response_cb=add_two_result_cb,
                                        request_slots = ['a', 'b']),
                           transitions={'succeeded':'SCENE'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()

