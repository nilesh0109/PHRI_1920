#!/usr/bin/env python
import sys
from os.path import dirname, abspath

import rospy
from speech.srv import CubeCounting, CubeCountingResponse
from AudioPlayer import AudioPlayer

def get_audio_filename(scene_id, cubesA, cubesB):

    rospy.loginfo("scene_id={} cubesA={} cubesB={}".format(scene_id,cubesA, cubesB))
    print('scene_id={} cubesA={} cubesB={}'.format(scene_id,cubesA, cubesB))

    STRONG_THRESHOLD = 5
    if  0<= scene_id < 5:
        filename = 'feedback_scene_'+str(scene_id)
        if cubesA >= STRONG_THRESHOLD:
            filename += '_strong_A'
        elif cubesB >= STRONG_THRESHOLD:
            filename += '_strong_B'
        elif cubesA > cubesB:
            filename += '_win_A'
        elif cubesA < cubesB:
            filename += '_win_B'
        elif cubesA == cubesB:
            filename += '_equal'
        return filename

def handle_cube_counting(req):
    speaker = 'S'
    player = AudioPlayer(speaker)
    filename = get_audio_filename(req.scene_id, req.num_cubes_A, req.num_cubes_B)
    rospy.loginfo('feedback is {:s}'.format(filename))
    if filename:
        filepath = dirname(dirname(abspath(__file__))) + '/generated_sounds/{}.wav'.format(filename)
        player.play(filepath, filename)
        rospy.loginfo("Finished feedback")
    return CubeCountingResponse(True)

def cube_counting_server():
    rospy.init_node('cube_counting_server', anonymous=True)
    rospy.Service('cube_counting', CubeCounting, handle_cube_counting)
    rospy.loginfo("Cube Counting launched.")
    rospy.spin()

if __name__ == "__main__":
    cube_counting_server()
