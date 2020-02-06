import rospy
import sys
from os.path import dirname, abspath
import wave
import time

import speech_recognition as sr

import docks2_remote.docks2_remote.postprocessor as postprocessor
from docks2_remote import Client


class SentenceList:
    # Class Constants:
    base_dir = dirname(dirname(abspath(__file__)))
    protocols = {"done": "done",  # maps the context to the corresponding mission/emergency protocol
                 "scene_0": "mission", 
                 "scene_1": "emergency", "scene_2": "emergency", "scene_3": "emergency", "scene_4": "emergency"}
    sentences = {}  # the possible sentences of the mission/emergency protocol

    def __init__(self):
        # Server settings from Johannes Twiefel: accessible only from the Informatikum network
        self.client = Client(server='sysadmin@wtmitx1', port=55101)
        self.listener = sr.Recognizer()
        # Filter ambient lab noise using a previously recorded sound file
        with sr.AudioFile(self.base_dir + "/recorded_sounds/lab_noise.wav") as noise:
            self.listener.adjust_for_ambient_noise(noise, duration=3)

    def initialize(self):
        # Create the sentencelist postprocessors on the Docks server
        with self.client.connect() as connection:
            for protocol in set(self.protocols.viewvalues()):
                rospy.loginfo("Creating postprocessor for \'%s\'", protocol)
                sentence_list = open(self.base_dir + "/scripts/{}.sentences.txt".format(protocol)).readlines()
                connection.create_postprocessor(postprocessor.SentencelistPostprocessor,
                                                '{}_sentencelist_postprocessor'.format(protocol),
                                                sentencelist=sentence_list, language="english", language_code="en-EN")
                # Store content of the sentence_list to fetch the index later
                self.sentences[protocol] = sentence_list

    def calibrate(self):
        # Filter ambient lab noise
        with sr.Microphone() as noise:
            self.listener.adjust_for_ambient_noise(noise, duration=3)

    def configure(self, context):
        """
        :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
        :return: none; only updates the internal state
        """
        # Setting the context specific parameters:
        # - confidence_threshold: how good the understood sentence matches the sentence from the list (range: [0, 1])
        # - phrase_threshold: minimum secs of speaking before we consider it a phrase (values before are discarded)
        # - pause_threshold: seconds of non-speaking audio before a phrase is considered complete
        # - silence_timeout: seconds before an error is raised when no speech is recorded.
        self.context = context
        self.post_processor = "{}_sentencelist_postprocessor".format(self.protocols[context])
        if context == "done":
            self.listener.phrase_threshold = 1
            self.listener.pause_threshold = 1

            self.context_sentences = 1
            self.confidence_threshold = 0.3
            self.silence_timeout = 60
        else:
            self.listener.phrase_threshold = 1
            self.listener.pause_threshold = 1.5

            if context == "scene_0" or context == "scene_1":
                self.context_sentences = 4
            else:
                self.context_sentences = 3

            self.confidence_threshold = 0.5
            self.silence_timeout = 10


        if context == "scene_0" or context == "scene_1":
            self.context_sentences = 4

    def recognize(self):
        with sr.Microphone() as source:
            rospy.loginfo("\n--------------------- Listening for Microphone Input -------------------")
            try:
                # Collect raw audio from microphone.
                audio_data = self.listener.listen(source, timeout=self.silence_timeout)

                # storing audio file
                time_stamp = time.strftime("%m%d-%H%M%S", time.gmtime())
                file_path = self.base_dir + '/recorded_sounds/{}_{}.wav'.format(self.context, time_stamp)
                with open(file_path, "wb") as f:
                    f.write(audio_data.get_wav_data())
                rospy.loginfo("\n--------------------- Sending recording to Docks -------------------")
                with self.client.connect() as connection:
                    # Transform the audio into a string
                    hypotheses, _ = connection.recognize(audio_data, ['ds', 'greedy'])
                    rospy.loginfo("Docks2 understood: %s", hypotheses.lower())
                    # Match the understood sentence to the best candidate from the sentence list
                    return connection.postprocess(self.post_processor, hypotheses)
            except sr.WaitTimeoutError as e:  # throws when "silence_timeout" is exceeded
                rospy.loginfo("Timeout: %s", e)
                return None, 0  # => will be turned into 'repetition_request' / 'timeout'
            except BaseException as e:
                 rospy.loginfo("Error occured in recognition: %s", e)
                 return "fallback", 0 # => need for fallback.

    def match_sentence(self, docks_hypotheses, confidence):
        """
        :param docks_hypotheses: sentence the docks postprocessor suggested.
        :param confidence: how sure the postprocessor is about the hypothesis (range: [0, 1])
        :return: sentence_id: The id of the sentence that was recognized.
        """
        # Unpredicted error occured, fallback needed.
        if docks_hypotheses == "fallback":
            return "fallback"

        # No need to match when below confidence threshold
        if confidence <= self.confidence_threshold:
            rospy.loginfo("Recognized with confidence %s:\n \'%s\'", confidence, docks_hypotheses)
            return "repetition_request" if self.context == "done" else "timeout"

        # Extracting sentence_id from recognition data.
        matched_line = self.sentences[self.protocols[self.context]].index(docks_hypotheses + "\n")
        rospy.loginfo("Recognized line number %s, %s with confidence %s.", matched_line, docks_hypotheses, confidence)
        if self.context == "done" and matched_line == 0:
            return "done_confirmation"
        elif matched_line < self.context_sentences:
            return self.context + "_question_" + str(matched_line)
        else:
            return "repetition_request"

if __name__ == "__main__":
    processor = SentenceList()
    processor.initialize()
    processor.configure(sys.argv[1])
    sentence = processor.match_sentence(processor.recognize())
    print(sentence)
