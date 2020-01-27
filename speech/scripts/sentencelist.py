import rospy
import sys
from os.path import dirname, abspath

import speech_recognition as sr

import docks2_remote.docks2_remote.postprocessor as postprocessor
from docks2_remote import Client


class SentenceList:
    # Class Constants
    base_dir = dirname(dirname(abspath(__file__)))
    sentences_dir = base_dir + '/scripts/'
    sentences = ["done", "mission", "emergency"]

    def __init__(self):
        # server settings from Johannes Twiefel: accessible only from the Informatikum network
        self.client = Client(server='sysadmin@wtmitx1', port=55101)
        # The main recognizing unit.
        self.listener = sr.Recognizer()
        # filter ambient lab noise using a previously recorded sound file
        with sr.AudioFile(self.base_dir + "/generated_sounds/lab_noise.wav") as noise:
            self.listener.adjust_for_ambient_noise(noise, duration=3)
        # Create postprocessors on the server
        with self.client.connect() as connection:
            for context in self.sentences:
                sentence_list = open(self.sentences_dir + "{}.sentences.txt".format(context)).readlines()
                connection.create_postprocessor(postprocessor.SentencelistPostprocessor,
                                                '{}_sentencelist_postprocessor'.format(context),
                                                sentencelist=sentence_list, language="english", language_code="en-EN")

    def configure(self, context):
        return

    def recognize(self, context):
        """
        :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
        :return: sentence_id: The id of the sentence that was recognized.
        """
        sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

        # Translation from the context id to values of parameters that define the context.
        if context == "done":
            sentence_list = open(sentences_dir + "done.sentences.txt").readlines()
            context_sentences = 1
            postprocessor = 'done_sentencelist_postprocessor'
        elif context == "scene_0" or context == "scene_1":
            sentence_list = open(sentences_dir + "mission.sentences.txt").readlines()
            context_sentences = 4
            postprocessor = "mission_sentencelist_postprocessor"
        else:
            sentence_list = open(sentences_dir + "emergency.sentences.txt").readlines()
            context_sentences = 3
            postprocessor = "emergency_sentencelist_postprocessor"

        # Setting the recognition specific parameters that have to be distinguished:

        # - phase_threshold: minimum seconds of speaking audio before we consider the speaking audio a phrase (values below this are ignored)
        # - pause_threshold: seconds of non-speaking audio before a phrase is considered complete
        # - confidence_threshold: how good the understood sentence matches the sentence from the list (range: [0, 1])
        # - silence_timeout: seconds before an error is raised when no speech is recorded.

        if context == "done":
            self.listener.phrase_threshold = 1
            self.listener.pause_threshold = 1
            confidence_threshold = 0.3
            silence_timeout = 60
        else:
            self.listener.phrase_threshold = 2
            self.listener.pause_threshold = 1.5
            confidence_threshold = 0.5
            silence_timeout = 10

        with sr.Microphone() as source:
            rospy.loginfo("\n--------------------- Listening for Microphone Input-------------------")
            try:
                # Collecting raw audio from microphone.
                audio_data = self.listener.listen(source, timeout=silence_timeout)
            except sr.WaitTimeoutError as e:  # throws when "silence_timeout" is exceeded
                rospy.loginfo("Timeout: %s", e)
                return "repetition_request" if context == "done" else "timeout"

        with self.client.connect() as connection:
            # Transforms the audio in a string based on the language
            hypotheses, _ = connection.recognize(audio_data, ['ds', 'greedy'])
            rospy.loginfo("Docks2 understood: %s", hypotheses.lower())

            # Match the sentence to a sentence in the list given, with certain confidence.
            docks_hypotheses, confidence = connection.postprocess(postprocessor, hypotheses)

        sentence_list_index = sentence_list.index(docks_hypotheses + "\n")
        rospy.loginfo("Recognized answer %s with %s confidence", sentence_list_index, confidence)

        # Extracting sentence_id from recognition data.
        if context == "done":
            if confidence > confidence_threshold and sentence_list_index == 0:
                sentence_id = "done_confirmation"
            else:
                sentence_id = "repetition_request"
        else:
            if confidence > confidence_threshold:
                if sentence_list_index < context_sentences:
                    sentence_id = context + "_question_" + str(sentence_list_index)
                else:
                    sentence_id = "repetion_request"
            else:
                sentence_id = "timeout"
        return sentence_id


if __name__ == "__main__":
    processor = SentenceList()
    result = processor.recognize(sys.argv[1])
    print(result)
