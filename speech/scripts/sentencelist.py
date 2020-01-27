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
    context2sentence = {"done": "done",
                        "scene_0": "mission", "scene_1": "mission",
                        "scene_2": "emergency", "scene_3": "emergency", "scene_4": "emergency"}

    def initialize(self):
        # server settings from Johannes Twiefel: accessible only from the Informatikum network
        self.client = Client(server='sysadmin@wtmitx1', port=55101)
        # The main recognizing unit.
        self.listener = sr.Recognizer()
        # filter ambient lab noise using a previously recorded sound file
        with sr.AudioFile(self.base_dir + "/generated_sounds/lab_noise.wav") as noise:
            self.listener.adjust_for_ambient_noise(noise, duration=3)
        # Create the sentencelist postprocessors on the Docks server
        with self.client.connect() as connection:
            for postproc in set(self.context2sentence.viewvalues()):
                rospy.loginfo("Creating postprocessor: %s", postproc)
                sentence_list = open(self.sentences_dir + "{}.sentences.txt".format(postproc)).readlines()
                connection.create_postprocessor(postprocessor.SentencelistPostprocessor,
                                                '{}_sentencelist_postprocessor'.format(postproc),
                                                sentencelist=sentence_list, language="english", language_code="en-EN")

    def configure(self, context):
        return

    def recognize(self, context):
        # Translation from the context id to values of parameters that define the context.
        silence_timeout = 60 if context == "done" else 10
        postproc_id = self.context2sentence[context] + "_sentencelist_postprocessor"

        with sr.Microphone() as source:
            rospy.loginfo("\n--------------------- Listening for Microphone Input-------------------")
            try:
                # Collecting raw audio from microphone.
                audio_data = self.listener.listen(source, timeout=silence_timeout)
                with self.client.connect() as connection:
                    # Transforms the audio in a string based on the language
                    hypotheses, _ = connection.recognize(audio_data, ['ds', 'greedy'])
                    rospy.loginfo("Docks2 understood: %s", hypotheses.lower())

                    # Match the sentence to a sentence in the list given, with certain confidence.
                    return connection.postprocess(postproc_id, hypotheses)
            except sr.WaitTimeoutError as e:  # throws when "silence_timeout" is exceeded
                rospy.loginfo("Timeout: %s", e)
                return None, 0

    def match_sentence(self, context, docks_hypotheses, confidence):
        """
        :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
        :return: sentence_id: The id of the sentence that was recognized.
        """

        confidence_threshold = 0.3 if context == "done" else 0.5

        # Exit when low confidence
        if confidence <= confidence_threshold:
            return "repetition_request" if context == "done" else "timeout"

        sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

        # Translation from the context id to values of parameters that define the context.
        if context == "done":
            sentence_list = open(sentences_dir + "done.sentences.txt").readlines()
            context_sentences = 1
        elif context == "scene_0" or context == "scene_1":
            sentence_list = open(sentences_dir + "mission.sentences.txt").readlines()
            context_sentences = 4
        else:
            sentence_list = open(sentences_dir + "emergency.sentences.txt").readlines()
            context_sentences = 3

        # Setting the recognition specific parameters that have to be distinguished:

        # - phase_threshold: minimum seconds of speaking audio before we consider the speaking audio a phrase (values below this are ignored)
        # - pause_threshold: seconds of non-speaking audio before a phrase is considered complete
        # - confidence_threshold: how good the understood sentence matches the sentence from the list (range: [0, 1])
        # - silence_timeout: seconds before an error is raised when no speech is recorded.

        if context == "done":
            self.listener.phrase_threshold = 1
            self.listener.pause_threshold = 1
        else:
            self.listener.phrase_threshold = 2
            self.listener.pause_threshold = 1.5

        sentence_list_index = sentence_list.index(docks_hypotheses + "\n")
        rospy.loginfo("Recognized answer %s with %s confidence", sentence_list_index, confidence)

        # Extracting sentence_id from recognition data.
        if context == "done":
            if sentence_list_index == 0:
                sentence_id = "done_confirmation"
        else:
            if sentence_list_index < context_sentences:
                sentence_id = context + "_question_" + str(sentence_list_index)
            else:
                sentence_id = "repetion_request"

        return sentence_id


if __name__ == "__main__":
    processor = SentenceList()
    result = processor.recognize(sys.argv[1])
    print(result)
