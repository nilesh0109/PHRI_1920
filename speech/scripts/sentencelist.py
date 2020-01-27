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
        with self.client.connect() as server:
            for context in self.sentences:
                sentence_list = open(self.sentences_dir + "{}.sentences.txt".format(context)).readlines()
                server.create_postprocessor(postprocessor.SentencelistPostprocessor,
                                            '{}_sentencelist_postprocessor'.format(context),
                                            sentencelist=sentence_list, language="english", language_code="en-EN")

    def recognize(self, context):
        """
        :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
        :return: sentence_id: The id of the sentence that was recognized.
        """
        sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

        rospy.loginfo("Received context: %s", context)

        # Translation from the context id to values of parameters that define the context.
        if context == "done":
            sentence_list = open(sentences_dir + "done.sentences.txt").readlines()
            context_sentences = 1
            postprocessor = 'done_sentencelist_postprocessor'
            print("Using wendigo sentences")
        elif context == "scene_0" or context == "scene_1":
            sentence_list = open(sentences_dir + "mission.sentences.txt").readlines()
            context_sentences = 4
            postprocessor = "mission_sentencelist_postprocessor"
            print("Using mission sentences")
        else:
            sentence_list = open(sentences_dir + "emergency.sentences.txt").readlines()
            context_sentences = 3
            postprocessor = "emergency_sentencelist_postprocessor"
            print("Using emergency sentences")


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

        with self.client.connect() as server:

            # print("Context time: " + str(timeit.default_timer()-pre_post_time))
            while True:

                # If recognition should be based on a file than use:
                # with sr.AudioFile("./example-wavs/test_sentence.wav") as source:

                # Recognizes directly from microphone stream
                # print("Context time: " + str(timeit.default_timer()-init_time))
                with sr.Microphone() as source:
                    print('--------------------- Listening -------------------')
                    try:
                        # Collecting raw audio from microphone.
                        audio_data = self.listener.listen(source, timeout=silence_timeout)

                        # Transforms the audio in a string based on the language
                        hypotheses, _ = server.recognize(audio_data, ['ds', 'greedy'])
                        rospy.loginfo("Docks2 understood: %s", hypotheses.lower())

                        # Match the sentence to a sentence in the list given, with certain confidence.
                        docks_hypotheses, confidence = server.postprocess(postprocessor,
                                                                          hypotheses)

                        sentence_list_index = sentence_list.index(docks_hypotheses + "\n")
                        print("This corresponds to answer " + str(sentence_list_index)
                              + " with confidence " + str(confidence))

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
                        print("To be returned to service: " + sentence_id)
                        return sentence_id

                    except Exception as e:  # This mainly catches a time out exception based on "silence_timeout".
                        rospy.logfatal("Error recognizing speech:\n %s", e)
                        if context == "done":
                            sentence_id = "repetition_request"
                        else:
                            sentence_id = "timeout"
                        print("To be returned to service: " + sentence_id)
                        return sentence_id


if __name__ == "__main__":
    processor = SentenceList()
    result = processor.recognize(sys.argv[1])
    print(result)
