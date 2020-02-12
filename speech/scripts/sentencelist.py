import rospy
import sys
from os.path import dirname, abspath
from os import listdir, devnull
import time
from termcolor import colored
from textblob import TextBlob

import speech_recognition as sr

import docks2_remote.docks2_remote.postprocessor as postprocessor
from docks2_remote import Client


class SentenceList:
    # Class Constants:
    base_dir = dirname(dirname(abspath(__file__)))
    protocols = {"done": "done",  # maps the context to the corresponding mission/emergency protocol
                 "scene_0": "mission", "scene_1": "mission",
                 "scene_2": "emergency", "scene_3": "emergency", "scene_4": "emergency"}
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
            self.listener.pause_threshold = 0.8

            self.context_sentences = 4
            self.confidence_threshold = 0.3
            self.silence_timeout = 60
        else:
            self.listener.phrase_threshold = 1
            self.listener.pause_threshold = 0.5

            if context == "scene_0" or context == "scene_1":
                self.context_sentences = 4
            else:
                self.context_sentences = 3

            self.confidence_threshold = 0.5
            self.silence_timeout = 20


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
                    match, confidence = connection.postprocess(self.post_processor, hypotheses)
                # No need to match when below confidence threshold
                if confidence > self.confidence_threshold: return match, confidence
                else:
                    rospy.loginfo("Low confidence, recognized with confidence %s:\n \'%s\'", confidence, match)
                    if self.context == "done":
                        return "repetition_request", confidence
                    elif TextBlob(hypotheses.lower()).sentiment[0] < 0:  # The sentence is negative
                        rospy.loginfo("Negativity of sentence: " + str(TextBlob(hypotheses.lower()).sentiment[0]))
                        return "no_question", confidence
                    else:
                        return "timeout", confidence
            except sr.WaitTimeoutError as e:  # throws when "silence_timeout" is exceeded
                rospy.loginfo("Timeout: %s", e)
                return "", 0  # => will be turned into 'repetition_request' / 'timeout'
            except BaseException as e:
                 rospy.loginfo("Error occured in recognition: %s", e)
                 return "fallback", 0 # => need for fallback.

    def match_sentence(self, docks_hypotheses, confidence):
        """
        :param docks_hypotheses: sentence the docks postprocessor suggested.
        :param confidence: how sure the postprocessor is about the hypothesis (range: [0, 1])
        :return: sentence_id: The id of the sentence that was recognized.
        """
        # The number of sentences that indicate that the user does not want to ask any questions.
        no_sentences = 4

        # No need to match when below confidence threshold
        if confidence <= self.confidence_threshold:
            return docks_hypotheses


        # Extracting sentence_id from recognition data.
        matched_line = self.sentences[self.protocols[self.context]].index(docks_hypotheses + "\n")
        rospy.loginfo("Recognized line number %s, %s with confidence %s.", matched_line, docks_hypotheses, confidence)
        if self.context == "done" and matched_line < self.context_sentences:
            return "done_confirmation"
        elif matched_line < self.context_sentences:
            return self.context + "_question_" + str(matched_line)
        elif self.context!="done" and matched_line < (self.context_sentences + no_sentences):
            return "no_question"
        else:
            return "repetition_request"

    def recognize_file(self, file_path):
        print_off()
        with sr.WavFile(file_path) as source:
            try:
                audio_data = self.listener.listen(source, timeout=self.silence_timeout)
                with self.client.connect() as connection:
                    # Transform the audio into a string
                    hypotheses, _ = connection.recognize(audio_data, ['ds', 'greedy'])
                    print_on()
                    unsupressed_print("Docks2 understood: " + hypotheses.lower())
                    # Match the understood sentence to the best candidate from the sentence list
                    match, confidence = connection.postprocess(self.post_processor, hypotheses)
                    # No need to match when below confidence threshold
                if confidence > self.confidence_threshold:
                    return match, confidence
                else:
                    unsupressed_print("Low confidence, recognized \"" + match + "\" with confidence " + str("%.2f" % confidence))
                    if self.context == "done":
                        return "repetition_request", confidence
                    elif TextBlob(hypotheses.lower()).sentiment[0] < 0:  # The sentence is negative
                        unsupressed_print("Negativity of sentence: " + str(TextBlob(hypotheses.lower()).sentiment[0]))
                        return "no_question", confidence
                    else:
                        return "timeout", confidence
                return connection.postprocess(self.post_processor, hypotheses)
            except sr.WaitTimeoutError as e:  # throws when "silence_timeout" is exceeded
                unsupressed_print("Timeout: " + str(e))
                return "", 0  # => will be turned into 'repetition_request' / 'timeout'
            except BaseException as e:
                unsupressed_print("Error occured in recognition: " + str(e))
                return "fallback", 0  # => need for fallback.

def get_context_from_file(filename):
    file_elements = filename.split("_")
    if file_elements[0] == "scene":
        return file_elements[0] + "_" + file_elements[1]
    else:
        return file_elements[0]

def print_on():
    sys.stdout = sys.__stdout__

def print_off():
    sys.stdout = open(devnull, 'w')

def unsupressed_print(sentence):
    print_on()
    print(sentence)
    print_off()

if __name__ == "__main__":
    processor = SentenceList()
    processor.initialize()
    if sys.argv[1] =='test':
        print_off()
        test_dir = processor.base_dir + "/recorded_sounds/" + sys.argv[2] + "/"
        correct=0
        for filename in listdir(test_dir):
            unsupressed_print("")
            processor.configure(get_context_from_file(filename))
            docks_hypotheses, confidence = processor.recognize_file(test_dir + filename)
            sentence = processor.match_sentence(docks_hypotheses, confidence)
            unsupressed_print("Recognized " + sentence + ": \"" + docks_hypotheses + "\". Confidence " + str("%.2f" % confidence))
            filename_array = filename.split("_")
            sentence_array = sentence.split("_")
            if filename_array[0] == sentence_array[0]:
                correct += 1
                unsupressed_print(colored("Original: " + filename, "green"))
            else:
                unsupressed_print(colored("Original: " + filename, "red"))
        unsupressed_print("\ntotal score: " + str(correct) + "/" + str(len(listdir(test_dir))))

    else:
        processor.configure(sys.argv[1])
        print("-------- Listening -------")
        docks_hypotheses, confidence = processor.recognize()
        sentence = processor.match_sentence(docks_hypotheses, confidence)
        print(sentence)
