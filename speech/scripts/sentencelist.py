import rospy
import sys
import timeit
from os.path import dirname, abspath

import speech_recognition as sr

import docks2_remote.docks2_remote.postprocessor as postprocessor
from docks2_remote import Client

def initialize():
    server = 'sysadmin@wtmitx1'
    port = 55101
    client = Client(server=server, port=port)

    # Language to be recognized
    language = "english"
    language_code = "en-EN"

    sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'
    done_sentence_list = open(sentences_dir + "wendigo.sentences.txt").readlines()
    mission_sentence_list = open(sentences_dir + "mission.sentences.txt").readlines()
    emergency_sentence_list = open(sentences_dir + "emergency.sentences.txt").readlines()
    with client.connect() as server:
        server.create_postprocessor(
            postprocessor.SentencelistPostprocessor,
            'done_sentencelist_postprocessor',
            sentencelist=done_sentence_list,
            language=language,
            language_code=language_code)

        server.create_postprocessor(
            postprocessor.SentencelistPostprocessor,
            'mission_sentencelist_postprocessor',
            sentencelist=mission_sentence_list,
            language=language,
            language_code=language_code)
        server.create_postprocessor(
            postprocessor.SentencelistPostprocessor,
            'emergency_sentencelist_postprocessor',
            sentencelist=emergency_sentence_list,
            language=language,
            language_code=language_code)


def recognize(context):
    """
    :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
    :return: sentence_id: The id of the sentence that was recognized.
    """

    # Server settings taken from Johannes Twiefel's recognition system.
    # This server is accessible only from computers connected to the informatikum network.
    time_0 = timeit.default_timer()
    server = 'sysadmin@wtmitx1'
    port = 55101
    client = Client(server=server, port=port)
    sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

    rospy.logdebug("Received context: %s", context)
    init_time = timeit.default_timer()
    print("Connection to server time: " + str(init_time - time_0))

    # Language to be recognized
    language = "english"
    language_code = "en-EN"

    # Translation from the context id to values of parameters that define the context.
    if context == "done":
        sentence_list = open(sentences_dir + "wendigo.sentences.txt").readlines()
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

    # The main recognizing unit.
    listener = sr.Recognizer()

    # Setting the recognition specific parameters that have to be distinguished:

    # - phase_threshold: minimum seconds of speaking audio before we consider the speaking audio a phrase (values below this are ignored)
    # - pause_threshold: seconds of non-speaking audio before a phrase is considered complete
    # - confidence_threshold: how good the understood sentence matches the sentence from the list (range: [0, 1])
    # - silence_timeout: seconds before an error is raised when no speech is recorded.

    if context == "done":
        listener.phrase_threshold = 1
        listener.pause_threshold = 1
        confidence_threshold = 0.3
        silence_timeout = 60
    else:
        listener.phrase_threshold = 2
        listener.pause_threshold = 1.5
        confidence_threshold = 0.5
        silence_timeout = 10

    pre_post_time = timeit.default_timer()
    with sr.AudioFile(dirname(dirname(abspath(__file__))) + "/generated_sounds/lab_noise.wav") as noise:
    # with sr.Microphone() as noise:
        listener.adjust_for_ambient_noise(noise, duration=3)
    print("Adjustment time: " + str(timeit.default_timer()-pre_post_time))
    with client.connect() as server:

        # print("Context time: " + str(timeit.default_timer()-pre_post_time))
        while True:

            # If recognition should be based on a file than use:
            # with sr.AudioFile("./example-wavs/test_sentence.wav") as source:

            # Recognizes directly from microphone stream
            # print("Context time: " + str(timeit.default_timer()-init_time))
            with sr.Microphone() as source:
                print('--------------------- Listening -------------------')
                print("Total elapsed time: " + str(timeit.default_timer() - time_0))
                try:
                    # Collecting raw audio from microphone.
                    audio_data = listener.listen(source, timeout=silence_timeout)

                    # Transforms the audio in a string based on the language
                    hypotheses, _ = server.recognize(audio_data, ['ds', 'greedy'])
                    rospy.logdebug("Docks2 understood: %s", hypotheses.lower())

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
    print(recognize(sys.argv[1]))
