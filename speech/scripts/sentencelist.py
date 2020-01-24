import sys
from docks2_remote import Client
import docks2_remote.docks2_remote.postprocessor as postprocessor
import speech_recognition as sr
from os.path import dirname, abspath
from argparse import ArgumentParser


def recognize(context):
    '''
    :param context: The specific scene number or "done" if listening for "Wendigo, I'm done".
    :return: sentence_id: The id of the sentence that was recognized.
    '''

    # Server settings taken from Johannes Twiefel's recognition system.
    # This server is accessible only from computers connected to the informatikum network.
    server = 'sysadmin@wtmitx1'
    port = 55101
    client = Client(server=server, port=port)
    sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

    print("Received context: " + context)

    # Language to be recognized
    language = "english"
    language_code = "en-EN"

    # Translation from the context id to values of parameters that define the context.
    if context == "done":
        sentence_list = open(sentences_dir + "wendigo.sentences.txt").readlines()
        context_sentences = 1
        print("Using wendigo sentences")
    elif context == "scene_0" or context == "scene_1":
        sentence_list = open(sentences_dir + "mission.sentences.txt").readlines()
        context_sentences = 4
        print("Using mission sentences")
    else:
        sentence_list = open(sentences_dir + "emergency.sentences.txt").readlines()
        context_sentences = 3
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


    # with sr.AudioFile("./example-wavs/test_adjust.wav") as noise:
    with sr.Microphone() as noise:
        listener.adjust_for_ambient_noise(noise)

    with client.connect() as server:
        # creates a sentencelist postprocessor on the server with a given sentence-list
        server.create_postprocessor(
            postprocessor.SentencelistPostprocessor,
            'sentencelist_postprocessor',
            sentencelist=sentence_list,
            language=language,
            language_code=language_code)

    while True:

        # If recognition should be based on a file than use:
        # with sr.AudioFile("./example-wavs/test_sentence.wav") as source:

        # Recognizes directly from microphone stream
        with sr.Microphone() as source:
            print('--------------------- Listening -------------------')
            try:
                # Collecting raw audio from microphone.
                audio_data = listener.listen(source, timeout=silence_timeout)

                # Transforms the audio in a string based on the language
                hypotheses, _ = server.recognize(audio_data, ['ds', 'greedy'])
                print
                "Docks2 understood: {}".format(hypotheses.lower())

                # Match the sentence to a sentence in the list given, with certain confidence.
                docks_hypotheses, confidence = server.postprocess('sentencelist_postprocessor',
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


            except: # This mainly catches a time out exception based on "silence_timeout".
                if context == "done":
                    sentence_id = "repetition_request"
                else:
                    sentence_id = "timeout"
                print("To be returned to service: " + sentence_id)
                return sentence_id


if __name__ == "__main__":
    print(recognize(sys.argv[1]))
