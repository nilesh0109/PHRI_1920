import sys
from docks2_remote import Client
import docks2_remote.docks2_remote.postprocessor as postprocessor
import speech_recognition as sr
from os.path import dirname, abspath
from argparse import ArgumentParser


def recognize(context, confidence_threshold=0.5):
	server = 'sysadmin@wtmitx1'
	port = 55101

	use_google = False
	decode_with = 'greedy'

	print(context)

	language = "english"
	language_code = "en-EN"

	client = Client(server=server, port=port)
	sentences_dir = dirname(dirname(abspath(__file__))) + '/scripts/'

	if context == "done":
		sentencelist = open(sentences_dir + "wendigo.sentences.txt").readlines()
		context_sentences = 1
		print("Using wendigo sentences")
	elif context == "scene_0" or context == "scene_1":
		sentencelist = open(sentences_dir + "mission.sentences.txt").readlines()
		context_sentences = 4
		print("Using mission sentences")
	else:
		sentencelist = open(sentences_dir + "emergency.sentences.txt").readlines()
		context_sentences = 3
		print("Using emergency sentences")

	print(sentencelist)

	# setting up the recognizer
	listener = sr.Recognizer()

	listener.phrase_threshold = 2 # minimum seconds of speaking audio before we consider the speaking audio a phrase - values below this are ignored (for filtering out clicks and pops)
	listener.pause_threshold = 1  # seconds of non-speaking audio before a phrase is considered complete


    # with sr.AudioFile("./example-wavs/test_adjust.wav") as noise:
	with sr.Microphone() as noise:
		listener.adjust_for_ambient_noise(noise)

	with client.connect() as server:
        # creates a sentencelist postprocessor on the server
        # with given sentencelist
		server.create_postprocessor(
			postprocessor.SentencelistPostprocessor,
			'sentencelist_postprocessor',
			sentencelist=sentencelist,
			language=language,
			language_code=language_code)

        while True:

            # with sr.AudioFile("./example-wavs/test_sentence.wav") as source:
            with sr.Microphone() as source:
                print('Say Something!')
		try:
                	audio_data = listener.listen(source, timeout=10)

		        if not(use_google):
		            if decode_with == "greedy":
		                hypotheses, _ = server.recognize(audio_data,['ds', 'greedy'])
		                print "Docks2 understood: {}".format(hypotheses.lower())
		            # elif decode_with == "four-gram":
		                # hypotheses = server.recognize(audio_data,['ds', '4gram'])[0]
		                # print "Docks2 understood: {}".format(hypotheses.lower())
		            else:
		                hypotheses, _ = server.recognize(audio_data,['ds', '3gram'])
		                print "Docks2 understood: {}".format(hypotheses.lower())
		        else:
		            # uses the google recognizer on the server
		            hypotheses = server.recognize(audio_data, ['google'])
		            print "Google understood: {}".format(hypotheses)

		        # postprocess
		        docks_hypotheses, confidence = server.postprocess('sentencelist_postprocessor',
		                                              hypotheses)

		        print "Sentencelist Postprocessor understood: {}".format(docks_hypotheses)
			if confidence > confidence_threshold:
				print("This corresponds to answer " + str(sentencelist.index(docks_hypotheses+"\n")) + "With confidence " + str(confidence))
				sentence_nr = sentencelist.index(docks_hypotheses+"\n")
				if sentence_nr < context_sentences:
					if context == "done":
						sentence_id = "done_confirmation"
					else:
						sentence_id = context + "_question_" + str(sentence_nr)
				else:
					sentence_id = "repetion_request"

				print("To be returned to service: " + sentence_id)
			else:
				sentence_id="timeout"
				print("To be returned to service: " + sentence_id)
			return sentence_id
		except:
			sentence_id ="timeout"
			print("To be returned to service: " + sentence_id)
			return sentence_id

if __name__ == "__main__":
	print(recognize(sys.argv[1]))
