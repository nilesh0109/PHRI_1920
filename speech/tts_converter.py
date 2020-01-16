import boto3
import yaml
import wave

# Load configuration: selected Polly voices for Robot A, Robot B and Ship S.
with open('speakers.yml', mode='r') as yml_file:
    speakers = yaml.safe_load(yml_file)
# voices = {'A': 'Matthew', 'B': 'Matthew', 'S': 'Joanna'}
with open('script.yml', mode='r') as yml_file:
    script = yaml.safe_load(yml_file)

with open('questions.yml', mode='r') as yml_file:
    questions = yaml.safe_load(yml_file)

with open('repetition.yml', mode='r') as yml_file:
    repetition = yaml.safe_load(yml_file)

# Create a new Polly Session
polly_client = boto3.Session().client('polly')

# %% main

# Script files
for scene_idx, scene in enumerate(script):
    # set the initial speaker index for every speaker to 0
    speaker_idx = {k: 0 for k in speakers.keys()}
    for line_idx, line in enumerate(scene['scene']):
        name = line["speaker"]
        voice = speakers[name]['voice']
        engine = speakers[name]['engine']
        try:
            # call AWS Polly
            response = polly_client.synthesize_speech(Text=line["text"], VoiceId=voice, TextType='ssml',
                                                      Engine=engine, OutputFormat='pcm')
            # write PCM data to a .wav file
            file_path = f'generated_sounds/scene_{scene_idx}_{name}_line_{speaker_idx[name]}.wav'
            with wave.open(file_path, 'wb') as wav_file:
                wav_file.setparams((1, 2, 16000, 0, 'NONE', 'NONE'))
                wav_file.writeframes(response['AudioStream'].read())
            # increment the speaker index
            speaker_idx[name] += 1
            print(f'Processed Scene {scene_idx}, Line {line_idx}.')
        except Exception as e:
            print(f'Error in Scene {scene_idx}, Line {line_idx}:'
                  f'\n Error: {e}'
                  f'\n Text: {line["text"]}')

# Question files
name = "A"
voice = speakers[name]['voice']
engine = speakers[name]['engine']
for scene_idx, scene in enumerate(questions):
	for question_idx, question in enumerate(scene['scene']):
		for answer_idx, answer in enumerate(question['question']):
		    try:
		        # call AWS Polly
		        response = polly_client.synthesize_speech(Text=answer["answer"], VoiceId=voice, TextType='ssml',
		                                                  Engine=engine, OutputFormat='pcm')
		        # write PCM data to a .wav file
		        file_path = f'generated_sounds/scene_{scene_idx}_question_{question_idx}_answer_{answer_idx}.wav'
		        with wave.open(file_path, 'wb') as wav_file:
		            wav_file.setparams((1, 2, 16000, 0, 'NONE', 'NONE'))
		            wav_file.writeframes(response['AudioStream'].read())
		        print(f'Processed Scene {scene_idx}, Question {question_idx}, Answer{answer_idx}.')
		    except Exception as e:
		        print(f'Error in Scene {scene_idx},Question {question_idx}, Answer{answer_idx}.'
		              f'\n Error: {e}'
		              f'\n Text: {answer["answer"]}')

# Request for repetition
name = repetition[0]['sentence'][0]['speaker']
voice = speakers[name]['voice']
engine = speakers[name]['engine']
try:
	response = polly_client.synthesize_speech(Text=repetition[0]['sentence'][0]['text'], VoiceId=voice, TextType='ssml', Engine=engine, OutputFormat='pcm')
	file_path = 'generated_sounds/repeat'
	with wave.open(file_path, 'wb') as wav_file:
		wav_file.setparams((1, 2, 16000, 0, 'NONE', 'NONE'))
		wav_file.writeframes(response['AudioStream'].read())
	print(f'Processed repetition')
except Exception as e:
	print(f'Error in repetition'
			f'\n Error: {e}'
	        f'\n Text: {repetition[0]["text"]}')
