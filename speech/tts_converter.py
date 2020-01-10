import boto3
import yaml
import wave

# Load configuration: selected Polly voices for Robot A, Robot B and Ship S.
with open('speakers.yml', mode='r') as yml_file:
    speakers = yaml.safe_load(yml_file)
# voices = {'A': 'Matthew', 'B': 'Matthew', 'S': 'Joanna'}
with open('script.yml', mode='r') as yml_file:
    script = yaml.safe_load(yml_file)
# Create a new Polly Session
polly_client = boto3.Session().client('polly')

# %% main
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
