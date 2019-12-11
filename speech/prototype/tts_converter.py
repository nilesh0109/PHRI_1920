import boto3
import yaml

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
    for line_idx, line in enumerate(scene['scene']):
        name = line["speaker"]
        voice = speakers[name]['voice']
        engine = speakers[name]['engine']
        try:
            # call AWS Polly
            response = polly_client.synthesize_speech(Text=line["text"], VoiceId=voice, TextType='ssml',
                                                      Engine=engine, OutputFormat='mp3')
            # write to .mp3 file
            file = open(f'generated_sounds/scene_{scene_idx}_line_{line_idx}_{name}.mp3', 'wb')
            file.write(response['AudioStream'].read())
            file.close()
            print(f'Processed Scene {scene_idx}, Line {line_idx}.')
        except Exception as e:
            print(f'Skipped Line {line_idx}: {e}')
