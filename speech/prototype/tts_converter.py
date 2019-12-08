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


# %% Call AWS Polly to convert text to mp3
def synthesize(client, speaker, text, line):
    response = client.synthesize_speech(Text=text, VoiceId=speakers[speaker]['voice'], TextType='ssml',
                                        Engine=speakers[speaker]['engine'], OutputFormat='mp3')
    file = open(f'generated_sounds/line_{line}_{speaker}.mp3', 'wb')
    file.write(response['AudioStream'].read())
    file.close()

# %% main
for idx, row in enumerate(script):
    # call AWS Polly
    try:
        synthesize(polly_client, row["speaker"], row["text"], idx)
        print(f'Processed line {idx}.')
    except Exception as e:
        print(f'Skipped line {idx}: {e}')
