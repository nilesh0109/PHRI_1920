import csv

import boto3

# Selected Polly voices for Robot A, Robot B and Ship S.
voices = {'A': 'Brian', 'B': 'Brian', 'S': 'Joanna'}
# Create a new Polly Session
polly_client = boto3.Session().client('polly')


# %% Call AWS Polly to convert text to mp3
def synthesize(client, speaker, text, lc):
    response = client.synthesize_speech(Text=text, VoiceId=voices[speaker],
                                        Engine='neural', OutputFormat='mp3')
    file = open(f'generated_sounds/line_{lc}_{speaker}.mp3', 'wb')
    file.write(response['AudioStream'].read())
    file.close()


# %% main
def main():
    with open('prototype.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        line_count = 0
        for row in csv_reader:
            if line_count == 0:
                print(f'Column names are {", ".join(row)}')
            else:
                # call AWS Polly using
                synthesize(polly_client, row["Speaker"], row["Text"], line_count)
            line_count += 1
            print(f'Processed {line_count} lines.')


if __name__ == '__main__':
    main()
