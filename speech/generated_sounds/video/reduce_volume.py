from pydub import AudioSegment

song = AudioSegment.from_wav("censor-beep-2.wav")

# reduce volume by 10 dB
song = song - 15

# save the output
song.export("beep_quieter.wav", "wav")
