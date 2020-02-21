import math
import struct
import wave

def generate_audio_file(volume=0.2, sampleRate=44100, duration=0.1,
                        frequency=1500.0):
    # taken from http://blog.acipo.com/wave-generation-in-python/
    wav = wave.open("generated_sounds/beep.wav", "wb")
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sampleRate)

    for i in range(int(duration * sampleRate)):
        value = int(volume * 32767.0 * math.cos(frequency *
                                                math.pi *
                                                float(i) / float(sampleRate)))
        data = struct.pack('<h', value)
        wav.writeframesraw(data)
    wav.close()


generate_audio_file()

