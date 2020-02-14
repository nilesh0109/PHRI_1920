import mido
from os.path import dirname, abspath
import sys
import yaml

class LightController:

    @staticmethod
    def get_output_names():
        return mido.get_output_names()

    def __init__(self, port_name='CH345:CH345 MIDI 1 36:0'):
        self.out_port = mido.open_output(port_name)
        # Load configuration file:
        conf_file = dirname(dirname(abspath(__file__))) + '/scripts/light_patterns.yml'
        with open(conf_file, mode='r') as yml_file:
            self.light_patterns = yaml.safe_load(yml_file)['light_patterns']

    def set_lights(self, setting):
        w, r, g, b = self.light_patterns[setting]
        [self.white_spotlight_on(light_id, cold=w, warm=w) for light_id in range(4)]
        [self.rgb_spotlight_on(light_id, red=r, green=g, blue=b) for light_id in range(2)]

    def set_light(self, light_id, intensity):
        self.out_port.send(mido.Message("note_on", note=light_id+69, velocity=int(intensity/100 * 127)))

    def white_spotlight_on(self, light_id, cold=1.0, warm=1.0):
        if light_id not in range(4):
            print("Please set an id between 0 and 3")
            exit(1)
        cold = max(0.0, min(cold, 1.0))
        warm = max(0.0, min(warm, 1.0))
        self.out_port.send(mido.Message("note_on", note=70 + light_id * 2, velocity=int(127 * cold)))
        self.out_port.send(mido.Message("note_on", note=71 + light_id * 2, velocity=int(127 * warm)))

    def white_spotlight_off(self, light_id):
        if light_id not in range(4):
            print("Please set an id between 0 and 3")
            exit(1)
        self.out_port.send(mido.Message("note_off", note=70 + light_id * 2))
        self.out_port.send(mido.Message("note_off", note=71 + light_id * 2))

    def rgb_spotlight_on(self, light_id, red=1.0, green=1.0, blue=1.0):
        if light_id not in range(2):
            print("Please set an id between 0 and 1")
            exit(1)
        red = max(0.0, min(red, 1.0))
        green = max(0.0, min(green, 1.0))
        blue = max(0.0, min(blue, 1.0))
        self.out_port.send(mido.Message("note_on", note=82 + light_id * 3, velocity=int(127 * red)))
        self.out_port.send(mido.Message("note_on", note=83 + light_id * 3, velocity=int(127 * green)))
        self.out_port.send(mido.Message("note_on", note=84 + light_id * 3, velocity=int(127 * blue)))

    def rgb_spotlight_off(self, light_id):
        if light_id not in range(2):
            print("Please set an id between 0 and 1")
            exit(1)
        self.out_port.send(mido.Message("note_off", note=82 + light_id * 3))
        self.out_port.send(mido.Message("note_off", note=83 + light_id * 3))
        self.out_port.send(mido.Message("note_off", note=84 + light_id * 3))

    def blackout_on(self):
        self.out_port.send(mido.Message("note_on", note=102))

    def blackout_off(self):
        self.out_port.send(mido.Message("note_off", note=102))

    def close(self):
        self.out_port.close()

    def __del__(self):
        self.close()

if __name__ == "__main__":
    ct = LightController()
    ct.set_lights(sys.argv[1])
