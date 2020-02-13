# Speech Module

[![speech pipeline status](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/badges/speech/pipeline.svg)](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/commits/speech)
[![speech coverage report](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/badges/speech/coverage.svg)](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/commits/speech)

This module contains all code related to the recognition and generation of spoken words.

## Speech Recognition Service (STT)

Upon call, this service will activate the microphone and listen for a matching sentence from either `wendigo.sentences.txt`, `mission.sentences.txt` or `emergency.sentences.txt` - depending on the value of the `context` parameter provided.
When above a certain threshold, it will return with the most plausible sentence. This means every individual recognition requires its own service call.

### How to use

**Start the service**
```
rosrun speech SpeechRecognitionStub.py <ROBOT_NAME(optional)>
```
it will start the service \ROBOT_NAME(if provided)\speech_recognition
**Call the service**
```
rosservice call speech_recognition <context>
```
speech_recognition is the service name to be used: Generally it is in following format
\ROBOT_NAME(if provided)\speech_recognition
where `context` can be either `scene_<number>` or `done`.

## Speech Production Service (TTS)

we have provided a service `speech_synthesis` for playing the soundfile for each scene. The service takes the <scene_ID, delay> as input and plays the audiofile corresponding
to the provided scene_ID. It also publishes the audio progress on `speech_progress` rostopic.

### How to use

**Run the package**
- pip install -r requirement.txt
- cd <CATKIN_WS>
- catkin_make
- source devel/setup.bash

**Start the service**
```
rosrun speech SpeechSynthesisStub.py
```

**Call the service**
```
rosservice call speech_synthesis <scene_id> <delay_in_sec>
```

**Setup a subscriber to subscribe to rosTopic**
```
import rospy
from speech.msg import SpeechProgress

sub = rospy.Subscriber('speech_progress', SpeechProgress, callback)
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d %s %d", data.stamp.secs, data.sentence, data.progress)
```

**Monitor the rostopic for getting the speech progress**(for debugging])
```
rostopic echo /speech_progress
```

## Initial Setup / Prerequisites

Please make sure that you followed the [General Setup](../README.md) and made sure you have all required dependencies (`ROS`, `NICO-software`) installed.

The speech module makes use of the WTM [docks](https://git.informatik.uni-hamburg.de/twiefel/docks) technology for speech recognition: for an initial setup you will therefore need to have access to the [docks2_remote](https://git.informatik.uni-hamburg.de/twiefel/docks2_remote) and [flaskcom](https://git.informatik.uni-hamburg.de/twiefel/flaskcom) repositories, which are included as [git submodules](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/blob/dev/.gitmodules).

For speech production, we make use of Amazon Polly to generate .wav sound files from input text provided by the scenario group, annotated with prosody tags using [SSML](https://de.wikipedia.org/wiki/Speech_Synthesis_Markup_Language).
These generated sound files are stored within this repository in the `generated_sounds` folder to be used by the SpeechProduction service.

### Regenerating the Sound Files (optional)

If however the `script.yml` changes, they have to be regenerated using the `tts_converter.py`. For this you need a valid AWS account, which as a university member, you can apply for at [AWS Educate](https://www.awseducate.com/registration#APP_TYPE).

After that, provide access via [Configuring the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-configure.html).
You should end up with both a `config` and `credentials` file a `.aws` folder in your home directory.
