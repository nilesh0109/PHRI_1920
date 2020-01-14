# Speech Module

[![speech pipeline status](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/badges/speech/pipeline.svg)](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/commits/speech)
[![speech coverage report](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/badges/speech/coverage.svg)](https://git.informatik.uni-hamburg.de/wtm-teaching-projects/phri1920_dev/commits/speech)

This module contains all code related to the recognition and generation of spoken words.

## Setup

For speech production will you need a valid AWS account. As a university member, you can apply for [AWS Educate](https://www.awseducate.com/registration#APP_TYPE).

After that, provide access via [Configuring the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-configure.html).
You should end up with both a `config` and `credentials` file a `.aws` folder in your home directory. They will be read by the `boto3` Python library.

## Speech-To-Text (STT)

## Text-To-Speech (TTS)

We use Amazon Polly to generate .wav sound files from input text.

## Speech Production Service

we have provided a service `speech_synthesis` for playing the soundfile for each scene. The service takes the <scene_ID, delay> as input and plays the audiofile corresponding
to the provided scene_ID. It also publishes the audio progress on `speech_progress` rostopic.

## HOW TO
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
rosservice call speech_synthesis '<SCENE ID>' <delay in sec>
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
rostopic echo speech_progress
```