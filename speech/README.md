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

We use Amazon Polly to generate .mp3 sound files from input text.

