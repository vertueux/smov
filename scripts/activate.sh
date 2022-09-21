#!/bin/bash

cd ~/smov || exit

python3 -m venv venv --clear
source venv/bin/activate

curl https://bootstrap.pypa.io/get-pip.py | python

python3 -m pip install --upgrade pip setuptools jmespath throttle adafruit-circuitpython-motor adafruit-circuitpython-pca9685 inputs smbus RPi.GPIO pick
