#!/bin/bash

SCRIPT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$SCRIPT_DIRECTORY/../.." || exit

python3 -m venv venv --clear
source venv/bin/activate

curl https://bootstrap.pypa.io/get-pip.py | python

python3 -m pip install --upgrade pip setuptools
python3 -m pip install --upgrade jmespath
python3 -m pip install --upgrade adafruit-circuitpython-motor
python3 -m pip install --upgrade adafruit-circuitpython-pca9685
python3 -m pip install --upgrade inputs
python3 -m pip install --upgrade pick

#python3 -m pip install --upgrade smbus
#python3 -m pip install --upgrade RPi.GPIO
