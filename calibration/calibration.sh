#!/bin/bash

cd ~/smov || exit
export PYTHONPATH=.

venv/bin/python3 calibration/calibration/calibration.py
