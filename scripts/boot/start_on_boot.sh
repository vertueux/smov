#!/bin/bash

yes | sudo cp -f ~/smov/utilities/boot/service /etc/systemd/system/service

echo "Enabling smov to run on boot"
sudo systemctl enable service;
sudo systemctl daemon-reload;
echo "Done"

if ! systemctl is-active --quiet service; then
  echo "Service smov is not running, run 'sudo systemctl start service' to start it"
fi

