#!/bin/bash

service_exists() {
    local n=$1
    if [[ $(systemctl list-units --all -t service --full --no-legend "$n.service" | cut -f1 -d' ') == $n.service ]]; then
        return 0
    else
        return 1
    fi
}

if ! service_exists smov; then
  echo "Service smov is not installed as service"
  exit
fi

echo "Disabling smov to run on boot"
sudo systemctl disable service;
sudo systemctl daemon-reload;
echo "Done"

if systemctl is-active --quiet smov; then
  echo "Service smov is still running, run 'sudo systemctl stop service' to stop it"
fi

