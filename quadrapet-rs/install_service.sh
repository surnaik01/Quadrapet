#!/bin/bash

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")
sudo ln -s ${SCRIPT_DIR}/quadrapet-gui.service /etc/systemd/system/quadrapet-gui.service
sudo systemctl enable quadrapet-gui.service