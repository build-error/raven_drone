#!/usr/bin/env bash

pkill -f px4
pkill -f "gz sim"
pkill -f MicroXRCEAgent
pkill -f rviz2
pkill -f rqt_image_view

tmux kill-session -t ibvs_demo 2>/dev/null

echo "IBVS demo stopped"