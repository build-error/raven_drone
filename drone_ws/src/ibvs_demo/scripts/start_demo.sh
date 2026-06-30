#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSION="ibvs_demo"
CONFIG="$SCRIPT_DIR/../tmux/demo.yaml"

pkill -f "gz sim" 2>/dev/null
pkill -f px4 2>/dev/null
pkill -f MicroXRCEAgent 2>/dev/null

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

tmuxp load "$CONFIG"