#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd "$SCRIPT_DIR/.."
source env/bin/activate
cd src

python VisualOdometry.py -c $HOME/.VisualNav/ -s "tcp:localhost:5762" -o $HOME/.VisualNav/output -i UnrealEngineCamera


