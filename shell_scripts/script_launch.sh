#!/bin/bash

echo "drayYes"
echo ".."
echo ".."
echo ".."
sleep 10s # It's not crucial to have the waiting time but just so the student has time to get ready to watch
echo "$1"

directory_name="$1"

python3 "$directory_name.py"
