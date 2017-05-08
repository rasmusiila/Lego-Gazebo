#!/bin/bash

echo "mawNoo"
cd ~
DIRECTORY="lego_test"

if [ "$#" != 3 ]; then
    echo "USAGE: robot_test [uni-id] [exercise_number] [map_number]"
    exit 0
fi

git_user_name=$1
exercise_number=$2
map_number=$3

git_address="https://github.com/$git_user_name/$DIRECTORY.git"

echo $git_address

if [ -d "$DIRECTORY" ]; then
    echo "Repository already exists. Updating..."
    cd $DIRECTORY
    git pull
else
    echo "Repository doesn't exist yet. Creating one now..."
    git clone $git_address
    cd $DIRECTORY
fi

exercise_name=""
directory_name=""

case "$exercise_number" in
                "1")       
     		    exercise_name="camera"
		    directory_name="EX01"
                    ;;
                "2")
     		    exercise_name="sonar"
		    directory_name="EX02"
                    ;;            
                "3")       
     		    exercise_name="gyro"
		    directory_name="EX03"
                    ;;
		"4")       
     		    exercise_name="touch"
		    directory_name="EX04"
                    ;;
                *)              
          esac 

if [ "$exercise_name" == "" ]; then
    echo "There's no exercise with a number of: $exercise_number"
    exit 1
fi

if ! [ -f "/home/robot/Lego-Gazebo/catkin_ws/src/lego_gazebo/launch/lego_$exercise_name$map_number.launch" ]; then
    echo "There's no map $map_number for this exercise."
    exit 1
fi

if [ -d "$directory_name" ]; then
    cd $directory_name
else
    echo "Your files do not contain the $directory_name folder."
    exit 1
fi

if ! [ -f "$directory_name.py" ]; then
    echo "Your directory $directory_name does not contain the necessary $directory_name.py file."
fi

x-terminal-emulator -e roslaunch lego_gazebo lego_$exercise_name$map_number.launch

x-terminal-emulator -e script_launch $directory_name

#roslaunch lego_gazebo lego_$exercise_name.launch
