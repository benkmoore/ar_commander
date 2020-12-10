#!/bin/bash

cwd=$(echo $PWD)

cd ~/catkin_ws/
rm -r devel/ build/

cd ~/catkin_ws/src/ar_commander/
touch CMakelists.txt

cd ~/catkin_ws/
catkin_make
source devel/setup.bash

cd $cwd
exec bash
