#!/bin/bash
echo '/home/'$USER >> list
#RESULT0=$(find / -type d -quit -name $USER)
#echo $RESULT0 >> list
RESULT1=$(find /opt -type d -print0 | grep -FzZ 'gazebo_ros/launch' 2>/dev/null)
echo $RESULT1 >> list

