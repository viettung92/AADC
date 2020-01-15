#! /bin/bash

export DISPLAY:=0
echo "Load and Execute final project from console, no GUI will be started!"
echo "changing directory ... "
export ADTF_DIR=/opt/adtf/2.13.1

if [ -d $ADTF_DIR ]; then
    echo "ADTF dir found."
else
    echo "ADTF dir not found in $ADTF_DIR. Check the path to ADTF."
    exit 1
fi

echo "Starting adtf_launcher with adtf_console.manifest and final project configuration "
$ADTF_DIR/bin/adtf_launcher $ADTF_DIR/bin/adtf_console.manifest -config="/home/aadc/AADC/config/CarProjects/5_Final/config/system.xml" -run

## use command "getrl" to get current running level
## use command "setrl" to set current running level
## Running level: RL_Shutdown = 0, RL_Kernel = 1, RL_System = 2, RL_XSystem = 3, RL_Application = 4, RL_Running = 5, RL_Internal = 6
## -> documentation / enumeration can be found in "runtim_intf.h" (line 117)
## if "-run" parameter is used in startup, no further running level has to be set!

## To exit the execution, use "quit" 
