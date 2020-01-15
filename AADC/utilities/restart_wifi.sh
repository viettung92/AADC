#!/bin/sh
# ping router and restart wifi
IPROUTER=192.168.1.1

while true
do

if ping -c 1 $IPROUTER >/dev/null 2>&1 ; then
    echo $IPROUTER ok
  else
    echo "No response, restart"
    sudo service network-manager restart
    #exit 0
fi

if ping -c 1 $IPROUTER >/dev/null 2>&1 ; then
  echo $IPROUTER ok
else
  #echo "No response, restart"
  sudo service network-manager restart
fi

sleep 200

done

#exit 0
