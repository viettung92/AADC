#!/bin/bash 

for i in /tmp/scm*.log; do

    echo "$i"

	while read line           
	do           
	    #echo "$line"
	    IFS=' = ' read -a myarray <<< "$line"
	    size=${#myarray} 

	    # line containg right data
	    if [ $size -gt 2 ]
	    then
	       TEXT=("${myarray[0]}")
	       number=("${myarray[1]}")
	       #echo $size

	       NUMBERCUT="${number%?}"

	       #echo "Text: $TEXT"
	       #echo "Number: $number"
	       #echo "Numbercut: $NUMBERCUT"
	       sizenumbercut=${#NUMBERCUT} 

	       # only repace if FB/AC greater 9999
	       if [ $sizenumbercut -gt 4 ]
	       then
		 sed -i "s/$NUMBERCUT/$TEXT/g" "$i"
	       fi

	    fi

	done <$HOME/ADTF/aadc2017/include/ScmCommunication.h          

	while read line           
	do           
	    #echo "$line"
	    IFS=' = ' read -a myarray <<< "$line"
	    size=${#myarray} 

	    # line containg right data
	    if [ $size -gt 2 ]
	    then
	       TEXT=("${myarray[0]}")
	       number=("${myarray[1]}")
	       #echo $size

	       NUMBERCUT="${number%?}"

	       #echo "Text: $TEXT"
	       #echo "Number: $number"
	       #echo "Numbercut: $NUMBERCUT"
	       sizenumbercut=${#NUMBERCUT} 

	       # only repace if FB/AC greater 999
	       if [ $sizenumbercut -gt 3 ]
	       then
		 sed -i "s/$NUMBERCUT/$TEXT/g" "$i"
	       fi

	    fi

	done <$HOME/ADTF/aadc2017/include/ScmCommunication.h

done
