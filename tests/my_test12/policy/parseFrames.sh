#!/bin/bash           

file=frames

while read line; do
	var=$line
done < $file

#set -- $var
#paramCount=$1 #Configuration parameter count
#numParams=paramCount/2 #hard-coded; The actual number of params for 1 robot

x=0
for i in $var
do
	if  [ $x -eq 0 ] || [ $x -eq 1 ] || [ $x -eq 2 ] || [ $x -eq 6 ] ; then
		line+=$i
		line+=' '
	fi
	if [ $x -eq 12 ] ; then
		x=-1
		echo $line >> output2.txt
		#echo $line
		line=''
	fi
((x++))
done
