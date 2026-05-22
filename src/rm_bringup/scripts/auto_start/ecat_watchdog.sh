#!/bin/bash
# Cilotta at 2024-9-15

baseDir=$(cd `dirname $0` ; pwd)
server="rm_ecat_start"
ecl=`cat /tmp/hw.log | tail -n 20 | grep "Working counter is too low" | grep -v grep | wc -l` # count Error Cmd Line
if [ $ecl -eq 0 ]
then 
	echo "Error line=$ecl,ecat is online."
else
	echo "Error line=$ecl,ecat is offline, will restart service $server."
	sudo systemctl restart $server
	echo "Restart $server."
fi
