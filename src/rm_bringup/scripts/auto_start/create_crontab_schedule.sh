#!/bin/bash
# Cilotta at 2024-9-16

if [ -f "$1" ];then
	echo "Files: $1"
else
	echo "usage:  $0 filename [execution_per_min]"
	exit 1
fi
if [ $2 = 0 ];then echo "Delay = $2,error";exit 1
fi

echo "######"
echo -e "\e[7mCurrent crontab contents:\e[0m"
echo "`crontab -l`"
echo "######"

current_cron=$(crontab -l 2>/dev/null)
full_path=$(realpath "$1")

echo "$current_cron" > /tmp/cron_tmp

if [ -z "$2" ];then
	echo "* * * * * $full_path" >> /tmp/cron_tmp
else
	freq=`expr 60 / $2`
        for ((i=0;i<60;i+=$freq))
        do
                echo "* * * * * sleep $i;$full_path" >> /tmp/cron_tmp
        done
fi

crontab /tmp/cron_tmp
rm /tmp/cron_tmp

echo -e "\e[7mNow crontab contents:\e[0m"
echo "`crontab -l`"
echo "######"
echo -e "Use \e[7mcrontab -e\e[0m to edit or \e[7mcrontab -r\e[0m to remove all"
echo "######"

