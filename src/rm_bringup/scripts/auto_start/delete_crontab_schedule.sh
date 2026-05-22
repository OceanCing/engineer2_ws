#!/bin/bash
# Cilotta at 2024-9-26

echo "######"
echo -e "\e[7mCurrent crontab contents:\e[0m"
echo "`crontab -l`"
echo "######"
crontab -r 2>/dev/null
echo -e "\e[7mNow crontab contents:\e[0m"
echo "`crontab -l`"
echo "######"
echo -e "Use \e[7mcrontab -e\e[0m to edit or \e[7mcrontab -r\e[0m to remove all"
echo "######"
