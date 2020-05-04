#!/bin/bash
echo "$(basename $0) start!"
VERSION=v0.1.6
wget https://github.com/AtsushiSakai/DiffSentinel/archive/${VERSION}.zip
unzip ${VERSION}.zip
./DiffSentinel*/starter.sh HEAD origin/master
check_result=$?
rm -rf ${VERSION}.zip DiffSentinel*
if [[ ${check_result} -ne 0 ]];
then
    echo "Error: Your changes contain pycodestyle errors."
    exit 1
fi
echo "$(basename $0) done!"
exit 0
