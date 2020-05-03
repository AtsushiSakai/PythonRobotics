#!/bin/bash
echo "$(basename $0) start!"
VERSION=v0.1.3
wget https://github.com/AtsushiSakai/DiffSentinel/archive/${VERSION}.zip
unzip ${VERSION}.zip
./DiffSentinel*/starter.sh HEAD remotes/origin/master
rm -rf ${VERSION}.zip DiffSentinel*
echo "$(basename $0) done!"
exit 0
