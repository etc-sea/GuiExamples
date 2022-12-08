#!/bin/bash

projPath=$(dirname $(winepath -u $EmWi_ProjectFile))
makePath=$(winepath -u $EmWi_OutputDirectory)

path=$(realpath --relative-to $makePath/../Project/GCC $projPath)

echo -e "\n\n vpath %c $path" >> $makePath/ewfiles.inc
