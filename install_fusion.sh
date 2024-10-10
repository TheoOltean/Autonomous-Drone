#!/bin/bash

git clone https://github.com/xioTechnologies/Fusion.git

cd Fusion/Fusion

gcc -lm -c -O3 *.c
ar rcsv libfusion.a *.o
mv libfusion.a /usr/local/lib

mkdir /usr/local/include/Fusion
mv *.h /usr/local/include/Fusion

cd ../../
rm -r Fusion
