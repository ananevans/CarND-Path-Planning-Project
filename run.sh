#!/bin/bash


cd build/

./path_planning &

cd ../

sleep 1

~/work/self-driving-car-sim/test.x86_64
