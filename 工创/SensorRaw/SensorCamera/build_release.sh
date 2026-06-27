#!/bin/bash
pushd ../3rdparty
echo Init sensor 3rdparties, please wait...
if [ ! -d "pcl" ]; then
  unzip -q Sensor3rdparty-Linux.zip
fi
echo Init sensor 3rdparties succeed.
popd

if [ ! -d "build" ]; then
  mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles"
make -j8

