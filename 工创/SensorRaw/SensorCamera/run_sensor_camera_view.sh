#!/bin/bash
cd `dirname $0`
CurrentPath=$(pwd)
export LD_LIBRARY_PATH=$CurrentPath/build/bin:$LD_LIBRARY_PATH
$CurrentPath/build/bin/SensorCameraView 127.0.0.1 13956
