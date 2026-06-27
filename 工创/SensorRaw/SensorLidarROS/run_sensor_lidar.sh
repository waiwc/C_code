#!/bin/bash
cd `dirname $0`
CurrentPath=$(pwd)
export LD_LIBRARY_PATH=$CurrentPath/build/bin:$LD_LIBRARY_PATH
$CurrentPath/build/bin/SensorLidarROS 127.0.0.1 6699 7788 /sensing/lidar/pointcloud lidar
