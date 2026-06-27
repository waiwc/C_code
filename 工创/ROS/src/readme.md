
Autoware Simulation:
  roslaunch autoware_launch autoware.launch map_path:=/home/oliver/Documents/Projects/Zhisuo/resource/map/AutowareSampleMap_Rosbag rosbag:=true
  rosbag play --clock /home/oliver/Documents/Projects/Zhisuo/resource/bag/sample.bag -r 0.2
  roslaunch autoware_launch planning_simulator.launch map_path:=/home/oliver/Documents/Projects/Zhisuo/resource/map/AutowareSampleMap_Planning

**********************************************************************************************************************************************

SimOne Simulation:
  cd /home/oliver/Documents/Projects/Zhisuo/melodic-ros-base-bionic/zhisuo_package && ./connectSimOne.sh
  cp simone_planning.launch simone_perception.launch AutowareArchitectureProposal/src/launcher/autoware_launch/launch

  roslaunch autoware_launch simone_planning.launch map_path:=/home/oliver/Documents/Projects/Zhisuo/resource/map/SimOneMap
  roslaunch autoware_launch simone_perception.launch map_path:=/home/oliver/Documents/Projects/Zhisuo/resource/map/SimOneMap

  cd /home/oliver/Documents/Projects/Zhisuo/melodic-ros-base-bionic/zhisuo_package && ./msgs_transfer.sh 0/1
  Usage:  ./msgs_transfer
		0 - Truth Mode 		simone_planning
		1 - Perception Mode	simone_perception
