ps -ef | grep /apollo/cyber | grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7
ps -ef | grep mainboard | grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7
ps -ef | grep dreamview | grep -v -E 'grep' | awk -F " " '{print $2}' | xargs kill -7
