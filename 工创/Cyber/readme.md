build:
    cd Cyber
    ./gen_make.sh
    cd build
    make
run:
    cd Cyber
    tar -czvf run.tar.gz run
    docker cp run.tar.gz simone-apollo:/apollo/bazel-bin/CyberBridge/
    docker:
        cd /apollo/bazel-bin/CyberBridge
        tar -xvf run.tar.gz
        cd run
        修改config.ini中的BridgeIO_IP
        ./trans.sh

编译依赖了PCL库，这个只有在 TaskCyberPointCloud 中用到，如果没安装可以屏蔽掉，图像和点云在这里并没有用到（这部分之前的ROS工程中在rviz上做过可视化）

实际工程的编译环境在docker中，这里将编译依赖独立出来，编译过程中会有运行时依赖的动态库找不到，不影响生成可执行程序，放在docker中运行可以找到

