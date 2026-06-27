CurrentPath=$(pwd)
export BridgeNodePath=$CurrentPath

echo $BridgeNodePath

# change the following ip to your coordinator ip
export CoordinatorIp=10.2.35.36
export CoordinatorPort=4500

export LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH

./CybertronDaemon "CPU Node 0" -c "{\"capableImages\":[\"CybertronBridgeSM\"],\"capableImagePaths\":[\"$BridgeNodePath/\"], \"port\":25002}" -h $CoordinatorIp -p $CoordinatorPort

