@echo off
set CurrentPath=%cd%
cd %CurrentPath%
path %ProgramFiles%\CMake\bin;%PATH%

pushd ..\3rdparty
echo Init sensor 3rdparties, please wait...
if not exist pcl start zip/7z.exe x Sensor3rdparty-win64.zip
echo Init sensor 3rdparties succeed.
popd

rmdir /s /q build
if not exist build md build
cd build
cmake .. -G "Visual Studio 15 2017 Win64" -DCMAKE_BUILD_TYPE=Release
cd ..
