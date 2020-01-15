#! /bin/bash

# Script to rebuild aruco marker detector.
# Uncomment lines 94-96 in /opt/Downloads/aruco-1.3.0/CMakeLists.txt
# to rebuild aruco without openmp (faster!)

echo "Without openmp: Uncomment lines 94-96 in /opt/Downloads/aruco-1.3.0/CMakeLists.txt!"

#tar -xvzf aruco-1.3.0.tgz
cd aruco-1.3.0 
echo "Building Aruco 1.3.0" 
export OpenCV_DIR=/opt/opencv/3.0.0 
mkdir build 
cd build 
cmake -D CMAKE_INSTALL_PREFIX=/opt/aruco/1.3.0/ -D BUILD_SHARED_LIBS=ON -D BUILD_UTILS=ON .. 
make install -j6 
echo "/opt/aruco/1.3.0/lib" > /etc/ld.so.conf.d/aruco.conf
ldconfig
cd ../..
