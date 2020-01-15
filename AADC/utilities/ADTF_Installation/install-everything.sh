#! /bin/bash

echo "Install nedded ADTF libs"
apt-get install libsdl-console
echo "Add user aadc to dialout group (for arduino)"
usermod -a -G dialout aadc


echo "Install ADTF to /opt/adtf/2.13.1"
bash ./adtf-2.13.1-linux64.run
echo "Install ADTF Compression Toolbox to  /opt/adtf/2.13.1"
bash ./adtf-compression-toolbox-2.5.0-adtf2.10.0-linux64.run
echo "Install ADTF Display Toolbox to  /opt/adtf/2.13.1"
bash ./adtf-display-toolbox-2.1.1-adtf2.12.0-linux64.run

echo "Installing common libs"
apt-get install ssh
apt-get install g++
apt-get install xrdp
#apt-get install xubuntu-desktop
#apt-get install xfce4
apt-get install gnome-icon-theme-full
apt-get install sl

#echo "Fix Keyboard Tab Key for Remote Desktop"
#cp xfce4-keyboard-shortcuts.xml /home/aadc/.config/xfce4/xfconf/xfce-perchannel-xml/xfce4-keyboard-shortcuts.xml


echo "Install needed QT dev libs"
apt-get install libx11-dev
apt-get install libXext-dev
apt-get install libgl1-mesa-dev
apt-get install libglu1-mesa-dev

echo "Building QT"
new script
tar -xvzf  qt-everywhere-opensource-src-4.7.1.tar.gz
cd qt-everywhere-opensource-src-4.7.1
echo "Configuring Qt4"
mkdir /opt/qt
mkdir /opt/qt/4.7.1
./configure -no-webkit -no-declarative  
echo "Building QT4"
make -j6
cd ..
echo "Installing QT4"
cp -r qt-everywhere-opensource-src-4.7.1/* /opt/qt/4.7.1
cp qt.conf /opt/qt/4.7.1/bin/


echo "/opt/qt/4.7.1/lib" > /etc/ld.so.conf.d/qt4.conf

echo "Installing needed OpenNI dev libs"
apt-get install  libopenni-sensor-primesense0
apt-get install libusb-1.0-0-dev
apt-get install libudev-dev

echo "Building OpenNI2"
unzip OpenNI2-develop.zip
cd  OpenNI2-develop
make -j6
cd ..
echo "Add Xtion dev to video group"
./Packaging/Linux/install.sh
echo "Add user aadc to video group"
usermod -a -G video aadc
mkdir /opt/openni
cp -R OpenNI2-develop /opt/openni/
mv /opt/openni/OpenNI2-develop  /opt/openni/2.3.0
echo "/opt/openni/2.3.0/Bin/x64-Release" > /etc/ld.so.conf.d/openni2.conf
ldconfig

echo "Installing needed OpenCV 3.0.0 libs"
apt-get install libtbb-dev
apt-get install cmake

unzip opencv-3.0.0.zip
cd  opencv-3.0.0

echo "Configuring OpenCV 3.0.0" 
export OPENNI2_INCLUDE="/opt/openni/2.3.0/Include" 
export OPENNI2_REDIST="/opt/openni/2.3.0/Bin/x64-Release" 
mkdir build 
cd build 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/opt/opencv/3.0.0 -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_OPENGL=ON -D INSTALL_C_EXAMPLES=ON -D BUILD_EXAMPLES=ON -D WITH_IPP=ON -D WITH_OPENNI2=ON -D WITH_OPENNI=OFF .. 
echo "Building OpenCV 3.0.0" 
make install -j6 
echo "/opt/opencv/3.0.0/lib" > /etc/ld.so.conf.d/opencv.conf
ldconfig
echo "OpenCV 3.0.0 ready to use"
cd ../..

tar -xvzf aruco-1.3.0.tgz
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

unzip osg_3.2.0_linux64.zip 
mkdir /opt/osg
mkdir /opt/osg/3.2.0
cp -r osg_3.2.0_linux64/* /opt/osg/3.2.0/
rm -rf /opt/osg/3.2.0/lib64/*.100
rm -rf /opt/osg/3.2.0/lib64/*.13
echo "/opt/osg/3.2.0/lib64" > /etc/ld.so.conf.d/osg.conf
ldconfig
