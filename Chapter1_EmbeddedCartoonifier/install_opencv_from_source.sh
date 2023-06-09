#!/bin/bash
# Linux script to download & build & install OpenCV from source, and install dependency packages.
# Should work on Debian or Ubuntu or Linux Mint or Raspbian, whether it is on a desktop PC or an
# embedded device such as a Raspberry Pi.
# By Shervin Emami 2016 (www.shervinemami.info)

# Exit the script if any command gives an error.
set -o errexit

# Set the version of OpenCV that we want.
VERSION=3.1.0
echo "This Linux script will download & install OpenCV $VERSION and its dependencies."
echo "Note that it can take anywhere between 15 minutes (on a PC) and 10 hours (on a RPi 1)!"

echo "Making sure we know where the latest dependency library packages are on the web ..."
sudo apt-get -y update

echo
echo "Installing many dependencies ..."

echo "Installing the compiler & build system ..."
sudo apt-get -y install build-essential make cmake cmake-curses-gui g++ pkg-config
echo "Installing libav video input/output development libraries ..."
sudo apt-get -y install libavformat-dev libavutil-dev libswscale-dev
echo "Installing video4Linux camera development libraries ..."
sudo apt-get -y install libv4l-dev
echo "Installing eigen3 math development libraries ..."
sudo apt-get -y install libeigen3-dev
echo "Installing OpenGL development libraries (to allow creating graphical windows) ..."
sudo apt-get -y install libglew1.6-dev
echo "Install GTK development libraries (to allow creating graphical windows) ..."
sudo apt-get -y install libgtk2.0-dev

echo
echo "Downloading OpenCV $VERSION source code including the contrib modules ..."
cd ~
wget --continue --tries=300 -O opencv-${VERSION}.zip https://github.com/Itseez/opencv/archive/${VERSION}.zip
wget --continue --tries=300 -O opencv_contrib-${VERSION}.zip https://github.com/Itseez/opencv_contrib/archive/${VERSION}.zip

echo "Unzipping OpenCV ..."
rm -rf ~/opencv-${VERSION} || true
rm -rf ~/opencv_contrib-${VERSION} || true
unzip opencv-${VERSION}.zip
unzip opencv_contrib-${VERSION}.zip

echo
echo "Configuring OpenCV settings using CMake ..."
cd ~/opencv-${VERSION}
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DBUILD_EXAMPLES=OFF -DWITH_OPENMP=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DOPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-${VERSION}/modules ..

echo
# Compile the code across the same number of threads as the CPU has cores,
# and if there is an error don't exit yet, try again using a single thread, for better error messages.
CPU_CORES=$(grep -c ^processor /proc/cpuinfo)
echo "Building OpenCV from source, directly on this machine using ${CPU_CORES} threads. Takes between 15 mins to 8 hours ..."
make -j ${CPU_CORES} || true
# Compile any remaining code using a single thread, since it occasionally has trouble using multiple threads.
make

echo
echo "Installing OpenCV $VERSION to a system folder ..."
sudo make install
sudo ldconfig

echo
echo "OpenCV installed successfully!"

