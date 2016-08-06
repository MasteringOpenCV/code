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

echo "Making sure we know where the latest dependency library packages are on the web ..."
sudo apt-get -y update

echo
echo "Installing many dependencies ..."

echo "Installing the compiler & build system ..."
sudo apt-get -y install build-essential make cmake cmake-curses-gui g++
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
echo "Downloading OpenCV $VERSION source code ..."
cd ~
wget --continue --tries=300 https://github.com/Itseez/opencv/archive/${VERSION}.zip
echo "Unzipping OpenCV ..."
rm -rf ~/opencv-${VERSION} || true
unzip ${VERSION}.zip

echo
echo "Configuring OpenCV settings using CMake ..."
cd ~/opencv-${VERSION}
mkdir build
cd build
cmake -DWITH_OPENMP=ON -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF ..

echo
echo "Building OpenCV from source, directly on this machine. Takes 15-90 minutes ..."
make -j4

echo
echo "Installing OpenCV $VERSION to a system folder ..."
sudo make install

echo
echo "OpenCV installed successfully!"

