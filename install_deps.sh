#!/bin/bash

echo "Installing dependencies."
mkdir deps
cd deps

echo "Building GTSAM for python."
echo "Install system dependencies."
sudo apt-get install -y cmake libboost-all-dev libtbb-dev git python-dev

echo "Cloning and checking out working commit."
git clone https://bitbucket.org/gtborg/gtsam.git
cd gtsam
git checkout 5a8bd5a

echo "Preparing compilation."
mkdir build
if [ -d cython_build ] ; then
  rm -rf cython_build
fi
mkdir cython_build
cd cython
sudo pip install --upgrade pip
sudo pip install numpy
sudo pip install Cython
sudo pip install eigency
sudo pip install graphviz
cd ..
cd build
cmake -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON -DGTSAM_CYTHON_INSTALL_PATH=../cython_build ..

echo "Building gtsam (change script for make install)."
make -j4
#cd cython
#cmake -P cmake_install.cmake
#cd ..
## Optional if you want to use gtsam with cpp
sudo make install
sudo ldconfig

echo "Copying gtsam to scripts folder (change script to install globally)"
cd ..
cd cython_build
## Install gtsam locally in scripts
if [ -d ../../../scripts/slambackends/gtsam ] ; then
  rm -rf ../../../scripts/slambackends/gtsam
fi
cp -R gtsam ../../../scripts/slambackends/gtsam
## Or install it globally
#sudo cp -R gtsam /usr/local/lib/python2.7/dist-packages/gtsam

echo "Removing temporary build directories."
cd ..
cd ..
#rm -rf gtsam
echo "Done building gtsam."


echo "Removing temporary build directories."
cd ..
#rm -rf deps
echo "Done. All dependencies installed."
