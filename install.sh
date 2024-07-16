#! /bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "---------------------------------"
echo "Installing Dependencies..."
echo "---------------------------------"
sudo apt -q -y install build-essential cmake libomp-dev
sudo apt-get install libeigen3-dev

echo "---------------------------------"
echo "Installing lgmath..."
echo "---------------------------------"
cd $SCRIPT_DIR/third-party/lgmath
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS="-fPIC" ..
make
sudo make install

echo "---------------------------------"
echo "Installing steam..."
echo "---------------------------------"
cd $SCRIPT_DIR/third-party/steam
mkdir build
cd build
cmake -DCMAKE_CXX_FLAGS="-fPIC" ..
make
sudo make install

echo "---------------------------------"
echo "Installing norlab_trajectory..."
echo "---------------------------------"
cd $SCRIPT_DIR
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install

echo "---------------------------------"
echo "Done!"