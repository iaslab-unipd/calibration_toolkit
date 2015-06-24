#!/bin/bash
# Ceres-solver installation following instructions at http://ceres-solver.org/building.html

# Install dependencies
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687 -y
sudo apt-get update
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y --force-yes

# Create build directory
mkdir /tmp/ceres_install
cd /tmp/ceres_install

# Download latest tested version: commit 5a21b8b1e9f0535c4b0710853bdb7bec42d5febc
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout 5a21b8b1e9f0535c4b0710853bdb7bec42d5febc
cd ..

# Build as a shared library
mkdir ceres-bin
cd ceres-bin
cmake -DBUILD_SHARED_LIBS=ON ../ceres-solver
make -j8 && make test

# Install
sudo make install

# Clean
sudo rm -r /tmp/ceres_install
