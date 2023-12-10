#!/bin/bash

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
source "${TOP_DIR}/scripts/smartsim.bashrc"

os_platform=$1
middleware=$2

if [ $os_platform == 'x86' ]; then
  info "Compile CM platform library based on x86 linux!"
  toolchain_cfg="${TOP_DIR}/toolchain/toolchain_x86.cmake"
elif [ $os_platform == 'ARM' ]; then
  info "Compile CM platform Library based on ARM!"
  toolchain_cfg="${TOP_DIR}/toolchain/toolchain_aos.cmake"
else
  info "Unsupported OS platform with \$1: $os_platform"
  exit
fi


if [ $middleware == 'ROS' ]; then
  info "Compile CM platform library based on ROS!"
elif [ $middleware == 'AP' ]; then
  info "Compile CM platform library based on AUTOSAR AP!"
elif [ $middleware == 'ART' ]; then
  info "Compile CM platform library based on Apollo Cyber RT!"else
  info "Unsupported middleware with \$2: $middleware"
  exit
fi

build_path="${TOP_DIR}/build/${os_platform}_${middleware}"
install_path="install"

if [ ! -d $build_path ]; then
  mkdir -p $build_path
  info "create build folder!"
else
  #rm -rf $build_path/*
  info "clear all temporary files!"
fi

cd $build_path
cmake ../.. -DCMAKE_TOOLCHAIN_FILE=$toolchain_cfg -DCMAKE_MW=$middleware -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=$install_path -DCMAKE_VERSION=3.20 ${@:5}

make -j8 VERBOSE=1
# make -j8
# make install