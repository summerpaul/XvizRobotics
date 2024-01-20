#!/bin/bash
# @Author: Xia Yunkai
# @Date:   2023-12-26 20:56:25
# @Last Modified by:   Xia Yunkai
# @Last Modified time: 2023-12-26 21:00:10

rm -rf build
mkdir build
cd build
cmake ..
make 