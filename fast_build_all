#! /bin/bash
echo ""
echo "------------------------------------ Building ------------------------------------"
echo ""


ARCH=$(uname -m)
if [ "$ARCH" == "aarch64" ]
then
    CMAKE_OPTIONS="-DARCH_DIR=$ARCH -DCMAKE_C_COMPILER=/usr/bin/gcc-5 -DCMAKE_CXX_COMPILER=/usr/bin/g++-5"
else
    CMAKE_OPTIONS="-DARCH_DIR=$ARCH"
fi

catkin_make --cmake-args $CMAKE_OPTIONS

. ./devel/setup.sh

