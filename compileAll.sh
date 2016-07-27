#!/bin/bash
rootdir="$(dirname "$0")"
cd "${rootdir}"
absrootdir="$PWD"

#compile input ouput library
cd "${absrootdir}/io/"
mkdir build
cd build
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2

#compile examples and applications
cd "${absrootdir}/examples"
mkdir build
cd build
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2

cd "${absrootdir}/applications"
mkdir build
cd build
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
