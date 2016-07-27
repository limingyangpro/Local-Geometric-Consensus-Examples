#!/bin/bash
rootdir="$(dirname "$0")"
cd "${rootdir}"
absrootdir="$PWD"

#compile input ouput library
cd "${absrootdir}/io/build"
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2

#compile examples and applications
cd "${absrootdir}/examples/build"
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2

cd "${absrootdir}/applications/build"
echo "Compiling subdirectory $PWD..."
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
