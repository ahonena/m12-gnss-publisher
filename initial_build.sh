#!/bin/sh
set -e
#cd src/IDL;
cd src;
git clone https://github.com/ahonena/m12-idl.git;
cd m12-idl;
cd IDL;
./build_types.py;
./command_for_includes.sh;
cd ..; cd ..; cd ..;
mkdir build;
cd build;
cmake ..;
make;

