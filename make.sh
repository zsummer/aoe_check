#/bin/bash
if [ ! -d ./local ]; then
	mkdir ./local
fi
cd ./local
#cmake ../ -DCMAKE_C_COMPILER="/usr/bin/gcc-6" -DCMAKE_CXX_COMPILER="/usr/bin/g++-6"
cmake ../ 
make -j4
cd ..

