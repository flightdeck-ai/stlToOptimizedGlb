# STL to GLB service
This C++ REST API converts a STL File to a GLB File

## Setup
- Git clone 
- Make a debug build using `cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_MAKE_PROGRAM=/usr/bin/make -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ -G "CodeBlocks - Unix Makefiles" -S . -B cmake-build-debug`
- launch using `./cmake-build-debug/stlToOptimizedGlb`
- If you use CLion it automates that for you, so you can skip the above 2 steps

## License
This project is licensed under AGPL-3.0. 3rd Party Licenses can be found in `licenses/`