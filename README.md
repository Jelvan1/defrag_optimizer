## Building
This project is writtin in C++ and uses CMake to control the building process.
1. Create a separate `build` directory (keeps your repository clean).
   ```
   $ mkdir build
   $ cd build
   ```
2. Generate input files for a native build system.
   To generate Ninja files for a 32 bit debug build on Linux with g++, do:
   ```
   $ cmake -G Ninja             \
       -DCMAKE_BUILD_TYPE=Debug \
       -DCMAKE_CXX_COMPILER=g++ \
       ..
   ```
3. Build the source code.
   ```
   $ cmake --build .
   ```
4. Profit.
