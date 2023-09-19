# libelas-gpu

This is an implementation of the well known [libelas](http://www.cvlibs.net/software/libelas/)
(LIBrary for Efficient LArge-scale Stereo matching) library for
sparse large real-time calculation of stereo disparity images.
This is for a college course final project and thus does not have
any support of any kind. The original source code can be found
in the CPU directory. This has been commented as we explore the code.


From there, the GPU cuda implementation of key methods can be found
in the GPU folder. For methods not implemented on the GPU the CPU
version is called, and such code is run on the CPU.

## Differences

The key difference is the changing of most of the methods in the ELAS class
to being virtual. This allows for the sub-classes to override such methods.
This allows for seamless transition between the GPU and CPU code. Additionally,
the methods where converted to public to allow for the testing and comparison of the two methods directly.

## Building

* First create a build directory `mkdir build`
* Move into this directory `cd build`
* Run the c++ [cmake](https://cmake.org/) build system `cmake ..`
* Finally build all the executables `make -j5`
* To build a single one preform the following `make -j5 <name-of-exc>`
* To run the program do the following
    - `./libelas_cpu demo`
    - `./libelas_gpu`


## Licenses

All the original code is licensed under the **GNU General Public License**. This can be found with the original  [libelas](http://www.cvlibs.net/software/libelas/) library. All other code is licensed under the MIT license, which is attached to this repo's LICENSE file.
