# This is the version adapted for ROS2 of the quadruped kinematics library, which you can take a look at by clicking [here](https://github.com/vertueux/quadruped_kinematics)
Quadruped kinematics is a C++ library created to handle operations on a quadruped robot. There are no external dependencies like eigen3 or another C++ math library needed.
This project uses `std::vector` to create matrices. You can adapt this library to your needs.

> **Here are the last steps to complete in order to enter a stable version of this repository:**
> * ~~Program an algorithm to find the inverse of a 4x4 matrix based on the std::vector~~
> * Run tests to verify that the entire program is working properly (90% test finished)

![license](https://img.shields.io/badge/license-MIT-important)
![discord](https://img.shields.io/badge/Contact%20me%20on%20Discord-now%239470-informational)

# Getting Started 
You need to have CMake installed to build the project on your computer. You can go to the direct link to download by clicking [here](https://cmake.org/download/). Follow all instructions on the CMake site.

# LICENSE
```
MIT License

Copyright (c) 2022 Virtuous

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```