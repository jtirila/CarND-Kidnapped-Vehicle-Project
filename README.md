by **Juha-Matti Tirilä** 

# Overview
This repository is based on the [Udacity repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project) for the third project of the second term of the Self-Driving 
Car Nanodegree.

My additions to the repository implement the remaining parts of the project and slightly refactor some of the 
provided code templates. 

## Project Introduction
My robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project have implemented a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter gets observation and control data. 

## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively, check out these scripts for doing the same with fewer commands: `./clean.sh`, `./build.sh` and `./run.sh`.

For further instructions and technical notes, please refer to the [original Udacity repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).

# Notes on my Submission

Even though it would have been sufficient to modify `particle_filter.cpp` only, I have also made 
slight modifications elsewhere and defined helper functions outside of the source file. I hope this won't matter.  

Here are some notes concerning individual template files: 
* `main.cpp`: Just whitespace changes (use space indent)
* `helper_functions.h`: Add a couple of functions that felt too generic to be included the particle filter class, e.g. 
   my own version of a function that computes the Euclidean distance between two vectors
* `particle_filter.h`: 
  - Some slight changes to the proposed signature of the functions
    x Using `const` on some occasions where nothing is modified
    x Using the appropriate data type for map landmark data points 
  - Add the declaration of one more helper function responsible for extracting the weights
    out of the particles vector
  - change from tab indentation to space indentation  
* `particle_filter.cpp`: 
  - the implementation of the missing functionality, reflecting the changes in `particle_filter.h`
  - Use space indentation instead of tab indentation
  
## Some Thoughts on the Implementation 

Even though the code works fine, it could be made more generic and decoupled from the related data types. Specifically, 
I think there should ideally be no references to the project specific data structures in `helper_functions.h` to better 
avoid cyclic dependencies and also to make the `helper_functions.h` code more generic. Then again, the whole helper 
function structure could use some re-thinkin. I think that a namespace would be better suited for this purpose. I did 
this for some of the earlier projects in this Term but did not bother this time.  
