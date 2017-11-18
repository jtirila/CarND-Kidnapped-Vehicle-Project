by **Juha-Matti Tirilä** 

# Overview
This repository is based on the Udacity repository for the third project of the second term of the Self-Driving 
Car Nanodegree.

My additions to the repository implement the remaining parts of the project and slightly refactor some of the 
provided code templates. 

### Notes on the Submission

Even though it would have been sufficient to modify `particle_filter.cpp` only, I have also made 
slight modifications elsewhere and defined helper functions outside of the source file. I hope this won't matter.  

Here are some notes concerning individual template files: 
* `main.cpp`: just whitespace changes (use space indent)
* `particle_filter.h`: 




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

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

For further instructions and technical notes, please refer to the [corresponding Udacity repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).
