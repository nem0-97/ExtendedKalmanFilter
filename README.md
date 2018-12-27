# Extended Kalman Filter

This code as is will interact with the Simulator found [here](https://github.com/udacity/self-driving-car-sim/releases)

##### INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


##### OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

The simulator after receiving the output from the program will use that to draw a green triangle where the program tells it the car is based on the sensor measurements passed to it. The program uses sensor fusion to track the x and y positions and velocities of the object being tracked(a car in the simulator in this case).

You can modify it to run with other data, see data folder for recommended data input format.

## Dependencies:

To build and run as is this project requires:

To run:
  1. [Simulator](https://github.com/udacity/self-driving-car-sim/releases)
  2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
  3. [Eigen Library](http://eigen.tuxfamily.org/index.php?title=Main_Page)(Already included in src folder)

To Build:
  4. cmake >= 3.5
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
  5. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
  6. gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)

## Build and Run:

 The main program can be built and run by running the following commands from the project's top directory(this has already been done):

 If no changes made to code just run:
 1. Run it: `./build/ExtendedKF `

 Or build with changes and run:

 1. Remove old build directory and make new one: `rm -r build && mkdir build && cd build`
 2. Compile: `cmake .. && make`
    * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
 3. Run it: `./ExtendedKF `

## Main Structure:

In src the following files:
 1. Main.cpp

    Builds MeasurementPackage object containing the inputs passed to program for a single measurement, passes this object to a FusionEKF object to process, calls Tools object to calculate RMSE between predictions and ground truths passed in , then gets results from both objects and combines it into a JSON object containing estimated positions and RMSE for the x and y positions and velocities.

 2. FusionEKF.cpp

    Contains KalmanFilter object used to store state as well as measurement covariance matrices for each sensor, and a ProcessMeasurement method which takes a measurement package and calls the KalmanFilter object's Predict and the proper Update method depending on the sensor type for the measurement.

 3. kalman_filter.cpp

    Contains Predict, Update, and UpdateEKF methods for the Kalman filter and Extended Kalman filter, also stores the current state, x and y positions and velocities, of object being tracked. Used to perform actual calculations during updates and pedictions. UpdateEKF is used for radar measurements since it is non-Linear to get from measurement to state, and Update for Lidar. Both Update methods take in z, a vector containing the measurement.

 4. tools.cpp

    Contains methods for calculating RMSE given a list of guesses and one of actual values and for calculating the Cartesian to Polar Jacobian matrix given some x and y positions and velocities.
