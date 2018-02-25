[//]: # (Image References)

[image1]: ./output/RMSE_position.png "RMSE position"
[image2]: ./output/RMSE_velocity.png "RMSE velocity"
[image3]: ./output/dataset1_result.png "Dataset 1 final RMSE"
[image4]: ./output/dataset2_result.png "Dataset 2 final RMSE"
[image5]: ./output/dataset1_lidarOnly_result.png "Dataset1 update using Lidar only"
[image6]: ./output/dataset1_radarOnly_result.png "Dataset1 update using Radar only"
[image7]: ./output/dataset2_lidarOnly_result.png "Dataset2 update using Lidar only"
[image8]: ./output/dataset2_radarOnly_result.png "Dataset2 update using Radar only"
[image9]: ./output/process_flow.png "Process flow"
[image10]: ./output/lidar_init.png "Lidar initialization"
[image11]: ./output/radar_init.png "Radar initialization"
[image12]: ./output/P_matrix.png "P matrix"
[image13]: ./output/prediction_formula.png "Prediction"
[image14]: ./output/update_formula.png "Update"
[image15]: ./output/f_equation.png "F matrix"
[image16]: ./output/q_equation.png "Q matrix"
[image17]: ./output/h_equation.png "H matrix"
[image18]: ./output/extend_update_formula.png "Update EKF"
[image19]: ./output/radar_h_equation.png "Radar transformation equation"
[image20]: ./output/hj_equation.png "Hj matrix"

# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

### Your code should compile
Refer to the instruction in section "Basic Build Instructions" to compile the code and run.

### Accuracy
Here is the accuracy from the test result.

#### Dataset 1

The final RMSE is px=0.0973, py=0.0855, vx=0.4513 and vy=0.4399

![alt text][image3]

The following graph shows how RMSE of px, py, vx and vy converge over time.

![alt text][image1]

![alt text][image2]


### Processing flow of the Sensor Fusion algorithm

The sensor fusion algorithm reads from both lidar and radar sensors for input.
First, in the Predict stage, the EKF (extended Kalman filter) predicts the position and velocity of the car using a given model. (In this project, I use Constant velocity model)
Then, in the Update stage, EKF uses the lidar or radar input values to update the position and velocity of the car.
By doing so, it reduce the effect of noise from the lidar and radar sensors.

The picture below shows the working mechanism of the EKF.

![alt text][image9]

### Handle the first measurement
The initial measurement can be taken from Lidar data and Radar data.

For lidar data (x, y), set 

![alt text][image10]

For radar data (rho, phi, rho_head), set
 
![alt text][image11]

The initial covariance matrix P is

![alt text][image12]


### Predict and update loop, and handling Radar and Lidar measurement

After the initialization part, all the lidar and radar readings go through the predict
and update loop.

The equations of the prediction part are as follow:

![alt text][image13]

Where F and Q are of matrices in following forms: 

![alt text][image15]

![alt text][image16]

At this stage, the EKF returns its estimate of x and P, which describe the probability that 
what the actual position and velocity of the car are.  

Then EKF, in the update part, uses the measured values from sensors to come up with a more precise prediction.

When processing measurements from lidar sensor, Kalman filter is used.

![alt text][image14]

where H is a matrix or a linear function of transforming the internal state (px, py, vx, vy) to the measurement (px, py).
H has the following form.

![alt text][image17]


When processing measurements from radar sensor, extended Kalman filter is used because h is not a linear function.

![alt text][image18]

function h takes the following form

![alt text][image19]

where Hj is the Jacobian matrix of function h, and has the following form.

![alt text][image20]


### Performance analysis (Combined vs Lidar vs Radar)
#### Dataset 1

Lidar only performance

![alt text][image5]

Radar only performance

![alt text][image6]

Lidar and Radar performance

![alt text][image3]

#### Dataset 2

Lidar only performance

![alt text][image7]

Radar only performance

![alt text][image8]

Lidar and Radar performance

![alt text][image4]

#### Conclusion
From the above 2 datasets, we can see that using both lidar and radar readings as input to the fusion sensor helps 
fusion sensor to make accurate prediction. It is not surprise because the lidar and radar measurements come after each 
other after 0.5 seconds. Using only lidar or radar measurement half the sampling frequency, and hence the error increases.

In the case which lidar or radar is used alone. Lidar give a better final RMSE in both dataset (especially true for predicting px and py).
