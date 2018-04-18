# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Reflection

Model predictive control reframes the task of following a trajectory as an optimization problem. Model predictive control involves simulating different actuator inputs, predicting the resulting trajectory and selecting trajectory with the minimum cost. Here the selected trajectory is used to simulate the movement of the car in the simulator to ensure that it mimics the reference trajectory path.

The data which is provided by the simulator are as follows

* ptsx (Array) - The global x positions of the waypoints.
* ptsy (Array) - The global y positions of the waypoints. Thiscorresponds to the z coordinate in Unity since y is the up-down direction.
* psi (float) - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* psi_unity (float) - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
* x (float) - The global x position of the vehicle.
* y (float) - The global y position of the vehicle.
* steering_angle (float) - The current steering angle in radians.
* throttle (float) - The current throttle value [-1, 1].
* speed (float) - The current velocity in mph.

Here we also have to consider a latency of 100 ms of the actuators of the car while calculating the trajectory.

##### Set N and dt and convert waypoints to car co-ordinates
The first step in the model is to set the N(number of timesteps) to 25 and the time interval between 2 time steps dt as 0.05 so that we can predict the trajectory for the next 1.25 seconds as a starting point. The waypoints that are provided( ptsx and ptsy arrays) are first converted to vehicle co-ordinates by subtracting px and py from each value in the arrays to adjust the position according to that of the vehicle(ie. translation is performed). Further the waypoint co-ordinates are changed using the homogeneous transformation matrix to calculate the waypoints in terms of vehicle co-ordinates using the following equations(Rotation).

* ptsx_car_co_ord[i]=x_diff* cos(-psi) - y_diff* sin(-psi);
* ptsy_car_co_ord[i]=x_diff* sin(-psi) + y_diff*cos(-psi);


##### Fit polynomial to waypoints and calculate cte and epsi
The next step in the model is to fit the transformed waypoints to a third degree polynomial using the polyfit to obtain the path/trajectory that is to be followed by the vehicle. Since the points have been converted to vehicle co-ordinates the x, y and psi values can be considered at the origin ie zero  initially and further the cross track error is calculated by using the polyeval function which takes the coefficients returned by the polyfit function and the current x value and subtracts the current y from it. The orientation error epsi is calculated by taking the difference of the psi  and arctan  of the first derivative of the third degree polynomial calculated. The values of x, y and psi are substituted and the values are calculated accordingly.

##### MPC solve
The calculated coefficients and the state vector is then passed to the solve function in which the  variables (which includes the state size, actuators, and timesteps) are first set to zero except the first variable, which is set to the respective value from the current state. The variables then have upper and lower boundaries set for their values.  The delta(Steering angle) is limitted between - 25 to 25 degrees (in radians) and the a ie throttle is limitted within the values  -1 to 1. After setting all the contraints and bounds the operator function of the FG eval class performs the cost additions and state value calculations as mentioned below. The updated state vector along with the variables(with constraints and bounds)are passed to the ipopt solver to produce the future predictions for the x, y , delta and acceleration values.

##### FGeval- Adding cost and calculating state values for N time steps

The next step is to minimize the cross track errors, orientation error and  velocity error , which can be done by adding these parameters to the cost fg[0] as seen in line 57-59 of MPC.cpp

 A further enhancement is to constrain erratic control inputs. In order to do this we add the steering angle and acceleration also to the cost  to ensure smoother turns and proper change in acceleration. We can further add the difference between consecutive steering angles and orientations to the cost to make the trajectory smoother and avoid radical changes. 


 After adding cost the next step would be to calculate each of the parameters in the state vector for the next N timesteps in the FG eval class.


##### Dealing with effects of latency

To account for the latency of 100ms between the actuator calculation and  when the state is actually reached and additional step was added to predict the state of the vehicle with a dt(latency) of 100ms using the model update equations as seen in line  keepin the values of x, y and psi as 0 (assuming to be at the origin of vehicle co-ordinates).This state was used to make future predictions instead of the older values(the ones without considering the latency) and was passed to the solve function to predict future states and actuator values.

##### Tuning

The values provided in the classroom sessions ie N and dt of 25 and 0.05 was working well for a reference velocity of 40km/hr. In order to further increase the speed of the vehicle and still ensure that the vehicle follows the right trajectory without taking sharp turns, I added an additional weight of 100 to the sequential steering angle difference component of the cost. This did not give very good results at higher speeds and indicated that the weights had to be increased further and hence I tried 300 and 500 and found that a weight of 500 gave better results. Later I also added weights to the individual steering angle component of the cost to ensure smoother turns which led to good results at a reference velocity of 50km/hr.I also tried adding weights to the cte, epsi and velocity cost comonents with little success hence I went back to keeping them at 1. Further I also tried varying the time steps and time interval N and dt to further optimize the performance by changing the dt to 0.1 and keeping the timesteps at 25 led to a prediction for 2.5 seconds with very less intermediate points and hence could not handle the turns properly and went off the track. Later I reduced the time steps to 20 and the time interval to 0.06 to get predictions for the next 1.2 seconds which seemed to work well with the tuned values.


 





