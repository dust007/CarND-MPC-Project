# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program
* By Xiangjun Fan

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Model Predictive Controller
MPC is a non-linear system which generates optimized parameters with  the constraint of obtaining minimal value of cost function. Here, cost  function refers to the problem built by taking into consideration the  model of the system, range of inputs, limitations on outputs and/or the  effect of external factors acting on the system.

A typical cost function in case of a self-driving vehicle will have following constraints:

1. The cross track error (cte), i.e. the distance of vehicle from the center of the lane must be minimal.
2. The heading direction of the vehicle must be close to perpendicular  to the lane. Error PSI (epsi) is the error in heading direction
3. Oscillations while riding the vehicle must be minimal, or one would  feel sea sick. This takes into account the change in heading direction  due to turns and also the change in speed due to acceleration/braking.
4. The vehicle must drive safely and should not exceed the speed limit.  In contrast, the vehicle must also not drive too slow as one wouldn't  reach any place.

These constraints are merged to form a cost function. MPC tries to  reduce the cost function and come up with the values of actuation inputs  to the vehicle, which can be the steering angle of the wheel and/or the  throttle/brake magnitude.

## Simulation Result
[./sim.mp4](./sim.mp4)

## Project Implementation

The final implementation consisted of following major steps:

* Align of data relative to the motion of car:
In this step, the coordinates of waypoints were transformed from  global map system to vehicle's coordinate system. This was done to  ensure the calculations of cte and epsi were less complex and involved  less calculations.
* Generation of reference trajectory from waypoints:
The transformed coordinates of waypoints were then used to create a  smooth curved trajectory, which will act as a reference for motion of  the car. 
* Calculation of CTE and EPSI:
* Definition of motion model, control/actuator inputs and state update equations:
The state consists of following parameters: 1. The x coordinate of position of car in vehicle's coordinate system (x) 2. The y coordinate of position of car in vehicle's coordinate system (y) 3. The heading direction of car in vehicle's coordinate system (psi) 4. The magnitude of velocity of car (v)
The actuator inputs used to control the car are given below: 1. The magnitude of steering (delta). This was limited to [-25, 25] as per the angle specification of simulator. 2. The magnitude of throttle (a). This was limited to [-1, 1] as per the throttle/brake specification of simulator.
* Definition of time step length (N) and duration between time steps (dt):
Given the reference trajectory from polynomial fit of waypoints and  the motion model of the car, MPC estimates the value of actuator inputs  for current time step and few time steps later. This estimate is used to  predict the actuator inputs to the car ahead of time. This process of  estimate generation is tunable with the use of N and dt. Higher value of  N ensures more number of estimates while higher value of dt ensures the  estimates are closer in time. 
After trial and error, a setting of **N = 7** and **dt = 0.07 (sec)** was used to predict actuator inputs and the trajectory of car for  roughly next 500ms. 
* Definition of desired behavior and actuator constraints:
In order to define the cost function of the system, it was essential  to list down the desired values of different parameters. They are given  below:
	1. Expected value of CTE to be zero
	2. Expect value of EPSI to be zero
	3. Maximum speed of the car to be 100mph. 
This was a tunable parameter  and the goal was to test the maximum speed at which the car stays on the  track and moves safely.
* Definition of cost function for MPC:
The last step in the implementation is to define the cost function for MPC.Key elements and features of the cost function are  given below:
    1. Highest weight for calculated CTE and EPSI. This was to ensure the car stays in the middle of lane and head in desired direction
    2. Reduce high initial values of control inputs (delta and a) to ensure there is no jerk in motion of the car
    3. Minimize the change in values of control inputs (delta and a) in order to minimize oscillations in the motion
    4. Minimize speeds at higher steering angle and minimize steering at  higher speeds. This was to ensure the car took smooth turns by reducing  the speed while it reached maximum possible speed on straight ahead path

## Incorporating effect of latency in the control actuations:

In real world systems as complex as commercial jet planes, there  exists certain amount of delay in time between the actuation of control  and its effect on the motion. To achieve an implementation close to real  world scenario, a latency of 100ms was introduced in the simulator.  This delay caused control actuations to reach the car in later of time.  This worked fine at low speeds till 25mph but resulted in undesired  behavior at high speeds. To take into account the effect of this  latency, the state parameters for next state were calculated beforehand  and were sent to MPC for generating steering and throttle values. This  ensured the actuations applied at current point of time were actually  for the next time step (i.e. after 100 ms). This small update in state  calculation solved the problem of latency and the car was back on track  w.r.t. its desired behavior.