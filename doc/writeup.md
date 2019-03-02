# Writeup: MPC Project

## 1. The Model

The state variable in the model is constructed as below,

```
state = [ x
          y
          psi
          v
          cte
          epsi ]
```
          
The model used is a Kinematic model neglecting the complex interactions between the tires and the road. The model equations are as follow:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

where,

- `x, y` : Car's position.
- `psi` : Car's heading.
- `v` : Car's velocity.
- `cte` : Cross track error.
- `epsi` : Heading error.

In addition to those above, `Lf` is the distance between the car of mass and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

- `delta` : Steering angle.
- `a` : Car's acceleration (throttle).

The objective is to find the steering angle `delta` and the acceleration `a` by means of minimizing the value of a cost function which is the combination of several factors:

- Square sum of `cte` and `epsi`. It could be found [here](./src/MPC.cpp#L71).
- Square sum of the difference actuators to penalize a lot of actuator's actions. It could be found [here](./src/MPC.cpp#L78).
- Square sum of the difference between two consecutive actuator values to penalize sharp changes. It could be found [here](./src/MPC.cpp#L86).

[The weights assigned to each of these factors](./src/MPC.cpp#L61) were tuned manually to obtain a stable and smooth track ride closest to the reference trajectory.

## 2. Timestep Length and Elapsed Duration (N & dt)

I tried several combinations of N and dt for comparison as shown in the below table,

|   N     |   dt     |
| ------- | -------- |
|    25   |   0.01   |
|    10   |   0.05   |
|    10   |   0.1    |

The number of points `N` and the time interval `dt` determine the prediction horizon. The number of points impacts the controller performance as well. I tried to keep the horizon around the same time the waypoints were on the simulator. With too many points the controller starts to run slower, and some times it went wild very easily. After these trials, I finally picked the last combination which contributes a smoother drive.

## 3. Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system at [./src/main.cpp](./src/main.cpp#L110) from line 110 to line 115. Then a 3-order polynomial is fitted to the transformed waypoints. These polynomial coefficients are used to calculate the `cte` and `epsi` later on. They are used by the solver as well to create a reference trajectory.

## 4. Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found at [./src/main.cpp](./src/main.cpp#L125) from line 125 to line 135.

## 5. Simulation

*The vehicle must successfully drive a lap around the track.*

The vehicle successfully drives a lap around the track. Here is a short video with the final parameters: [./videos/final-parameters.mov](./videos/final-parameters.mov).
