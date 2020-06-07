# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
---


[//]: # (Image References)

[image0]: ./pictures/architecture.png "processing flow"
[image1]: ./pictures/trajectory_generation.png "trajectory generation"
[image2]: ./pictures/speed_scheduling.png "speed scheduling"
[image3]: ./pictures/behavior_planning.png "behavior planning"

# Introduction

The target of this project is to implement and design the parameter of a PID controller for a self-driving car to follow the path. Since the project restricted the controller to a pure PID controller, the main focus of this project is to find a set of parameters that resulted in a smooth drive in the simulator.

# The PID Controller

The PID controller used in this project is described below.

```c++
steer_value = -1 * (Kp*p_error + Ki*i_error + Kd*d_error);
```

The actual implementation are done in the `PID` class (`code line #33 ~ #38 in PID.cpp`)

```c++
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return -1*(Kp*p_error + Ki*i_error + Kd*d_error);  // TODO: Add your total error calc here!
}
```

, and the calculation of each error term (`code line #24 ~ #31 in PID.cpp`)

```c++
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   d_error = cte - p_error; // p_error is equal to the previous cte at current stage
   p_error = cte;
   i_error += cte;
}
```

The methods are called in `main.cpp` at each iteration 
(`code line #71 in main.cpp`)

```c++
pid.UpdateError(cte);
steer_value = pid.TotalError();
```




# Parameters

## P-gain (Kp)

## D-gain (Kd)

## I-gain (Ki)


# Result

The final parameters are chosen as follow.

|Parameter| Value|
|:---:|:---:|
| Kp  | 0.11|
| Ki  | 0.0019|
| Kd  | 1.2|

Here's the [link to the final result video.](https://youtu.be/wIGluTLbiUY)

[![video - final](http://img.youtube.com/vi/wIGluTLbiUY/0.jpg)](https://youtu.be/wIGluTLbiUY)


