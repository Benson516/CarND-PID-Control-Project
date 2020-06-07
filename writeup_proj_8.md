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

The PID controller used in this project is described in psudo code as below.

```c++
p_error = cte;
i_error += cte;
d_error = cte - previous_cte; 
steer_value = -1 * (Kp*p_error + Ki*i_error + Kd*d_error);
```

Where the `cte` is the error from the car reference point to the path.

---

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

Each parameter covers differenct aspect about stabilizing the system. This section describe the characteristic of each parameter in this problem.

## P-gain (Kp)

The **P-gain** (propotional gain) determine the effect of the `p_error` to the final steering value. It determine the imeidiate response of the error correction.

Compare to the final tuened parameter, the following test demonstrate the variation on `Kp`.


**a. Zero Kp**

Here's the [link to the video.](https://youtu.be/UBVHEr5Cb4c)

[![video - final](http://img.youtube.com/vi/UBVHEr5Cb4c/0.jpg)](https://youtu.be/UBVHEr5Cb4c)



**b. Twice the Kp**


Here's the [link to the video.](https://youtu.be/Zd_zaaDy7SE)

[![video - final](http://img.youtube.com/vi/Zd_zaaDy7SE/0.jpg)](https://youtu.be/Zd_zaaDy7SE)



Both situations resulted in oscillation; however, the reason is different. Since the `Kp` term can act as a `damper` relative to the `Ki` term in the overall closeloop system, lacking `Kp` term let the intergral dynamic being underdamped and resulted in oscillation. On the other hand, too much `Kp` let the overall system been underdamped again, since the `Kd` term, which acts as the damper of the `p_error` term, wan't tuned up as well.

Too much `Kp` also resulted in rough intermittent steering during turn. The solution for this is utilizing the `Ki` term.

## D-gain (Kd)

The **D-gain** (differentail gain) determine the effect of the `d_error` to the final steering value.  

**a. Zero Kd**

Here's the [link to the video.](https://youtu.be/qeMfHjFftTE)

[![video - final](http://img.youtube.com/vi/qeMfHjFftTE/0.jpg)](https://youtu.be/qeMfHjFftTE)



**b. Twice the Kd**


Here's the [link to the video.](https://youtu.be/1hBAHDtH6QY)

[![video - final](http://img.youtube.com/vi/1hBAHDtH6QY/0.jpg)](https://youtu.be/1hBAHDtH6QY)





## I-gain (Ki)

The **I-gain** (integral gain) determine the effect of the `i_error` to the final steering value.  

**a. Zero Ki**

Here's the [link to the video.](https://youtu.be/x1ZoREgAHco)

[![video - final](http://img.youtube.com/vi/x1ZoREgAHco/0.jpg)](https://youtu.be/x1ZoREgAHco)



**b. Twice the Ki**

Here's the [link to the video.](https://youtu.be/36640Dd5fg0)

[![video - final](http://img.youtube.com/vi/36640Dd5fg0/0.jpg)](https://youtu.be/36640Dd5fg0)




# Result

The final parameters are chosen as follow.

|Parameter| Value|
|:---:|:---:|
| Kp  | 0.11|
| Ki  | 0.0019|
| Kd  | 1.2|

Here's the [link to the final result video.](https://youtu.be/ywz0B7GqCwU)

[![video - final](http://img.youtube.com/vi/ywz0B7GqCwU/0.jpg)](https://youtu.be/ywz0B7GqCwU)


