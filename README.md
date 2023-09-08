# Animation Keyframe Interpolation



## Environment

```
Windows 11
Visual Studio 2017
```

## Directory Structure

```
└───Mocap-Interpolation
    ├───Assets
    │   ├───AutomationScripts
    │   ├───Builds
    │   ├───Extracted
    │   ├───Graphs
    │   ├───Interpolated
    │   ├───Logs
    │   ├───Original
    │   └───Recording
    │       ├───Input motion and Bezier Euler
    │       ├───Input motion and Bezier SLERP quaternion
    │       └───Input motion and SLERP quaternion
    ├───fltk-1.3.8
    │   ├───...
    ├───IDE-starter
    │   └───...
    └───mocapPlayer-starter
```

AutomationScripts: scripts for automation

Builds: builds of mocapPlayer and interpolator

Extracted: extracted joint rotation value

Graph: required plots

Interpolated: interpolated amc files

Logs: an output log for performance evaluation

Original: original amc and asf files

Recording: image sequences of required recording



## Features Implemented

1. Quaternion/Euler utilities
2. Interpolation
   1. Linear (Lerp) Interpolation (Euler Angles)
   2. Bezier (Lerp) Interpolation (Euler Angles)
   3. Linear (Slerp) Interpolation (Quaternions)
   4. Bezier (Slerp) Interpolation (Quaternions)
3. DeCasteljau Construction for evaluating splines
4. Automation PowerShell and Python scripts (**Extra Credits**)
   1. Generate interpolated amc files
   2. Extract certain joint rotation value
   3. Generate Plots based on rotation value
5. Performance evaluation for different interpolation methods (**Extra Credits**)



## Findings and Analysis

First, I looked through the performance log:

```
Interpolation type is: LINEAR
Angle representation for interpolation is: EULER
Interpolating...
Time taken: 53968 microseconds
```

```
Interpolation type is: BEZIER
Angle representation for interpolation is: EULER
Interpolating...
Time taken: 474007 microseconds
```

```
Interpolation type is: LINEAR
Angle representation for interpolation is: QUATERNION
Interpolating...
Time taken: 323599 microseconds
```

```
Interpolation type is: BEZIER
Angle representation for interpolation is: QUATERNION
Interpolating...
Time taken: 1133724 microseconds
```

Then I looked through graphs and video playback from mocap player.

![Graph 1](./assets/Graph 1.png)

![Graph 2](./assets/Graph 2.png)

![Graph 3](./assets/Graph 3.png)

![Graph 4](./assets/Graph 4.png)



1. Quaternion costs more than Euler representation of joint angles.

2. Bezier Interpolation costs more than Linear Interpolation.

3. Quaternion is generally better interpolating rotations than Euler.

In general:

1. Linear
   1. Fast to compute
   2. Very rigid transition
2. Bezier
   1. Gives much better interpolation, no rigid transition
   2. Takes much more time to compute

3. Quaternion

   1. Takes more time to compute

   2. More stable when interpolate rotation

4. Euler

   1. Computationally better than quaternion

   2. Not very stable when interpolate rotations, might cause strange interpolated value.

Though technically Bezier Slerp Quaternion interpolation can solve most issue, I notice that it still have some restrictions.

For animations like punching (martial arts), it has very special curves between keyframes that gives the impression of impact and strength. Directly interpolate keyframes removes these characteristics. Using constant like 1/3 gives approximate constant speed along spline, but in this case we might want to change it for aesthetic reason. 
