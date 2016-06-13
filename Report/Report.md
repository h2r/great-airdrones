---
title: Exploration and Evaluation of PTAM and LSD-SLAM
date: May 5, 2016
author: Josh Roy, Yury Gitman, Vidur Joshi
---

# Objectives

Our objective in this project was to evaluate the accuracy of the PTAM and
LSD-SLAM algorithms when run on the Parrot AR Drone 2.0 quadcopter. Both PTAM
and LSD-SLAM implement the monocular SLAM (Simultaneous Localization and
Mapping) protocol. In addition, PTAM also utilizes navigation data from the
drone's IMU (Inertial Measurement Unit). Both algorithms output the drone's
predicted position and create a point cloud of the environment that the drone
observes.

## Risks

The risks in this project were not large. In a worst case scenario, we would
crash the drones acquired for this project. There were no other associated
risks with this project.


## Payoffs

Completing this project allowed us to work towards making the drone a better
vehicle for research. Allowing the drone to know its own position will enable
more research to be done on the drone, and the first step is to evaluate the
currently existing localization protocols.

# Our Approach

In order to measure the accuracy of each of the algorithms, we used the
following measurement methods.

## Quantitative

In order to quantitatively measure the accuracy of each of the algorithms, we
measured the actual position of the drone using an OptiTrack motion capture
system that measures the position and orientation of the drone with an accuracy
of $2$ millimeters and the predicted position of each of the algorithms. We ran
this test multiple times on the same dataset in order to manually optimize the
parameters of each of the algorithms. We then compared the two positions and
calculated errors as shown in the results section below.

### Results

See the **[GitHub Wiki Page](https://github.com/h2r/great-ardrones/wiki/)**.


## Qualitative

While analysing the quantitative results allowed us to gain some insight into
the accuracy of the algorithms, we also feel that it is important to also
include qualitative results.

### Results

#### PTAM

PTAM was a difficult algorithm to test due to the fact that it frequently got
lost, especially when moving around corner, even after manual optimization of
the parameters. This makes us feel that PTAM would be difficult to use to
localize the drone in an area with obstacles. In addition, we found that upon
being lost and finding itself, PTAM attemped to recover its position by
accounting for the time that it was lost, however, this often resulted in false
movements. That being said, we found that when translating on a Y-Z plane, PTAM
was able to mostly consistently generate estimated positions, even if they were
inaccurate, as shown in the quantitative results above.

#### LSD-SLAM

LSD-SLAM was able to continue generating position estimates during most of the
movements which lost PTAM, however, it also got lost when the drone moved or
turned too quickly. In addition, LSD-SLAM seemed to be much more
accurate than PTAM, however, its position estimates were also off, as shown in
the quantitative results. Manually optimizing the parameters made LSD-SLAM both
more accurate and much faster, and allowed the algorithm to more consistently
generate position estimates.

# Future Work

There are a few branches that future work could take.

1) Further optimizing the parameters of the algorithms (computationally) and
running tests in order to more thoroughly test the accuracy of each algorithm
2) Building upon and improving LSD-SLAM by integrating the drone's navdata,
etc.
3) Moving the drone using OptiTrack data in order to more consistently generate
maps and measurements using LSD-SLAM and PTAM.
4) Running PTAM and LSD-SLAM on more test data in order to better rate their
accuracy.
