# Goals (essential)
- [x] Testing Monocular RGB Slam protocols
- [x] Writing PID controller to allow the drone to fly to any position in
Optitrack
- [x] Fine tuning PID parameters
- [x] Rewrite PID controller in C++
- [ ] Integrating with ein
- [ ] Run through ein tutorial with drone
- [ ] Generating SLUGS using the drone

# SLAM Goals
- [ ] Write a program that takes a picture (map), chops it into small parts,
and annotates poses (and optionally adds noise)
- [ ] Write a program that takes the above as input and reassembles the map as
a simulation of SLAM

# To Do Sometime (slightly less essential)
- [x] Override takeoff and landing sequences
- [ ] Fabricate custom hull
- [x] Determine how ardrone_autonomy flies the drone (what determines
linear/angular velocity? etc)
- [ ] Recalculate pid controller tests

# Reading
- [x] [SLAM Paper](http://people.eecs.berkeley.edu/~pabbeel/cs287-fa09/readings/Durrant-Whyte_Bailey_SLAM-tutorial-I.pdf)
- [ ] RGB Monocular SLAM papers
- [x] Lightfield paper (In email from Tellex)
- [x] [Applied AI Course Unit 2, uncertainty] (https://edge.edx.org/courses/course-v1:Brown+CSCI1410+2015/2ce1704939854c1894a1ef0a02b23d36/)
- [ ] Probabilistic Robotics textbook (In process)
