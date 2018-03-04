# Kidnapped Vehicle Project

## The Code
The modifed code is in the src directory.

# Implementing the Particle Filter
The directory structure of this repository is as follows:


## Success Criteria

1. **Accuracy**: your particle filter should localize vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

- Here is a snapshot of the particle filter results with 10 partices. It has an error of for x of for y an error of 0.163m, and for yaw an error of 0.005 radians.

![N10](./N10_release.PNG)

2. **Performance**: your particle filter should complete execution within the time of 100 seconds.

- As shown in the above screenshot, the simulation was completed in under 50s.
