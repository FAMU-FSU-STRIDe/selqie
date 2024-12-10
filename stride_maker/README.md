# Stride Maker Notes

## Stride Generation
- File: include/stride_maker/stride_maker.hpp
- Generates trajectory for single leg

- Params:
    - number of points in trajectory
    - stride frequency [Hz]
    - duty factor (0.0 to 1.0)
    - trajectory shift relative to center
        - 0.0: centered with body
        - -1.0: foot lands directy below body
        - +1.0: foot lifts directly below body
        - Recommended: -0.5 (foot spends more time pushing off in direction of travel)
    - stance length [m] (= stride length * duty factor)
    - body height [m]
    - step height [m]

- Stance trajectory:
    - straight line
    - constant z = -body height
    - touchdown x = 1/2 * stance length * (1 + center shift)
    - takeoff x = touchdown x - stance length
    - time in stance = duty factor / frequency

- Swing trajectory:
    - semi ellipse
    - centered about (x = midpoint of touchdown x and takeoff x, z = -body height)
    - radius x = 1/2 stance length
    - radius z = step height
    - time in swing = (1 - duty factor) / frequency

## Walking w/ 2DOF Legs
- File: src/walk2d_node.cpp
- Based on the desired velocity (v) and angular velocity (omega), an arc of radius R = v / omega and the body width of the robot is used to find the velocities of the left and right sides of the robot.
    - left velocity = v - 1/2 * robot width * omega
    - right velocity = v + 1/2 * robot width * omega
- We want to match these velocities using different strides for the left and right foot 
    - velocity = stride length * stride frequency = stance length * stride frequency / duty factor
- For walking, the trajectories of alternate left and right feet must be in stance phase at the same time
    - Therefore, the stride frequency and duty factors of the two trajectories must match
    - This leaves only the stance length to be adjusted
- Using a defined maximum stance length as the stance left of the trajectory for feet on the outside of the turning arc, the stance length of the inner feet is = maximum stance length * (velocity inner / velocity outer)
- The stride frequency is = (velocity outer / max stance length) * duty factor
- Params:
    - robot width [m]
    - duty factor
    - max stance length [m]

## Walking Command Adjustments
- Experimentally, the desired velocity and angular velocity don't match the actual velocities of the robot, but are proportional
    - `ros2 run robot_experiments walk_stride_tracking` to see command vs actual velocities over a sample time
- By sweeping over many variations of commanded linear and angular velocities and plotting the result, its possible to find a mapping between the two
    - `ros2 run robot_experiments walk_stride_sweep` to run sweep
- The commanded to actual velocity mapping was of the form:
    - actual lin vel = (A - B * commanded ang vel) * commanded lin vel
    - actual ang vel = (C - D * commanded lin vel) * commanded ang vel
- Then, inverting the equations gives a mapping from actual to commanded of the form:
    - commanded lin vel = f(actual lin vel, actual ang vel)
    - commanded ang vel = g(actual lin vel, actual ang vel)
    - Finding the functions f and g are difficult, so MATLAB is used to solve them (File: matlab/walk_stride_cmd.m)
- The result is implemented in a `map_des2cmd(des_v, des_omega)` function to get the commanded velocities for our desired velocities
- This mapping approximation is good, but not perfect