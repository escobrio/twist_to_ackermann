# twist_to_ackermann

A ROS 2 Humble package that converts velocity commands from differential drive to Ackermann steering, enabling differential drive velocity commands to control Ackermann drive vehicles like the Hunter SE robot.

Specifically, the package provides a node that subscribes and transforms differential drive velocity commands (`/cmd_vel_differential_drive`) and publishes Ackermann steering commands (`/cmd_vel`). 

## Demo Videos

### Driving in a Circle
This video demonstrates the robot executing a circular trajectory using the converted Ackermann steering commands.

![Demo](videos/drive_circle.gif)

### Pure Rotation Handling
This video demonstrates how pure rotation is handled, which is infeasible for an Ackermann drive robot. The wheels turn to the maximum steering angle in the intended direction while the vehicle remains stationary.

![Demo](videos/pure_rotation_handling.gif)

## Design Approach

### Conversion Formula

The steering angle is calculated with the equation:

![steering angle equation](https://latex.codecogs.com/svg.image?\delta=\arctan(\frac{L\omega}{v}))

Where:
- δ = steering angle (rad)
- L = wheelbase length (0.55 m for Hunter SE)
- ω = angular velocity (rad/s)
- v = linear velocity (m/s)

### Pure Rotation Handling

Ackermann drive vehicles cannot perform pure rotation in place (linear velocity = 0, angular velocity ≠ 0) because:
1. The formula would involve division by zero
2. Ackermann vehicles are nonholonomic, thus require forward/backward maneuvers to turn.

**Solution:** When a pure rotation command is detected (linear velocity < 0.001 m/s):
- Linear velocity is set to 0 (vehicle stops)
- Steering angle is set to maximum (±0.69 rad) in the intended direction
- This provides a visual indicator of the intended rotation direction
- Safer than attempting complex maneuvers
- Makes it clear to the operator that linear velocity is required for rotation

The calculated steering angle is clamped to the maximum steering angle limit to prevent invalid commands.

## systemd Service
```bash
./systemd/service_setup.sh install
```

This installs a systemd service on the host that launches the node automatically inside the Docker container, assuming the Docker container and ROS package are properly installed
