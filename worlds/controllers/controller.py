from controller import Robot

TIME_STEP = 32
MAX_VELOCITY = 26

# Initialize robot
robot = Robot()

# Get motor devices
motors = {
    "fl": robot.getDevice("fl_wheel_joint"),
    "fr": robot.getDevice("fr_wheel_joint"),
    "rl": robot.getDevice("rl_wheel_joint"),
    "rr": robot.getDevice("rr_wheel_joint")
}

# Set motors to speed control mode
for motor in motors.values():
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Get position sensors and enable them
position_sensors = [robot.getDevice(f"{name} wheel motor sensor") for name in ["front left", "front right", "rear left", "rear right"]]
for sensor in position_sensors:
    sensor.enable(TIME_STEP)

# Get camera and enable
camera_rgb = robot.getDevice("camera rgb")
camera_depth = robot.getDevice("camera depth")
camera_rgb.enable(TIME_STEP)
camera_depth.enable(TIME_STEP)

# Get LiDAR and enable
lidar = robot.getDevice("laser")
lidar.enable(TIME_STEP)
lidar.enablePointCloud(True)

# Get IMU sensors and enable them
accelerometer = robot.getDevice("imu accelerometer")
gyro = robot.getDevice("imu gyro")
compass = robot.getDevice("imu compass")
accelerometer.enable(TIME_STEP)
gyro.enable(TIME_STEP)
compass.enable(TIME_STEP)

# Get distance sensors and enable them
distance_sensors = [robot.getDevice(f"{name}_range") for name in ["fl", "rl", "fr", "rr"]]
for sensor in distance_sensors:
    sensor.enable(TIME_STEP)

# Avoidance coefficients
coefficients = [[15.0, -9.0], [-15.0, 9.0]]
base_speed = 6.0

# Main loop
while robot.step(TIME_STEP) != -1:
    # Get accelerometer values
    a = accelerometer.getValues()
    print(f"Accelerometer values: {a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}")

    # Get distance sensor values
    distance_values = [sensor.getValue() for sensor in distance_sensors]

    # Compute motor speeds
    avoidance_speed = [0.0, 0.0]
    for i in range(2):
        for j in range(1, 3):
            avoidance_speed[i] += (2.0 - distance_values[j]) ** 2 * coefficients[i][j - 1]
    
    motor_speeds = [base_speed + avoidance_speed[i] for i in range(2)]
    motor_speeds = [min(MAX_VELOCITY, speed) for speed in motor_speeds]

    # Set motor speeds
    motors["fl"].setVelocity(motor_speeds[0])
    motors["fr"].setVelocity(motor_speeds[1])
    motors["rl"].setVelocity(motor_speeds[0])
    motors["rr"].setVelocity(motor_speeds[1])
