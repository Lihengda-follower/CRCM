Initialize:
    Desired longitudinal distance DY
    Desired lateral distance DX (usually 0)
    PID parameters for speed control: Kp_v, Ki_v, Kd_v
    PID parameters for steering control: Kp_s, Ki_s, Kd_s
    previous_error_v = 0
    integral_v = 0
    previous_error_s = 0
    integral_s = 0
    Control time step dt

Loop every dt seconds:
    Measure relative position of the target in FCR's coordinate frame:
        AX = lateral offset (target's x in FCR frame)
        AY = longitudinal offset (target's y in FCR frame)

    ------------------------------
    Longitudinal PID Control (Speed)
    ------------------------------
    error_v = AY - DY
    integral_v += error_v * dt
    derivative_v = (error_v - previous_error_v) / dt
    output_speed = Kp_v * error_v + Ki_v * integral_v + Kd_v * derivative_v
    previous_error_v = error_v

    Limit output_speed to [min_speed, max_speed]

    ------------------------------
    Lateral PID Control (Steering)
    ------------------------------
    error_s = AX - DX
    integral_s += error_s * dt
    derivative_s = (error_s - previous_error_s) / dt
    output_steering = Kp_s * error_s + Ki_s * integral_s + Kd_s * derivative_s
    previous_error_s = error_s

    Limit output_steering to [min_angle, max_angle]

    ------------------------------
    Send Commands to FCR
    ------------------------------
    Set FCR speed to output_speed
    Set FCR steering angle to output_steering

    If output_speed is very low and AY < DY:
        Apply brake to FCR to stop smoothly

    Wait for next control cycle
