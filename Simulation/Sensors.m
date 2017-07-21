function [ o_angle, o_current_states ] = ...
         Sensors( i_lean, i_lean_d1, i_wheel_rotation, i_wheel_rotation_d, ...
         i_previous_states, i_previous_angle)

    Gravity = 9.8;

    % Calculate ideal values
    Angular_velocity = i_lean_d1;
    Linear_acceleration = Gravity * sin(i_lean);
    
    % Add noise
    [Angular_velocity, o_current_states(1, :)] = IMU_Noise(Angular_velocity, 0.1, 0.05, 0.01, i_previous_states(1, :));
    [Linear_acceleration, o_current_states(2, :)] = IMU_Noise(Linear_acceleration, 0, 0.15, 0.05, i_previous_states(2, :));

    % Get angle using complementary filter
    o_angle = Complementary_Filter(Angular_velocity, Linear_acceleration, i_previous_angle);
    
end