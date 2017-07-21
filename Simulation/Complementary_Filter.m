function [ o_current_angle ] = Complementary_Filter( i_angular_velocity, i_linear_acceleration, i_rotation, o_previous_angle )

    [Rotation_angle_steps, Rotary_encoder_steps] = i_rotation;
    
    Coeffitient = 0.5;
    Delta_time = 0.1;
    
    o_current_angle = Coeffitient * (o_previous_angle + i_angular_velocity * Delta_time) + ...
                     (1 - k) * i_linear_acceleration;
                 
    o_current_angle = o_current_angle + Rotation_angle_steps / Rotation_angle_steps * 360;

end

