function [ o_current_angle ] = Complementary_Filter( i_angular_velocity, i_linear_acceleration, o_previous_angle )
    
    Coeffitient = 0.5;
    Delta_time = 0.1;
    
    o_current_angle = Coeffitient * (o_previous_angle + i_angular_velocity * Delta_time) + ...
                     (1 - Coeffitient) * i_linear_acceleration;

end

