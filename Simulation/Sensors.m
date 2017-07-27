function [ o_angle,  Gyroscope_angle, Accelerometer_angle, o_current_states ] = ...
         Sensors( i_lean, i_lean_d1, i_wheel_rotation, i_wheel_rotation_d, ...
         i_previous_states, i_previous_angle,i_dprevious_angle, Voltage_input,Gyroscope_pangle,Accelerometer_pangle,System_dinamics, Motor_dinamics, first_call)

    Gravity = 9.8;
    Left_margin_gyroskop = -2000;
    Right_margin_gyroskop = 2000;
    Left_margin_accelerometer = -20*Gravity;
    Right_margin_accelerometer = 20*Gravity;

    % Calculate ideal values
    Angular_velocity = i_lean_d1;
    Linear_acceleration = Gravity * cos(i_lean);
    
    o_current_states = i_previous_states;
    
    % Add noise
    [Angular_velocity, o_current_states(1, :)] = IMU_Noise(Angular_velocity, 0.1, 0.05, 0.01, i_previous_states(1, :));
    [Linear_acceleration, o_current_states(2, :)] = IMU_Noise(Linear_acceleration, 0, 0.001, 0.02, i_previous_states(2, :));

    % Siganl integration
     Gyroscope_angle = i_previous_angle + Angular_velocity;
 
     if(Linear_acceleration/Gravity<-1) Accelerometer_angle = 0;
     elseif(Linear_acceleration/Gravity>1) Accelerometer_angle = 0; else
     Accelerometer_angle = acos(Linear_acceleration/Gravity); end;
     if(sign(i_lean)~=sign(Accelerometer_angle)) Accelerometer_angle = Accelerometer_angle * sign(i_lean); end;
    
    if(Gyroscope_angle>Right_margin_gyroskop) Gyroscope_angle = Right_margin_gyroskop; end;
    if(Gyroscope_angle<Left_margin_gyroskop) Gyroscope_angle = Left_margin_gyroskop; end;
    if(Accelerometer_angle>Right_margin_accelerometer) Accelerometer_angle = Right_margin_accelerometer; end;
    if(Accelerometer_angle<Left_margin_accelerometer) Accelerometer_angle = Left_margin_accelerometer; end;
     
     % Get angle using complementary filter
     %o_angle = Complementary_Filter( Gyroscope_angle,Accelerometer_angle, i_previous_angle);
     o_angle = IMU_Kalman([Gyroscope_angle;Gyroscope_pangle;Accelerometer_angle;Accelerometer_pangle],Voltage_input,[i_previous_angle;i_dprevious_angle;],System_dinamics, Motor_dinamics, first_call);
end