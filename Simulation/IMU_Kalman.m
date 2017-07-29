function [ o_current_angle ] = IMU_Kalman( i_sensors, i_actuators, i_previous_state, ...
                                           i_dynamics_system, i_dynamics_actuators, i_set_matrices )

    %% Combine sensor readings
    %  i_sensors    =   [ gyro_angle, gyro_angle_diff, accel_angle, accel_angle_diff ]

    %% Kalman filter matrices
    
    if (i_set_matrices)
        KALMAN_MATRICES(i_dynamics_system, i_dynamics_actuators);
    end
    
    global A;   global B;   global H;   global Q;   global P;   global R;
    Un = i_actuators;
    Xn = [i_previous_state; 0; 0];
    Zn = i_sensors;
    Pp = P;
    
    %% Apply Kalman filter
    
    [ Xn, Pn, A, B, H, Q, R ] = Kalman_filter( Un, Zn, Xn, Pp, A, B, H, Q, R );
    
    o_current_angle = Xn;


end

function [ o_state_space_matrix ] = TF2SS( i_transfer_function )

    [Numerator, Denominator] = tfdata(i_transfer_function);
    Numerator_vector = cell2mat(Numerator(1));
    Denominator_vector = cell2mat(Denominator(1));
    o_state_space_matrix = tf2ss(Numerator_vector, Denominator_vector);

end

function [] = KALMAN_MATRICES ( i_dynamics_system, i_dynamics_actuators )

    % State transition matrix
    A = i_dynamics_system;
    global A;
    
    % Control matrix
    B = i_dynamics_actuators;
    global B;

    % Observation matrix
    H = [
        15   0   0   0
        0   15   0   0
        15   0   0   0
        0   15   0   0
    ];
    global H;
    
    % Estimated process error covariance
    Q = [
        0   0   0   0
        0   0   0   0
        0   0   0   0
        0   0   0   0
    ];
    global Q;

    % Estimated measurement error covariance
    R = [
        0.1 0   0   0
        0   0.1 0   0
        0   0   0.1 0
        0   0   0   0.1 
    ];
    global R;

    % Initial covariance
    P = [
        1   0   0   0
        0   1   0   0
        0   0   1   0
        0   0   0   1
    ];
    global P;

end