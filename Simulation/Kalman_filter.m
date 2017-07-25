function [ o_state_estimate, o_error_estimate ] = Kalman_filter( i_control, i_measurement, ...
           i_state_previous, i_error_previous, i_transfer_function_vehicle, i_transfer_function_actuator )

    %% Input definition
    % i_control = [wheel_angle_difference]
    % i_measurement = [gyroscope_angle, acelerometer_angle]
 
    %% Constants
    
    % Coverting transfer function of vehicle dynamics to state space
    State_transition_matrix = Kalman_filter_tf2ss(i_transfer_function_vehicle);

    % Coverting transfer function of actuator dynamics to state space
    Control_matrix = Kalman_filter_tf2ss(i_transfer_function_actuator);

    % Complementary filter for both sensors
    Observation_matrix = [
        0.25 0.25 0.25 0.25 ;
        0    0    0    0    ;
        0    0    0    0    ;
        0    0    0    0    ;
    ];

    Estimated_process_error_covariance = [
        0   0   0   0   ;
        0   0   0   0   ;
        0   0   0   0   ;
        0   0   0   0   ;
    ];

    Estimated_measurement_error_covariance = [
        0.1 0   0   0   ;
        0   0.1 0   0   ;
        0   0   0.1 0   ;
        0   0   0   0.1 ;
    ];
    
    %% Prediction
    
    Predicted_state_estimate = State_transition_matrix * i_measurement + Control_matrix * i_control;
    
    %Predicted_probability_estimate = (State_transition_matrix * i_state_previous).' * ...
    Predicted_probability_estimate = (State_transition_matrix * i_state_previous).' * ...
                                     State_transition_matrix.' + Estimated_process_error_covariance;
                                 
    %% Innovation
    
    size(Observation_matrix)
    size(Predicted_state_estimate)
    Innovation = i_measurement - Observation_matrix * Predicted_state_estimate;
    
    Innovation_covariance = Observation_matrix * Predicted_probability_estimate * ...
                            Observation_matrix.' + Estimated_measurement_error_covariance;
                         
    %% Update
    
    Kalman_gain = Predicted_probability_estimate * Observation_matrix.' * inv(Innovation_covariance);
    
    o_state_estimate = Predicted_state_estimate + Kalman_gain * Innovation;
    
    o_error_estimate = (eye(size(i_error_previous, 1)) * Kalman_gain * Observation_matrix) * o_state_estimate;

end

