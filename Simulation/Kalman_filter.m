function [ o_state_estimate, o_error_estimate ] = Kalman_filter( i_control, i_measurement, i_state_previous, i_error_previous )

    State_transition_matrix = [];
    Control_matrix = [];
    Observation_matrix = [];
    Estimated_process_error_covariance = [];
    Estimated_measurement_error_covariance = [];
    
    % Prediction
    Predicted_state_estimate = State_transition_matrix * i_measurement + Control_matrix * i_control;
    Predicted_probability_estimate = (State_transition_matrix * i_state_previous) * ...
                                     State_transition_matrix.' + Estimated_process_error_covariance;
                                 
    % Innovation
    Innovation = i_measurement - Observation_matrix * Predicted_state_estimate;
    Innovation_covariance = Observation_matrix * Predicted_probability_estimate * ...
                            Observation_matrix.' + Estimated_measurement_error_covariance;
                         
    % Update
    Kalman_gain = Predicted_probability_estimate * Observation_matrix.' * inv(Innovation_covariance);
    o_state_estimate = Predicted_state_estimate + Kalman_gain * Innovation;
    o_error_estimate = (eye(size(i_error_previous, 1)) * Kalman_gain * Observation_matrix) * o_state_estimate;

end

