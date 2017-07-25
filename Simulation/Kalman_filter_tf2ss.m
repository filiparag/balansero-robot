function [ o_state_space_matrix ] = Kalman_filter_tf2ss( i_transfer_function )

    [Numerator, Denominator] = tfdata(i_transfer_function);
    Numerator_vector = cell2mat(Numerator(1));
    Denominator_vector = cell2mat(Denominator(1));
    o_state_space_matrix = tf2ss(Numerator_vector, Denominator_vector);

end

