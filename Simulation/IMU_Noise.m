function [ o_signal, o_current_state ] = IMU_Noise( i_signal, i_cb, i_rw, i_fn, i_previous_state )
    
    % Constant Bias
    Constant_bias = i_cb;
    o_current_state(1) = Constant_bias + i_previous_state(1);
    o_signal = i_signal + i_cb;

    % Random Walk
    Random_walk_step = i_rw;
    Random_walk_current_location = i_previous_state(2);
    Random_walk_current_location = Random_walk_current_location + ...
                                   ((rand(1) > 0.5)*2 - 1) * Random_walk_step * 2 / 8 + ...
                                   (o_signal < 0) * Random_walk_step * 6 / 8;
    o_signal = o_signal + Random_walk_current_location;
    o_current_state(2) = Random_walk_current_location;
    
    % Flicker noise
    Flicker_noise_power = i_fn;
    o_signal = o_signal + wgn(1, 1, Flicker_noise_power)/10;

end

