function [ o_signal ] = IMU_Noise( i_signal, i_cb, i_rw, i_fn )

    Length = size(i_signal, 2);

    % Constant Bias
    Constant_bias = i_cb;
    o_signal = i_signal + (Constant_bias / Length : Constant_bias / Length : Constant_bias);

    % Random Walk
    Random_walk_step = i_rw;
    Random_walk_current_location = 0;
    for Index = 1 : Length
        Random_walk_current_location = Random_walk_current_location + ...
                                       ((rand(1) > 0.5)*2 - 1) * Random_walk_step * 2 / 8 + ...
                                       (o_signal(1, Index) < 0) * Random_walk_step * 6 / 8;
        o_signal(1, Index) = o_signal(1, Index) + Random_walk_current_location;
    end

    % Flicker noise
    Flicker_noise_power = i_fn;
    o_signal = o_signal + wgn(1, Length, Flicker_noise_power);

end

