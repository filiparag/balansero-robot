function [ o_steps ] = Encoder_Noise( i_steps, i_noise_treshold )

    o_steps = i_steps +  ((rand(1) > 0.5)*2 - 1) * (rand(1) * i_noise_treshold) / i_noise_treshold;

end

