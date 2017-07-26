clear all;
clc;

run Dinamika_robota.m;
run Dinamika_motora.m;

Target_angle = 90;
Iterations = 100;

True_angle = @ (x) Target_angle + 10 * cos( x / (Iterations / 10) );
Measured_angle = @ (x, n) True_angle(x) + (-1 + 2 * rand(1, size(x, 2))) * n;

Gyro_angle = Measured_angle(1:Iterations, 5);
Gyro_angle_diff = [0 diff(Gyro_angle)];
Accel_angle = Measured_angle(1:Iterations, 10);
Accel_angle_diff = [0 diff(Accel_angle)];

Kalman_angle = zeros(Iterations);

for It = 1:Iterations
   
   %Kalman_angle(It)
    oooooooooooooooo = IMU_Kalman( [Gyro_angle(It), Gyro_angle_diff(It), Accel_angle(It), Accel_angle_diff(It)], ...
                                   Target_angle, Hr, Hm, It == 1 );
    
end