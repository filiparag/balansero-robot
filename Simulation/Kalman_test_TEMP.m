clear all;
clc;

run Dinamika_robota.m;
run Dinamika_motora.m;

Target_angle = 90;
Iterations = 100;

True_angle = @ (x) Target_angle + cos( x / (Iterations / 10) );
Measured_angle = @ (x, n) True_angle(x) + (-1 + 2 * rand(1, size(x, 2))) * n;
True_actuator = @ (x) cos( x / (Iterations / 10) + pi);

Gyro_angle = Measured_angle(1:Iterations, 10);
Gyro_angle_diff = [0 diff(Gyro_angle)];
Accel_angle = Measured_angle(1:Iterations, 15);
Accel_angle_diff = [0 diff(Accel_angle)];

Kalman_estimate = zeros(4, Iterations);

for It = 1:Iterations
   
    Kalman_value = IMU_Kalman( [Gyro_angle(It), Gyro_angle_diff(It), Accel_angle(It), Accel_angle_diff(It)], ...
                               True_actuator(It), Target_angle, dinamika_robota, -dinamika_motora, It == 1 );
                              
    Kalman_estimate(:, It) = Kalman_value(:, 1);
    
end

hold on;

plot(True_angle(1:Iterations));
plot(Measured_angle(1:Iterations, 10), '--');

plot(Kalman_estimate(1,:), 'LineWidth', 2);

hold off;
