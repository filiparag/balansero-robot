clear all;
clc;

%% System dynamics

[ Tf_robot, Tf_actuator, Ss_robot, Ss_actuator ] = System_dynamics;

Tf_system = Tf_actuator * Tf_robot;

%% Unclean

run params.m

%% Simulation

Simulation_steps = 1000;

History = zeros(size(1:Simulation_steps, 1), 4);
% [ theta fi angle angle_accel angle_gyro ]


for t = 6:Simulation_steps
   
    %% Transfer functions
    
    e(t) = target - y(t-1);
    x(t) = x(t-1)+Kp*((1+(dt/Ti)+(Td/dt))*e(t)+(-1-(2*Td/dt))*e(t-1)+(Td/dt)*e(t-2));
    y(t) = a1*x(t)+a2*x(t-1)+a3*x(t-2)+a4*x(t-3)+a5*x(t-4)+a6*x(t-5)-b1*y(t-1)-b2*y(t-2)-b3*y(t-3)-b4*y(t-4)-b5*y(t-5);
    z(t) = c1*x(t)+c2*x(t-1)+c3*x(t-2)+c4*x(t-3)-d1*z(t-1)-d2*z(t-2)-d3*z(t-3)-d4*z(t-4);

    History(t, 1:2) = [y(t) z(t)];
    
    theta(t) = y(t);
    fi(t) = z(t);
    dtheta(t) = theta(t) - y(t-1);
    dfi(t) = fi(t) - fi(t-1);
    
    if(theta(t)> pi) theta(t) = pi;  end;    
    if(theta(t)<-pi) theta(t) = -pi; end;

    %% Sensors
    
    [ugao ugao_gyro ugao_accel xx] = Sensors(theta(t),dtheta(t),fi(t),dfi(t),xx,pugao,ppugao - pugao,x(t),pugao_gyro,pugao_accel,Ss_robot,Ss_actuator,t==6); 

    History(t, 3:5) = [mean(ugao)/20+1.85 ugao_gyro ugao_accel];
    
end


%% Plotting

hold on;

plot(History(:, 1))
plot(History(:, 4:5), '--')
plot(History(:, 3), 'LineWidth', 2)

