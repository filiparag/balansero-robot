Fs = 1/2000;
Zs = c2d(Tf_system,Fs,'tustinc');

[num,den] = tfdata(Zs); 
  
num1=cell2mat(num(1));  
den1=cell2mat(den(1));
num2=cell2mat(num(2));  
den2=cell2mat(den(2));

a1 = num1(1); a2 = num1(2); a3 = num1(3); a4 = num1(4); a5 = num1(5); a6 = num1(6);
b1 = den1(2); b2 = den1(3); b3 = den1(4); b4 = den1(5); b5 = den1(6);
c1 = num2(1); c2 = num2(2); c3 = num2(3); c4 = num2(4); c5 = num2(5); 
d1 = den2(2); d2 = den2(3); d3 = den2(4); d4 = den2(5); 

theta0 = pi/4;
Tu = 50;

pugao = 0.8;
ppugao = 0;
pugao_gyro = 0;
pugao_accel = 0;
Robot_angle = zeros(Tu,1)+theta0;
x(1:100) = 0;
x(51:Tu) = 10;
y(1:Tu) = theta0;
z(1:Tu) = 0;
e(1:Tu) = 0;
theta = y;
fi = z;
xx = [0 0; 0 0];

Kp=9000;
Kd=20;
Ki=1800000;

Ti = Kp/Ki; Td = Kd/Kp;

dt=1/2000;
target = pi/2;