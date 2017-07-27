run Dinamika_robota.m;  %Dobijam prenosnu funkciju robota
run Dinamika_motora.m;  %Dobijam prenosnu funkciju motora
Hs = Hm * Hr;           %Dobijam prenosnu funckiju sistema

Fs = 1/2000;             %Frekvencija odabiranja
Zs = c2d(Hs,Fs,'tustinc');

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
Tu = 3000;

pugao = 0;
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
stanje = [0 0]; pstanje = stanje;

for t = 6:Tu
e(t) = target - y(t-1);

x(t) = x(t-1)+Kp*((1+(dt/Ti)+(Td/dt))*e(t)+(-1-(2*Td/dt))*e(t-1)+(Td/dt)*e(t-2));

y(t) = a1*x(t)+a2*x(t-1)+a3*x(t-2)+a4*x(t-3)+a5*x(t-4)+a6*x(t-5)-b1*y(t-1)-b2*y(t-2)-b3*y(t-3)-b4*y(t-4)-b5*y(t-5);
z(t) = c1*x(t)+c2*x(t-1)+c3*x(t-2)+c4*x(t-3)-d1*z(t-1)-d2*z(t-2)-d3*z(t-3)-d4*z(t-4);

theta(t) = y(t);
fi(t) = z(t);
dtheta(t) = theta(t) - theta(t-1);
dfi(t) = fi(t) - fi(t-1);

[stanje xx] = Sensors(theta(t),dtheta(t),fi(t),dfi(t),xx,pstanje); 
ugao = IMU_Kalman([stanje(1) pstanje(1)-stanje(1) stanje(2) pstanje(2)-stanje(2)],x(t),pugao,dinamika_robota,-dinamika_motora,t==6);

Robot_angle(t) = ugao(1,1);

if(theta(t)> pi) theta(t) = pi;  end;    
if(theta(t)<-pi) theta(t) = -pi; end;
%if(Robot_angle(t)> pi) Robot_angle(t) = pi; end;    
%if(Robot_angle(t)<-pi) Robot_angle(t) = -pi; end;

pstanje = stanje;
pugao = ugao;
end;

vreme = linspace(0,3,3000);
hold on;
plot(vreme,theta);
plot(vreme,Robot_angle);