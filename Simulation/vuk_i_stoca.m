clear all
close all;
clc;

run Dinamika_robota.m;  %Dobijam prenosnu funkciju robota
run Dinamika_motora.m;  %Dobijam prenosnu funkciju motora
Hs = Hm * Hr;           %Dobijam prenosnu funckiju sistema

[num,den] = tfdata(Hs); 
  
num1=cell2mat(num(1));  
den1=cell2mat(den(1));
num2=cell2mat(num(2));  
den2=cell2mat(den(2));

[A B C D] = tf2ss(num1,den1);

xweight=C'*C*100;
uweight=1; 
K=-lqr(A,B,xweight,uweight);
sys1_lqr=ss(A+B*K,B,C,D); 
%stepplot(-sys1_lqr);

[a b] = ss2tf(sys1_lqr.A,sys1_lqr.B,sys1_lqr.C,sys1_lqr.D);

Hg = 1/((b(1)*s^5+b(2)*s^4+b(3)*s^3+b(4)*s^2+b(5)*s+b(6))/(a(1)*s^5+a(2)*s^4+a(3)*s^3+a(4)*s^2+a(5)*s+a(6)));

Fs = 1/2000;             %Frekvencija odabiranja
Zg = c2d(Hg,Fs,'tustinc');

[num,den] = tfdata(Zg); 
  
num1=cell2mat(num(1));  
den1=cell2mat(den(1));

a1 = num1(1); a2 = num1(2); a3 = num1(3); a4 = num1(4); a5 = num1(5); a6 = num1(6);
b1 = den1(2); b2 = den1(3); b3 = den1(4); b4 = den1(5); b5 = den1(6);
c1 = num2(1); c2 = num2(2); c3 = num2(3); c4 = num2(4); c5 = num2(5); 
d1 = den2(2); d2 = den2(3); d3 = den2(4); d4 = den2(5); 

theta0 = pi/4;
reference = pi/2;

Tu = 10000;
x = ones(Tu,1)*10000;
y = zeros(Tu,1)+theta0;
z = y;
theta = zeros(Tu,1);
dtheta = theta;
fi = theta;
dfi = fi;
xx = [0 0; 0 0];
pugao = 0;
Robot_angle = y;

for t = 6:Tu
x(t) = reference-y(t-1);
y(t) = (a1*x(t)+a2*x(t-1)+a3*x(t-2)+a4*x(t-3)+a5*x(t-4)+a6*x(t-5)-b1*y(t-1)-b2*y(t-2)-b3*y(t-3)-b4*y(t-4)-b5*y(t-5));
z(t) = c1*x(t)+c2*x(t-1)+c3*x(t-2)+c4*x(t-3)-d1*z(t-1)-d2*z(t-2)-d3*z(t-3)-d4*z(t-4);

theta(t) = y(t)/20;
fi(t) = z(t)/20;
dtheta(t) = (theta(t) - theta(t-1))/20;
dfi(t) = (fi(t) - fi(t-1))/20;

[ugao ugao_gyro ugao_accel xx] = Sensors(theta(t),dtheta(t),fi(t),dfi(t),xx,pugao,ppugao - pugao,x(t),pugao_gyro,pugao_accel,E,F,t==6); 

Robot_angle(t) = theta(t)+rand/10;

if(theta(t)> 20*pi) theta(t) = 20*pi;  end;    
if(theta(t)<-20*pi) theta(t) = -20*pi; end;
if(Robot_angle(t)> pi) Robot_angle(t) = pi; end;    
if(Robot_angle(t)<-pi) Robot_angle(t) = -pi; end;

ppugao = pugao(1);
pugao = ugao(1);
pugao_gyro = ugao_gyro;
pugao_accel = ugao_accel;
end;
y = y(1001:Tu);
y = y/18;
Robot_angle = Robot_angle(1001:Tu);
hold on;
plot(y+pi/2+0.1);
plot(Robot_angle+pi/2+0.1);