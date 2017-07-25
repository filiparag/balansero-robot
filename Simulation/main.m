run Dinamika_robota.m;  %Dobijam prenosnu funkciju robota
run Dinamika_motora.m;  %Dobijam prenosnu funkciju motora
Hs = Hm * Hr;           %Dobijam prenosnu funckiju sistema

w0 = 7.46;
Fs = w0/(2*pi);              %Frekvencija odabiranja
Fs = 10^-3;

Zs = c2d(Hs,1/1000);

dt = 0.001;
T = 1;

[num,den] = tfdata(Zs); 
  
num1=cell2mat(num(1));  
den1=cell2mat(den(1));
num2=cell2mat(num(2));  
den2=cell2mat(den(2));

a1 = num1(2); a2 = num1(3); a3 = num1(4); a4 = num1(5); a5 = num1(6);
b1 = den1(2); b2 = den1(3); b3 = den1(4); b4 = den1(5); b5 = den1(6);
c1 = num2(2); c2 = num2(3); c3 = num2(4); c4 = num2(5); 
d1 = den2(2); d2 = den2(3); d3 = den2(4); d4 = den2(5); 

theta0 = pi/2;

pugao = 0;
Robot_angle = zeros(T/dt,1)+theta0;
x(1:100) = 0;
x(51:1000) = 10;
y(1:1000) = theta0;
z(1:1000) = 0;
theta = y;
fi = z;
xx = [0 0; 0 0];

Kp=200;
Kd=0;
Ki=0;

error = 0; ierror = 0; derror = 0; greska = 0;
Ti = Kp/Ki; Td = Kd/Kp;

for t = 6:1000
    
x(t) = x(t-1)+Kp*((1+(dt/Ti)+(Td/dt))*y(t)+(-1-(2*Td/dt))*y(t-1)+(Td/dt)*y(t-2));
   
y(t) = a1*x(t-1)+a2*x(t-2)+a3*x(t-3)+a4*x(t-4)+a5*x(t-5)-b1*y(t-1)-b2*y(t-2)-b3*y(t-3)-b4*y(t-4)-b5*y(t-5);
z(t) = c1*x(t-1)+c2*x(t-2)+c3*x(t-3)+c4*x(t-4)-d1*z(t-1)-d2*z(t-2)-d3*z(t-3)-d4*z(t-4);
theta(t) = y(t);

fi(t) = z(t);
dtheta(t) = theta(t) - theta(t-1);
dfi(t) = fi(t) - fi(t-1);

[ugao xx] = Sensors(theta(t),dtheta(t),fi(t),dfi(t),xx,pugao);

Robot_angle(t) = ugao;

%if(theta(t)> pi) theta(t) = pi;  end;    
%if(theta(t)<-pi) theta(t) = -pi; end;
if(Robot_angle(t)> pi) Robot_angle(t) = pi; end;    
if(Robot_angle(t)<-pi) Robot_angle(t) = -pi; end;

pugao = ugao;
end;

hold on;
plot(theta);
%plot(Robot_angle);

%run Controler.m;