%run Dinamika_robota.m;  %Dobijam prenosnu funkciju robota
%run Dinamika_motora.m;  %Dobijam prenosnu funkciju motora
%Hs = Hm * Hr;           %Dobijam prenosnu funckiju sistema

%load Transfer_function_system.mat;
s = tf('s');
Hs(1) = (257.6*s^2+1.602*10^-12-1.434*10^4)/(0.01*s^5+13.25*s^4+3251*s^3+846.8*s^2-1.793*10^5*s);
Hs(2) = (-1762*s-2.228*10^4)/(0.01*s^4+13.25*s^3+3251*s^2+846.8*s^1-1.793*10^5);

Fs = 2*10^-3;              %Frekvencija odabiranja

[num,den] = tfdata(Hs); %Prebacivanje iz s u z domen
  
num1=cell2mat(num(1));  %Preabacivanje u nizove za theta
den1=cell2mat(den(1));
num2=cell2mat(num(2));  %Prebacivanje u nizove za fi
den2=cell2mat(den(2));

[dnum1,dden1]=bilinear(num1,den1,Fs);  %Bilinear za theta
[dnum2,dden2]=bilinear(num2,den2,Fs);  %Bilinear za fi

Y(1:50) = 0; Y(51:1000) = 10;          %Definisanje ulaznog signala
X=zeros(1000,1);                       %Definisanje izlaznog signala
for i=6:1000                           %Primena formule
X(i) = (Y(i-5)*dnum1(1)+Y(i-4)*dnum1(2)+Y(i-3)*dnum1(3)+Y(i-2)*dnum1(4)+Y(i-1)*dnum1(5)+Y(i)*dnum1(6)...
-X(i-5)*dden1(1)-X(i-4)*dden1(2)-X(i-3)*dden1(3)-X(i-2)*dden1(4)-X(i-1)*dden1(5)-X(i)*dden1(6))/dnum1(1);
end;

dt = 0.001;
T = 1;

x0 = [pi/2 pi/2];
t = 0:dt:T;
u = ones(size(t))*10;
out = lsim(Hs,u,t,x0);

theta = out(:,1);
fi = out(:,2);
dtheta = diff(theta);
dfi = diff(fi);
fi = fi(1:T/dt);
theta = theta(1:T/dt);

x = [pi/2 0; pi/2 0];
pugao = 0;
Robot_angle = zeros(T/dt,1);
br=0;
       
K = 1+Hs;
K(1) = Hs(1)/K(1);
K(2) = Hs(2)/K(2);

for i=1:T/dt
[ugao x] = Sensors(theta(i),dtheta(i),fi(i),dfi(i),x,pugao);
br = br+1;
    if(theta(i)>pi)  theta(i) = pi; end;    
    if(theta(i)<-pi) theta(i) = -pi; end;
    
Robot_angle(br) = ugao;
pugao = ugao;
end;

hold on;
plot(Robot_angle);
plot(theta);