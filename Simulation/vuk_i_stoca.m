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

theta0 = pi/4;
reference = pi/2;

Tu = 10000;
x = zeros(Tu,1)*100;
y = zeros(Tu,1)+theta0;

for t = 6:Tu
y(t) = a1*x(t)+a2*x(t-1)+a3*x(t-2)+a4*x(t-3)+a5*x(t-4)+a6*x(t-5)-b1*y(t-1)-b2*y(t-2)-b3*y(t-3)-b4*y(t-4)-b5*y(t-5);

end;
plot(y);