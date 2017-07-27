clear all;
clc;

%Parameters
m=513.3*10^(-3);  
m2=0*10^(-3); 
m1=m-m2; 
mw=7.2*10^(-3); 
L1=40*10^(-3);
L2=60*10^(-3); 
I=m1*(L1/2+L2)^2+m2*L2*L2/12;
Iw=389.6*10^(-9)*2; 
Br=0.01; 
Bm=0.01; 
L=L2/2+(L1+L2)*m1/(2*m);
R=16*10^(-3); 
g=9.8; 

A=[Iw+(mw+m)*R*R m*R*L; m*R*L m*L*L+I]; 
B=[Br+Bm -Bm; -Bm Bm]; 
C=[0; -m*g*L];
D=[1; -1]; 

E = [0 0 1 0; 0 0 0 1; [0; 0] -inv(A)*C -inv(A)*B];
F = [0; 0; inv(A)*D];
G = [R 0 0 0; 0 1 0 0];

sys=ss(E,F,G,0);
Hr=tf(sys);
dinamika_robota = E;
%stepplot(Hr,1);