run Dinamika_robota.m;
run Dinamika_motora.m;
Hs = Hm * Hr;

dT=0.001;

theta = pi/4; 
dtheta = 0;
fi = pi/2;
dfi = 0;

pvector = [[0 0]; [0 0]];
pV=0;
V=0;

br=0; gain = 200000;
for T=0:dT:1
br=br+1;

[V, pvector] = Sensors (theta, dtheta, fi, dfi, pvector, pV);

t = [0 dT];
u = ones(size(t))*((V-pi/2-pV)*gain);
out = lsim(Hs,u,t,[theta fi]);


theta = out(2,1);
fi = out(2,2);
dtheta = out(1,1)-out(2,1);
dfi = out(1,2) - out(2,2);

ugao(br) = theta;

pV=V;
end;