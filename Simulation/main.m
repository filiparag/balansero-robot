%[pvector V] = senzor (pvector theta dtheta fi dfi);
run Dinamika_robota.m;
run Dinamika_motora.m;
Hs = Hm * Hr;

Tfinal = 0.001;
dt=0.001;
t = 0:dt:Tfinal;
u = ones(size(t))*5;
out = lsim(Hs,u,t);

theta = out(:,1);
dtheta = diff(theta);
fi = out(:,2);
dfi = diff(fi);
theta = theta(1:Tfinal/dt,1);
fi = fi(1:Tfinal/dt,1);

pvector = [theta dtheta fi dfi];