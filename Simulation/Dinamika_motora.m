 ke=1;  % Koeficijent indukcije meh-elk
 km=10;  % Koeficijent indukcije elk-meh
 R=1;    % Otpor
 M=0.01;  % Ukupni moment sila
 L=0;    % Induktivnost
 Fv=0;   % Viskozne sile
 
 Iw=389.6*10^(-9)*2; %Moment inercije tocka

 s=tf('s');
 Hm=ke/((L*s+R)*(M*s+Fv)+km*ke);
 %Hm=Iw*Hm;
 %stepplot(Hm);

 dinamika_motora = 1;