 ke=10;  % Koeficijent indukcije meh-elk
 km=50;  % Koeficijent indukcije elk-meh
 R=2;    % Otpor
 M=0.3;  % Ukupni moment sila
 L=0;    % Induktivnost
 Fv=0;   % Viskozne sile
 
 Iw=389.6*10^(-9)*2; %Moment inercije tocka

 s=tf('s');
 Hm=ke/((L*s+R)*(M*s+Fv)+km*ke);
 Hm=Iw*Hm;
 stepplot(Hm);