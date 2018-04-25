% Alex Mueller
% Galvin Greene, Ben Cummins, Matt Ruffner
% EE 572 Final Project Simulation

% Purpose: To simulate our 1st order type zero system when
% subject to ramp and step inputs. Also we wish to compute PID parameters
% and discrete time compensator filter parameters
% to form the compensated system for simulation and programing of the system.

close all

syms s

%motor parameters
k = 810; % dc gain (max of 810 rpm)
tau = .4; % time constant ( time to get to .632 of max speed)

Ts = .1; % sampling time

Mp = 4.32; % max percent overshoot
m = Mp/100;

% compute zeta based on given specs
zeta = -log(m)/sqrt((pi^2)+log(m)^2);

ts = 2; % desired settling time (seconds)
wnzeta = 4/ts;
wn = wnzeta/zeta;

% define system and compensator functions
gzoh = 1/(1+(Ts/2)*s);
g = k/(1+tau*s);

% desired dominant closed-loop pole
s1 = -wnzeta + 1i*wn*sqrt(1-(zeta^2));

% find transfer function of the system before compensation
H1a = gzoh*g;
H1b = syms2tf(H1a);

% simulate uncompensated system step response
step(H1b)
title('Step Response of System Before Compensation')
ylabel('RPM')

%generate a ramp input
len =85;
u1 = ones(1,len);
for i = 1:len
    u1(i) = 10*i*u1(i);
end
t = 0:Ts:Ts*len-Ts;
x0 = 0;

% simulate uncompensated system with the ramp input
y = lsim(H1b,u1,t,x0,'zoh');

% plot simulation
figure
plot(t,y,t,u1)
ylim([-.1 2000])
xlim([0 2])
legend('System Response','Ramp Input')
title('Ramp Response of System Before Compensation')
ylabel('RPM')

s = s1;
gzohg = double(abs(subs(g*gzoh/s1)));
ang_def = 180-((180*angle(double(subs(g)*subs(gzoh))/s1)/pi));
half_ang = ang_def/2;
Zc = (imag(s1)/tan(half_ang*pi/180))-real(s1);
Kc = 1/(abs(s1+Zc)^2+gzohg);
Kd = Kc;
Ki = Kc*Zc^2;
Kp = 2*Zc*Kc;

syms s

% symbolic pid function
gpid = (Ki+Kp*s+Kd*s^2)/s;

% find closed-loop transfer function of the compensated system 
H2a = (gzoh*g*gpid)/(1+gzoh*g*gpid);
H2b = syms2tf(H2a);

% simulate compensated system step response
figure
step(H2b)
title('Step Response of System After Compensation')
ylabel('RPM')

y2 = lsim(H2b,u1,t,x0,'zoh');
% plot simulation
figure
plot(t,y2,t,u1)
%ylim([-.1 2000])
%xlim([0 2])
legend('System Response','Ramp Input')
title('Ramp Response of System After Compensation')
ylabel('RPM')


% discretize system
syms z
s = (z-1)/Ts; % use rectanglar estimation holding from left side
H_z = subs(gpid/(1+gpid));
[symNum,symDen] = numden(H_z); %Get num and den of Symbolic TF
TFnum = sym2poly(symNum);    %Convert Symbolic num to polynomial
TFden = sym2poly(symDen);    %Convert Symbolic den to polynomial
H_z_tf =tf(TFnum,TFden,Ts);
[num,den,Ts1] = tfdata(H_z_tf,'v');

% assign filter coefficients
factor = den(1);
num = num/factor;
den = den/factor;

b0 = num(1);
b1 = num(2);
b2 = num(3);
a0 = den(1);
a1 = den(2);
a2 = den(3);