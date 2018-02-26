% problem 1

A=[0 3.0999;0 -5.27]
B=[0;68.0346]
kp=5.139
k=0.1842
Ts=0.01
na=1/9
y=[1;0]
C=[1 0]

[P,S]=eig(A)

% descretizes
Ahat=expm(A.*Ts)
Bhat=P*[Ts 0;0 1/S(2,2)*(exp(S(2,2)*Ts)-1)]*inv(P)*B 

% check if discrete time model is still marginally stable
eig(Ahat)

% controllability matrix
M=[Bhat Ahat*Bhat]

% check controllability
rank(M)

% 1d
% set sampling time
ts=0.3
zmax=exp((-4*Ts)/ts)

% choose target eigen values
z1=0.71
z2=0.70

poly(Ahat)
Apv=[0 1; -ans(3) -ans(2)]
Bpv=[0;1]


Tpv=[Bhat Ahat*Bhat]*inv([Bpv Apv*Bpv]) 
 
poly([z1 z2])

khat=[ans(3)+Apv(2,1) ans(2)+Apv(2,2)]*inv(Tpv) 

display('-----------------------------------')

figure(1)
u=5*ones(1,101);
t=[0:Ts:1];
[y,x]=dlsim(Ahat-Bhat*khat,Bhat,C,0,u);
plot(t,x)
grid;xlabel('time (sec)');ylabel('Vout and Vg')
title('First attempt at Regulator Design: z1=.71 z2=.70')
[Phat,Shat]=eig(Ahat-Bhat*khat);
xdecoupled=inv(Phat)*x';

figure(2)
plot(t,xdecoupled,t,.98*xdecoupled(1,101)*ones(1,101),'--')
grid;xlabel('time (sec)');ylabel('Decoupled modes')
title('First attempt at Regulator Design: Decoupled modes') 



% now switch up target eigenvalues
z1=.76
z2=.75
poly([z1 z2])
khat=[ans(3)+Apv(2,1) ans(2)+Apv(2,2)]*inv(Tpv)
[y,x]=dlsim(Ahat-Bhat*khat,Bhat,C,0,u);
figure(3)
plot(t,x)
grid;xlabel('time (sec)');ylabel('Vout and Vg')
title('First attempt at Regulator Design: z1=.76 z2=.75')

figure(4)
plot(t,x)
grid;xlabel('time (sec)');ylabel('Vout and Vg')
title('Second attempt at Regulator Design: z1=.76 z2=.75')
[Phat,Shat]=eig(Ahat-Bhat*khat);
xdecoupled=inv(Phat)*x';

figure(5)
plot(t,xdecoupled,t,.98*xdecoupled(1,101)*ones(1,101),'--')
grid;xlabel('time (sec)');ylabel('Decoupled modes')
title('Second attempt at Regulator Design: Decoupled modes') 
