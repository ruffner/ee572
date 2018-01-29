%1c Bilinear Integrator Design
Ts=0.01;
b_bilin=Ts/2*[1 1];
a_bilin=[1 -1];
%Impulse Invariant Design
b_impls=Ts*[1 0];
a_impls=[1 -1];
%Step Invariant Design
b_step=Ts^2*[0 1];
a_step=[1 -1];
%Inputs to the digital integrators
dk=[1 zeros(1,100)];
uk=ones(1,101);
t=[0:Ts:1];
yk=10*sin(16*pi*t);
%Find Integrator Outputs using Filter
y_bilin_dk=filter(b_bilin,a_bilin,dk);
y_bilin_uk=filter(b_bilin,a_bilin,uk);
y_bilin_yk=filter(b_bilin,a_bilin,yk);
y_impls_dk=filter(b_impls,a_impls,dk);
y_impls_uk=filter(b_impls,a_impls,uk);
y_impls_yk=filter(b_impls,a_impls,yk);
y_step_dk=filter(b_step,a_step,dk);
y_step_uk=filter(b_step,a_step,uk);
y_step_yk=filter(b_step,a_step,yk);


figure(1)
plot(t,y_bilin_dk)
title('delta bilinear')

figure(2)
plot(t,y_bilin_uk)
title('step bilinear')

figure(3)
plot(t,y_bilin_yk)
title('sine bilinear')



figure(4)
plot(t,y_impls_dk)
title('delta, impulse invariant design')

figure(5)
plot(t,y_impls_uk)
title('step, impulse invariant design')

figure(6)
plot(t,y_impls_yk)
title('sine, impulse invariant design')



figure(7)
plot(t,y_step_dk)
title('delta, step invariant design')

figure(8)
plot(t,y_step_uk)
title('step, step invariant design')

figure(9)
plot(t,y_step_yk)
title('sine, step invariant design')


