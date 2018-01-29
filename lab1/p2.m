% ee572 lab1 p2

nyq=16;
N=128;

% part i
fs = 4*nyq;
t=[0:N-1]*1/fs;
y=10*sin(16*t);
figure(1)
plot(t,y)
title('f_s=4\cdot f_{nyquist}')
figure(2)
fax=[0:N-1]*fs/N;
yf=imag(fftshift(fft(y,N)));
plot(fax, yf)
title('freq domain, f_s=4\cdot f_{nyquist}')
xlabel('Hz')


% part ii
fs = 2*nyq;
t=[0:N-1]*1/fs;
y=10*sin(16*t);
figure(3)
plot(t,y)
title('f_s=4\cdot f_{nyquist}')
figure(4)
fax=[0:N-1]*fs/N;
yf=imag(fftshift(fft(y,N)));
plot(fax, yf)
title('freq domain, f_s=4\cdot f_{nyquist}')
xlabel('Hz')


% part iii
fs = nyq;
t=[0:N-1]*1/fs;
y=10*sin(16*t);
figure(5)
plot(t,y)
title('f_s=4\cdot f_{nyquist}')
figure(6)
fax=[0:N-1]*fs/N;
yf=imag(fftshift(fft(y,N)));
plot(fax, yf)
title('freq domain, f_s=4\cdot f_{nyquist}')
xlabel('Hz')



% part iv
fs = (3/4)*nyq;
t=[0:N-1]*1/fs;
y=10*sin(16*t);
figure(7)
plot(t,y)
title('f_s=4\cdot f_{nyquist}')
figure(8)
fax=[0:N-1]*fs/N;
yf=imag(fftshift(fft(y,N)));
plot(fax, yf)
title('freq domain, f_s=4\cdot f_{nyquist}')
xlabel('Hz')



D=20;

N=12;
MSE_12bit=D^2/(12*2^(2*N))

N=6;
MSE_6bit=D^2/(12*2^(2*N))

N=2;
MSE_2bit=D^2/(12*2^(2*N))
