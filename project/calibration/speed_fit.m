D=dlmread('speedcal2.tsv','\t');
P=polyfit(D(:,1),D(:,2),3);
figure(1)
plot(D(:,1),D(:,2),D(:,1),polyval(P,D(:,1)))

pwmvals=linspace(81,255,175);
lut=polyval(P,pwmvals);
figure(2)
plot(pwmvals,lut)

fprintf("PROGMEM float RPMMap[175] = {");
for n=1:175
   fprintf("%d,\n",lut(n));
end