D=dlmread('speedcal-inc5.txt','\t');
P=polyfit(D(:,1),D(:,2),3)
plot(D(:,1),D(:,2),D(:,1),polyval(P,D(:,1)))
