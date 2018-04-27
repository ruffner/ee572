clear;
s=serial('COM18');
s.BaudRate = 115200;

fopen(s);

%fprintf(s,"%d",300);

rpm=300;

i=1;
while (i<250)

%     if i==50
%        fprintf(s,"%d",750);
%     end
%     
%     if i==175
%         fprintf(s,"%d",350);
%     end

    %fprintf(s,"%d",rpm);
    %rpm=rpm+3;
    
    sSerialData = fscanf(s); %read sensor
    flushinput(s);
    t = strsplit(sSerialData,'\t'); % same character as the Arduino code
    mData(i,1) = str2double(t(1)); 
    mData(i,2) = str2double(t(2));
    mData(i,3) = str2double(t(3)); 
    mData(i,4) = str2double(t(4));

    flushinput(s);
    disp(sSerialData);

    i=i+1;
end
% close the serial port!
fclose(s);
delete(s);
clear s;

Ts=.1;
tax=0:Ts:25-2*Ts;
tax=tax';

figure(1)
plot(tax,mData(:,1:2));
legend('Target RPM','Actual RPM')

figure(2)
plot(tax,mData(:,3:4));
legend('W_k','Y_k');