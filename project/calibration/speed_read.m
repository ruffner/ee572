clear;
s=serial('/dev/ttyACM0');
s.BaudRate = 115200;

fopen(s);

i=1;
while (i<2000)
    sSerialData = fscanf(s); %read sensor
    flushinput(s);
    t = strsplit(sSerialData,','); % same character as the Arduino code
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


figure(1)
plot(mData(:,1),mData(:,2:4));
legend('Target RPM','Actual RPM','W_k','Y_k')