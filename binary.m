clc;clear; 

csv_content=importdata('collect_5840.csv'); %import the data 
t=(csv_content.data(:,3)-csv_content.data(1,3))./1000000; %time in seconds
X=csv_content.data(:,7)-mode(csv_content.data(:,7));
L=1000*(t(end)-t(1)); %signal length in milliseconds
Fs=1000*length(t)/L; %sampling frequency in Hz
T=1/Fs; %period of sampling in seconds
N=length(t);
for i=1:N
    if X(i)>7
        X(i)=1;
    elseif X(i)<-7
        X(i)=1;
    else
        X(i)=0;
    end 
end

figure (1)

plot(t, X)
%
Y=fft(X);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure (2)
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

