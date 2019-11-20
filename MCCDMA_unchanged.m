clc
clear all;
close all

users = 2;      % Number of user

%---------------Generation of walsh Code-------

n = 4;                   %Number of Data Sub-Carriers
walsh = hadamard(n);     
code1 = walsh(2,:);      %Taking 2nd Row of Walsh code for User1
code2 = walsh(4,:);      %Taking 4th Row of Walsh code for User2

%--------------Generating data for User1----------

N = 10^4;                 %Number of bits for data_user1 
data_user1 = rand(1,N)>0.5; %Generating data for User1
data_user1bpsk = 2*data_user1-1;    %BPSK modulation

%--------------Spreading & ifft for User1--------------

data_user11 = data_user1bpsk';
spdata_user1 = data_user11*code1;
spdata12 = (spdata_user1)';
ifftdata_user1 = ifft(spdata12);
ifftdata12 = ifftdata_user1';


%------------------Append cyclic Prefix for User1------

y1 = [ifftdata12(:,[(n-2):n]) ifftdata12];
transdata1 = y1';
tx_user1 = transdata1;

%-----------------Generating data fro User2------------

N = 10^4;                 %Number of bits for data_user1 
data_user2 = rand(1,N)>0.5; %Generating data for User1
data_user2bpsk = 2*data_user2-1;    %BPSK modulation

%--------------Spreading & ifft for User2--------------

data_user22 = data_user2bpsk';
spdata_user2 = data_user22*code2;
spdata22 = (spdata_user2)';
ifftdata_user2 = ifft(spdata22);
ifftdata22 = ifftdata_user2';


%------------------Append cyclic Prefix for User2------

y2 = [ifftdata22(:,[(n-2):n]) ifftdata22];
transdata2 = y2';
tx_user2 = transdata2;


%-----------------Adding data for transmission of all Users-------
x = tx_user1 +tx_user2;

%------------------Creating Rayleigh Channel-----------

Taps = 4;          % creating Taps
p1 = 1-exp(0.5/2.3);           % Power of taps 1 to 4
p2 = 1-exp(0.9/2.3);
p3 = 1-exp(0.7/2.3);
p4 = 1-exp(0.2/2.3);

gain1 = sqrt(p1/2)*[rand(1,N) + j*rand(1,N)];     %Gain for 1 to 4
gain2 = sqrt(p2/2)*[rand(1,N) + j*rand(1,N)];    
gain3 = sqrt(p3/2)*[rand(1,N) + j*rand(1,N)];     
gain4 = sqrt(p4/2)*[rand(1,N) + j*rand(1,N)];    


x11 = x(:);
x12 = reshape(x11,1,length(x11));
i = 1:length(x12);
delay1 =1;

for i = delay1+1:length(x12)
  x13(i) = x(i-delay1);
 end
 
 delay2 = 2;
 for i = delay2+1:length(x12)
   x14(i) = x(i-delay2);
 end
 delay3 =3;
 for i = delay3+1:length(x12)
   x15(i) = x(i-delay3);
 end
 
 x1 = reshape(x13,(n+3),length(x13)/(n+3));
 x2 = reshape(x14,(n+3),length(x14)/(n+3));
 x3 = reshape(x15,(n+3),length(x15)/(n+3));
 

ch1 = repmat(gain1,(n+3),1);
ch2 = repmat(gain2,(n+3),1);
ch3 = repmat(gain3,(n+3),1);
ch4 = repmat(gain4,(n+3),1);

data_channel = x.*ch1 + x1.*ch2 + x2.*ch3 +x3.*ch4; %Passing data through channel

%--------------Addition of AWGN-------------
data_noise1 = data_channel(:);
data_noise2 = reshape(data_noise1,1,length(data_noise1));

noise = 1/sqrt(2)* [randn(1,length(data_noise2)) + j*randn(1,length(data_noise2))];
snr = [0:20];

for i = 1 :length(snr)
  y = data_noise2 + (sqrt(1)*10^(-snr(i)/20))*noise;

%---------------Receiver---------------

data_received = y;
%----------------------Removing Cyclic Prefix-------------
rx1 = reshape(data_received,(n+3), length(data_received)/(n+3));
rx12 =rx1';
rx13 = rx12(:,[(4:(n+3))]);
rx14 = rx13' ;

%----------------------Taking FFT----------------------------

fft_data_received = fft(rx14);

%---------------------------equilization of the channel-----------------
channel_response = fft([gain1;gain2;gain3;gain4],n);
data_equilized = fft_data_received.*conj(channel_response);

%---------------------------------BER of Data User1-----------------
recdata11 = (data_equilized'*code1')';
recdata12 = real(recdata11)>0;
errors_user1(i) = size(find([data_user1 - recdata12]),2);
SBer1 = errors_user1/N;

%--------------------------------BER of Data User2----------------
recdata21 = (data_equilized'*code2')';
recdata22 = real(recdata21)>0;
errors_user2(i) = size(find([data_user2 - recdata22]),2);
SBer2 = errors_user2/N;
end 
 
%---------------------Theorectical result----------------------

snrlnr = 10.^(snr/10);
TBer = 0.5*erfc(sqrt(snrlnr));
TBerf = 0.5.*(1-sqrt(snrlnr./(snrlnr+1)));

%---------------------Displaying the Result------------
figure
semilogy(snr,TBer,'c*-','lineWidth',2);
hold on;
semilogy(snr,TBerf,'r-','lineWidth',3);
hold on;
semilogy(snr,SBer1,'bd','lineWidth',4);
hold on;
semilogy(snr,SBer2,'go-','lineWidth',1);
hold on;
grid on;
legend('TheorecticalBER for Bpsk on Awgn', 'TheorecticalBER for Bpsk on Rayleigh channel' ,'sim','location','best');
xlabel ('Eb/No,dB');
ylabel('bit Error rate');
title('BER vs Eb/No on Rayleigh Channel')







