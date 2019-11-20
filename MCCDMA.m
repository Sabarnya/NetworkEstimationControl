clc
clear all;
close all


N = 10;                %Number of bits for data_user1 
snr=linspace(0,1,10);             %SNR
data_user1 = rand(1,N)>0.5; %Generating data for User1

p = (0.5/2.3);
%p2 = (0.2/2.3);
gain1 = sqrt(p/2)*[rand(1,N)];% + j*rand(1,N)];
gain2 = sqrt(p/2)*[rand(1,N)];

for i=1:N
   for j=1:length(snr)
    lambda1(i,j) = (1-exp(snr(1,j)*p*gain1(1,i)));
    lambda2(i,j) = (1-exp((snr(1,j)*p*gain2(1,i))));
  
    if lambda1(i,j)>lambda2(i,j)
        decV=1;
    else
        decV=0;
    end
    
   power(i,j)= kalman_filter(j,decV,p);
  
   end
end
power

% data_noise1 = data_channel(:);
% data_noise2 = reshape(data_noise1,1,length(data_noise1));
% 
% noise = 1/sqrt(2)* [randn(1,length(data_noise2)) + j*randn(1,length(data_noise2))];
% snr = [0:20];


