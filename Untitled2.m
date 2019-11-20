clc;
clear all;
close all;

%Initialisation
A=[0.2 0;0 0.1];%System Matrix
C=[1 2]; %Measurement Matrix
eig(A);
mu0=0;%mean of the initial random state vector
mu1=[0;0];%mean of Proess noise W
mu2=0;%mean of measurment noise V
sig0=0;%standard deviation of the random initial state vector X
%sig1=2;%standard deviation of noise W; Variance of W is then sig1^2
%sig2=3;%standard deviation of noise V

X(:,:,1)=sig0*randn(2,1)+mu0; %Initial state
%X(:,:,1)=[0;0];
Q=[2 0;0 4]; %Initialise covariance matrix of Process Noise W
R=0.3; %Initialise covariance of Measurement Noise V
I=eye(size(A));% Identity matrix used in Kalman equations
PS(:,:,1)=[0 0;0 0]; %Initial estimate of state covariance
XS(:,:,1)=[0;0]; %Initial estimate of state
PE(:,:,1)=[0 0;0 0];%Initial estimate of state covariance at the Eavesdropper
XE(:,:,1)=[0;0];%Initial estimate of state at the Eavesdropper
XR(:,:,1)=[0;0];%Initial estimate of state at the Receiver
PR(:,:,1)=[0 0;0 0];%Initial estimate of state at the Receiver
PS1(:,:,1)=[0 0;0 0]; %Initial estimate of state covariance
XS1(:,:,1)=[0;0]; %Initial estimate of state

p = 0.4;
r = 0.45;
total_packs = 10;
check = 100;
while check >= 10
good = 1;
packets = [];
size = 1;
u = 10;
h=sigma*randn(n,1); 
while size <= total_packs
if good == 1
    packets = [packets good];
    good = rand(1) > p;
    P=1-exp(u.g); 
elseif good == 0
    packets = [packets good];
    good = rand(1) > (1-r);
    P=0;
else
    fprintf('error\n');
    break;
end
size = size + 1;
end
fid = fopen('Loss_Pattern.txt','w');
fprintf(fid, '%d ', packets);
fclose(fid);
received_packs = nnz(packets);
theo_pack_loss_rate = 1 - r / (p+r);
act_pack_loss_rate = 1 - received_packs/total_packs;
check = abs(theo_pack_loss_rate - act_pack_loss_rate) / theo_pack_loss_rate * 100;
end