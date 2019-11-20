
clc;
clear all;

%Initialisation
A=[0.2 0;0 0.1];%System Matrix
C=[1 2]; %Measurement Matrix
eig(A)
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


DecV=[1;0;1;0;1;1;1;1;1;0;0;0;1;1;1;1;1;1;0;0;1;1;1;1;1;0;0;1;0;1]  %Experimental Decision Bits(Not Optimal)

%Gilbert Elliot Model Code for the Receiever

p = 0.4;
r = 0.45;
total_packs = 31;
check = 100;
while check >= 10
good = 1;
packets = [];
size = 1;
while size <= total_packs
if good == 1
    packets = [packets good];
    good = rand(1) > p;
elseif good == 0
    packets = [packets good];
    good = rand(1) > (1-r);
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

theo_pack_loss_rate = p / (p+r);
act_pack_loss_rate = 1 - received_packs/total_packs;

%EAVESDROPPER
%Gilbert Elliot Model Code for the Eavesdropper

p1 = 0.7;
r1 = 0.25;
total_packsE = 31;
checkE = 100;
while checkE >= 10
goodE = 1;
packetsE = [];
sizeE = 1;
while sizeE <= total_packsE
if goodE == 1
    packetsE = [packetsE goodE];
    goodE = rand(1) > p1;
elseif goodE == 0
    packetsE = [packetsE goodE];
    goodE = rand(1) > (1-r1);
else
    fprintf('error\n');
    break;
end
sizeE = sizeE + 1;
end
fid = fopen('loss_pattern_E','w');
fprintf(fid, '%d ', packetsE);
fclose(fid);
received_packsE = nnz(packetsE);
theo_pack_loss_rateE = 1 - r1 / (p1+r1);
act_pack_loss_rateE = 1 - received_packsE/total_packsE;
checkE = abs(theo_pack_loss_rateE - act_pack_loss_rateE) / theo_pack_loss_rateE * 100;
end

theo_pack_loss_rateE = p1/ (p1+r1);
act_pack_loss_rateE = 1 - received_packsE/total_packsE;

%Channel Dropout probabilities
display("The Channel Probability for Receiver:")
disp(packets);

display("The Channel Probability for Eavesdropper:")
disp(packetsE);

%Transmission
%SYSTEM 

for k=1:30
   W(:,:,k)=Q*randn(2,1)+mu1; %Gaussian white noise at input
   V(k)=R*randn(1,1)+mu2;% Gaussian whote noise at output
   X(:,:,k+1)=A*X(:,:,k) + W(:,:,k); %State Equation
   Y(:,:,k)=  C*X(:,:,k) + V(k);%Measurement Equation
   Y(:,:,k+1)=  C*X(:,:,k+1) + V(k);
%  end
%  
% for k = 1:30 % no. of iteration
  
   %Kalman Equations at the Sensor(XS)
   
   XS(:,:,k+1)=A*XS(:,:,k); 
   PS(:,:,k+1)=A*PS(:,:,k)*A' + Q;
   KS(:,:,k+1)=PS(:,:,k+1)*C'*inv((R+(C*PS(:,:,k+1)*C')));
   XS(:,:,k+1)=XS(:,:,k+1)+KS(:,:,k+1)*(Y(:,:,k+1)-C*XS(:,:,k+1));
   PS(:,:,k+1)=(I-KS(:,:,k+1)*C)*PS(:,:,k+1);
  
   
   
   %Kalman Equations at the Receiver(XR) AND the Eavesdropper(XE)
   
   if DecV(k)==1 % Decision bit is one>> We are transmitting XS OR Y depending on Packet(k)
   XR(:,:,k+1)=A*XR(:,:,k); 
   PR(:,:,k+1)=A*PR(:,:,k)*A' + Q;
   KR(:,:,k+1)=PR(:,:,k+1)*C'*inv((R+(C*PR(:,:,k+1)*C')));
   XR(:,:,k+1)=XR(:,:,k+1)+ packets(k)*KR(:,:,k+1)*(Y(:,:,k)-C*XR(:,:,k+1));
   PR(:,:,k+1)=(I-packets(k)*KR(:,:,k+1)*C)*PR(:,:,k+1);
   
   XE(:,:,k+1)=A*XE(:,:,k); 
   PE(:,:,k+1)=A*PE(:,:,k)*A' + Q;
   KE(:,:,k+1)=PE(:,:,k+1)*C'*inv((R+(C*PE(:,:,k+1)*C')));
   XE(:,:,k+1)=XE(:,:,k+1)+ packetsE(k)*KE(:,:,k+1)*(Y(:,:,k)-C*XE(:,:,k+1));
   PE(:,:,k+1)=(I-packetsE(k)*KE(:,:,k+1)*C)*PE(:,:,k+1);
   
   else %VK==0>> We are not transmitting ANYTHING and we need to estimate the state at the receiver
   % Kalman Estimation with intermittent observations
   
   XR(:,:,k+1)=A*XR(:,:,k); 
   PR(:,:,k+1)=A*PR(:,:,k)*A' + Q;
   KR(:,:,k+1)=PR(:,:,k+1)*C'*inv((R+(C*PR(:,:,k+1)*C')));
   XR(:,:,k+1)=XR(:,:,k+1)+packets(k+1)*KR(:,:,k+1)*(Y(:,:,k+1)-C*XR(:,:,k+1));
   PR(:,:,k+1)=PR(:,:,k+1)-packets(k+1)*KR(:,:,k+1)*C*PR(:,:,k+1);
   
   XE(:,:,k+1)=A*XE(:,:,k); 
   PE(:,:,k+1)=A*PE(:,:,k)*A' + Q;
   KE(:,:,k+1)=PE(:,:,k+1)*C'*inv((R+(C*PE(:,:,k+1)*C')));
   XE(:,:,k+1)=XE(:,:,k+1)+packetsE(k+1)*KE(:,:,k+1)*(Y(:,:,k+1)-C*XE(:,:,k+1));
   PE(:,:,k+1)=PE(:,:,k+1)-packetsE(k+1)*KE(:,:,k+1)*C*PE(:,:,k+1);
   
   
 end
 
 
  %PLOTTING
 
  %Plotting Estimates of State 1(X1)
  figure(1)
  plot(k,X(1,:,k),'k*',k,XS(1,:,k),'g*',k,XR(1,:,k),'r*',k,XE(1,:,k),'m*');
  hold on;
  xlabel("k");
  ylabel("State 1");
  title("Variation of State 1 with k");
  legend('X(1)','XS(1)','XR(1)','XE(1)','location',"south");
  grid on;
   
   
  %Plotting Estimates of State 2(X2)
  figure(2)
  plot(k,X(2,:,k),'k*',k,XS(2,:,k),'g*',k,XR(2,:,k),'r*',XE(2,:,k),'m*');
  hold on;
  xlabel("k");
  ylabel("State 2");
  title("Variation of State 2 with k "); 
  legend('X(2)','XS(2)','XR(2)','XE(1)','location',"north");
  grid on;

  
  %Plotting Trace of Covariance Matrices at the Receiver
  figure(3)
  plot(k,trace(PR(:,:,k)),'r*');
  hold on;
  xlabel("k");
  ylabel("Trace of Covariance Matrix");
  title("Trace of Covariance Matrix vs k (plots for Sensor, Receiver & Eavesdroper)");
  legend('Tr(PR)','location',"north");
  grid on;
  
  %Plotting Trace of Covariance Matrices at the Eavesdropper
  figure(4)
  plot(k,trace(PE(:,:,k)),'m*');
  hold on;
  xlabel("k");
  ylabel("Trace of Covariance Matrix");
  title("Trace of Covariance Matrix vs k (plots for Sensor, Receiver & Eavesdroper)");
  legend('Tr(PE)','location',"north");
  grid on;
  
  
  
%Trace of Covariance Matrix PR at the Reciever
 display("PR(k)");
 disp(trace(PR(:,:,k)));
 
%Trace of Covariance Matrix PE at the Eavesdropper
 display("PE(k)");
 disp(trace(PE(:,:,k)));
 
 end
 
display("The Actual States X:")
disp(X);

display("The Sensor Estimates XS:")
disp(XS);

display("The Receiver Estimates XR:")
disp(XR);

display("The Eavesdropper Estimates XE:")
disp(XE);
 
display("The Sensor Covariance Matrices PS:")
disp(PS);

display("The Receiver Covariance Matrices PR:")
disp(PR);

display("The Eavesdropper Covariance Matrices PE:")
disp(PE);
 
 
