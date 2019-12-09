clc
clear all
close all

%Initialisation
bm=7; %number of bits
ber=0.6; %bit error rate
u1=3e-4; %initial power given to the channel
delu=3e-5; %incremental power given at the sensor
rho=e6; %tuning parameter for power control

A=[1.6718 -0.9048;1 0];%System Matrix
C=[1 2]; %Measurement Matrix
%input_signal=Normal_db(mu0,sig0,0,1,100); %Gaussian signal
mu0=0;%mean of the initial random state vector
mu1=[0;0];%mean of Proess noise W
mu2=0;%mean of measurment noise V
sig0=0.3;%standard deviation of the random initial state vector X
I=eye(size(A));% Identity matrix used in Kalman equations
P0=0.3*I; %Initial Gaussian distribution with covariance matrix
Q= 0.5*I; %Driving noise 
R= 1/100;
X(:,:,1)=sig0*randn(2,1)+mu0; %Initial state
PS(:,:,1)=[0 0;0 0]; %Initial estimate of state covariance at sensor
XS(:,:,1)=[0;0]; %Initial estimate of state at sensor
PE(:,:,1)=[0 0;0 0];%Initial estimate of state covariance at the Eavesdropper
XE(:,:,1)=[0;0];%Initial estimate of state at the Eavesdropper
XR(:,:,1)=[0;0];%Initial estimate of state at the Receiver
PR(:,:,1)=P0;%Initial estimate of state at the Receiver

gain1 = sqrt(u1/2)*[rand(1,1)]; %gain for receiver channel
gain2 = sqrt(u1/2)*[rand(1,1)]; %gain for eavesdropper channel

lambda1 = (1-exp(-ber*u1*gain1)); %probability of reaching at the receiving end
lambda2 = (1-exp(-ber*u1*gain2)); %probability of reaching at the eavesdropper end

% calculation of the probability of success

if lambda1>=0.5
    gammaR=1;    %if received at the receiver end successfully
else
    gammaR=0;    %otherwise
end
    
if lambda2>=0.7
   gammaE=1;     %if received at the eavesdropper end successfully
else 
   gammaE=0;     %otherwise
end

if u1>0
    thetaR=gammaR;
    thetaE=gammaE;
else
    thetaR=0;
    thetaE=0;
end
CR=thetaR.*C; %time varying kalman filter output matrix at receiver end
CE=thetaE.*C; %time varying kalman filter output matrix at eavesdropper end

for j=2:30
    
    %sensor state estimation
    W(:,:,j)=Q*randn(2,1)+mu1; %Gaussian white noise at input
    V(j)=R*randn(1,1)+mu2; %Gaussian whote noise at output
    X(:,:,j+1)=A*X(:,:,j) + W(:,:,j); %State Equation
    Y(:,:,j)=C*X(:,:,j) + V(j);%Measurement Equation
    Y(:,:,j+1)=C*X(:,:,j+1) + V(j); %Output state estimation
    
    %state estimation at the receiver end while the signal reached there.(gammaR=1)
    if (gammaR==1)
        PR(:,:,j)= (I-KR(:,:,j)*CR)*PR(:,:,j-1);
        PR(:,:,j+1)=A*PR(:,:,j)*A' + Q;
        KR(:,:,j+1)=PR(:,:,j+1)*CR'*inv((R+(CR*PR(:,:,j+1)*CR')));
        XR(:,:,j+1)= A*XR(:,:,j)+KR(:,:,j+1)*(Y(:,:,j+1)-(CR(:,:,j+1)*A*XR(:,:,j)));
    end
    if (j==2)
     u(j-1)=u1;
    end
%     u(j)=u(j-1)+delu;
%     powerR(j)= tr(PR(:,:,j)+rho*(bm*u(j))); %total power at receiver
    
    %state estimation at the eavesdropper end while the signal reached there.(gammaE=1)
    if (gammaE==1)
        PE(:,:,j)= (I-KE(:,:,j)*CE)*PE(:,:,j-1);
        PE(:,:,j+1)=A*PE(:,:,j)*A' + Q;
        KE(:,:,j+1)=PE(:,:,j+1)*CE'*inv((R+(CE*PE(:,:,j+1)*CE')));
        XE(:,:,j+1)= A*XE(:,:,j)+KE(:,:,j+1)*(Y(:,:,j+1)-(CE(:,:,j+1)*A*XE(:,:,j)));
    end
    
    %u(j)=u(j-1)-delu;
    %powerE(j)= tr(PE(:,:,j)-rho*(bm*u(j))); %total power at eavesdropper
    
    if (gammaR==gammaE==1)
        if (trace(PR(:,:,j+1))< trace(PE(:,:,j+1)))
          power(j)=tr(PR(:,:,j)+rho*(bm*u(j-1)));
        elseif (trace(PR(:,:,j+1))==trace(PE(:,:,j+1)))
          u(j)=u(j-1)+delu;
          power(j)=tr(PR(:,:,j)+rho*(bm*u(j)));
        else (trace(PR(:,:,j+1))>trace(PE(:,:,j+1)))
          u(j)=u(j-1)+delu;
          power(j)=tr(PR(:,:,j)-rho*(bm*u(j)));
        end
    elseif(gammaR==1 && gammaE==0)
        power(j)=power(j-1);
    elseif(gammaR==0 && gammaE==1)
        u(j)=u(j-1)+delu;
        power(j)=tr(PR(:,:,j)+rho*(bm*u(j-1)));
    else (gammaR==gammaE==0)
        continue;
    end
end


    