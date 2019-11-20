function power= kalman_filter(j,decV,p)

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


    
   W(:,:,j)=Q*randn(2,1)+mu1; %Gaussian white noise at input
   V(j)=R*randn(1,1)+mu2;% Gaussian whote noise at output
   X(:,:,j+1)=A*X(:,:,j) + W(:,:,j); %State Equation
   Y(:,:,j)=  C*X(:,:,j) + V(j);%Measurement Equation
   Y(:,:,j+1)=  C*X(:,:,j+1) + V(j); %Output state estimation
  
   %Kalman Equations at the Sensor(XS)
   
   XS(:,:,j+1)=A*XS(:,:,j); 
   PS(:,:,j+1)=A*PS(:,:,j)*A' + Q;
   KS(:,:,j+1)=PS(:,:,j+1)*C'*inv((R+(C*PS(:,:,j+1)*C')));
   XS(:,:,j+1)=XS(:,:,j+1)+KS(:,:,j+1)*(Y(:,:,j+1)-C*XS(:,:,j+1));
   PS(:,:,j+1)=(I-KS(:,:,j+1)*C)*PS(:,:,j+1);
  
   
   
   %Kalman Equations at the Receiver(XR) AND the Eavesdropper(XE)
   
   if decV==1 % Decision bit is one>> We are transmitting XS OR Y depending on Packet(k)
   XR(:,:,j+1)=A*XR(:,:,j); 
   PR(:,:,j+1)=A*PR(:,:,j)*A' + Q;
   KR(:,:,j+1)=PR(:,:,j+1)*C'*inv((R+(C*PR(:,:,j+1)*C')));
   XR(:,:,j+1)=XR(:,:,j+1)+ KR(:,:,j+1)*(Y(:,:,j)-C*XR(:,:,j+1));
   PR(:,:,j+1)=(I-KR(:,:,j+1)*C)*PR(:,:,j+1);
   
   XE(:,:,j+1)=A*XE(:,:,j); 
   PE(:,:,j+1)=A*PE(:,:,j)*A' + Q;
   KE(:,:,j+1)=PE(:,:,j+1)*C'*inv((R+(C*PE(:,:,j+1)*C')));
   XE(:,:,j+1)=XE(:,:,j+1)+ KE(:,:,j+1)*(Y(:,:,j)-C*XE(:,:,j+1));
   PE(:,:,j+1)=(I-KE(:,:,j+1)*C)*PE(:,:,j+1);
   
   else %VK==0>> We are not transmitting ANYTHING and we need to estimate the state at the receiver
   % Kalman Estimation with intermittent observations
   
   XR(:,:,j+1)=A*XR(:,:,j); 
   PR(:,:,j+1)=A*PR(:,:,j)*A' + Q;
   KR(:,:,j+1)=PR(:,:,j+1)*C'*inv((R+(C*PR(:,:,j+1)*C')));
   XR(:,:,j+1)=XR(:,:,j+1)+KR(:,:,j+1)*(Y(:,:,j+1)-C*XR(:,:,j+1));
   PR(:,:,j+1)=PR(:,:,j+1)-KR(:,:,j+1)*C*PR(:,:,j+1);
   
   XE(:,:,j+1)=A*XE(:,:,j); 
   PE(:,:,j+1)=A*PE(:,:,j)*A' + Q;
   KE(:,:,j+1)=PE(:,:,j+1)*C'*inv((R+(C*PE(:,:,j+1)*C')));
   XE(:,:,j+1)=XE(:,:,j+1)+KE(:,:,j+1)*(Y(:,:,j+1)-C*XE(:,:,j+1));
   PE(:,:,j+1)=PE(:,:,j+1)-KE(:,:,j+1)*C*PE(:,:,j+1);
   
   end
  
 if PR(:,:,j+1)>PE(:,:,j+1)
     power=p; 
 else
     power=1;
 end
 
%  
%  W(:,:,k)=Q*randn(2,1)+mu1; %Gaussian white noise at input
%    V(k)=R*randn(1,1)+mu2;% Gaussian whote noise at output
%    X(:,:,k+1)=A*X(:,:,k) + W(:,:,k); %State Equation
%    Y(:,:,k)=  C*X(:,:,k) + V(k);%Measurement Equation
%    Y(:,:,k+1)=  C*X(:,:,k+1) + V(k); %Output state estimation
%   
%    %Kalman Equations at the Sensor(XS)
%    
%    XS(:,:,k+1)=A*XS(:,:,k); 
%    PS(:,:,k+1)=A*PS(:,:,k)*A' + Q;
%    KS(:,:,k+1)=PS(:,:,k+1)*C'*inv((R+(C*PS(:,:,k+1)*C')));
%    XS(:,:,k+1)=XS(:,:,k+1)+KS(:,:,k+1)*(Y(:,:,k+1)-C*XS(:,:,k+1));
%    PS(:,:,k+1)=(I-KS(:,:,k+1)*C)*PS(:,:,k+1);
%   
%    
%    
%    %Kalman Equations at the Receiver(XR) AND the Eavesdropper(XE)
%    
%    if decV==1 % Decision bit is one>> We are transmitting XS OR Y depending on Packet(k)
%    XR(:,:,k+1)=A*XR(:,:,k); 
%    PR(:,:,k+1)=A*PR(:,:,k)*A' + Q;
%    KR(:,:,k+1)=PR(:,:,k+1)*C'*inv((R+(C*PR(:,:,k+1)*C')));
%    XR(:,:,k+1)=XR(:,:,k+1)+ KR(:,:,k+1)*(Y(:,:,k)-C*XR(:,:,k+1));
%    PR(:,:,k+1)=(I-KR(:,:,k+1)*C)*PR(:,:,k+1);
%    
%    XE(:,:,k+1)=A*XE(:,:,k); 
%    PE(:,:,k+1)=A*PE(:,:,k)*A' + Q;
%    KE(:,:,k+1)=PE(:,:,k+1)*C'*inv((R+(C*PE(:,:,k+1)*C')));
%    XE(:,:,k+1)=XE(:,:,k+1)+ KE(:,:,k+1)*(Y(:,:,k)-C*XE(:,:,k+1));
%    PE(:,:,k+1)=(I-KE(:,:,k+1)*C)*PE(:,:,k+1);
%    
%    else %VK==0>> We are not transmitting ANYTHING and we need to estimate the state at the receiver
%    % Kalman Estimation with intermittent observations
%    
%    XR(:,:,k+1)=A*XR(:,:,k); 
%    PR(:,:,k+1)=A*PR(:,:,k)*A' + Q;
%    KR(:,:,k+1)=PR(:,:,k+1)*C'*inv((R+(C*PR(:,:,k+1)*C')));
%    XR(:,:,k+1)=XR(:,:,k+1)+KR(:,:,k+1)*(Y(:,:,k+1)-C*XR(:,:,k+1));
%    PR(:,:,k+1)=PR(:,:,k+1)-KR(:,:,k+1)*C*PR(:,:,k+1);
%    
%    XE(:,:,k+1)=A*XE(:,:,k); 
%    PE(:,:,k+1)=A*PE(:,:,k)*A' + Q;
%    KE(:,:,k+1)=PE(:,:,k+1)*C'*inv((R+(C*PE(:,:,k+1)*C')));
%    XE(:,:,k+1)=XE(:,:,k+1)+KE(:,:,k+1)*(Y(:,:,k+1)-C*XE(:,:,k+1));
%    PE(:,:,k+1)=PE(:,:,k+1)-KE(:,:,k+1)*C*PE(:,:,k+1);
%    
%    end
%   
%  if PR(:,:,k)>PE(:,:,k)
%      power=p; 
%  else
%      power=1;
%  end

% %Trace of Covariance Matrix PR at the Reciever
%  display('PR(k)');
%  disp(trace(PR(:,:,k)));
%  
% %Trace of Covariance Matrix PE at the Eavesdropper
%  display('PE(k)');
%  disp(trace(PE(:,:,k)));
%  %plot(trace(PR(:,:,k)),trace(PE(:,:,k)))
 

end