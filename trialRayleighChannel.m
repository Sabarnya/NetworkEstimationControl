clear all
close all
clc

time_1 = (linspace (0, 10, 1000));%time
unitstep = time_1>=0;
signal_input = unitstep;%sine wave
         
plot (time_1, signal_input, 'b*');grid on;%blue=signal_in
xlabel('time');
ylabel('amplitude');
title('Rayleigh fading channel with two path sine wave input')
hold on
for k = 1:10%# iterations
    tau=round(50*rand(1,1)+1);% variable delay(phase shift)
    g1=round(.5*rand(1,1)+1);%variable gain or attenuation
    signal_out= g1*[zeros(1,tau) signal_input(1:end-tau)];
    plot (time_1,(signal_out),'r')%red=signal_out
    pause (2)%~ seconds
end
hold off

% clear
% f_c=1e3;%carrier frequency(no modulation)
% time_1 = (linspace (0, 10, 1000));%time
% signal_in = sin (2 * pi *f_c* time_1);%sine wave
%          
% plot (time_1, signal_in, 'b');grid on;%blue=signal_in
% xlabel('time');ylabel('amplitude');
% title('Rayleigh fading channel with two path sine wave input')
% hold on
% for ii = 1:10%# iterations
%     tau=round(50*rand(1,1)+1);% variable delay(phase shift)
%     g1=1;%fixed gain
%     g2=round(.5*rand(1,1)+1);%variable gain or attenuation
%     signal_out=g1*signal_in + g2*[zeros(1,tau) signal_in(1:end-tau)];
%   plot (time_1,(signal_out),'r')%red=signal_out
%   pause (2)%~ seconds
% end
% hold off