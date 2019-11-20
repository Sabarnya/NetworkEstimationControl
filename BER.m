clear all
close all
clc

EbNo = 0:2:20;
ber = zeros(length(EbNo),20);
for L = 1:20 
    ber(:,L) = berfading(EbNo,'qam',16,L);
end
semilogy(EbNo,ber,'b')
text(18.5, 0.02, sprintf('L=%d',1))
text(18.5, 1e-11, sprintf('L=%d',20))
title('QAM over fading channel with diversity order 1 to 20')
xlabel('E_b/N_0 (dB)')
ylabel('BER')
grid on