clearvars; close all; clc;

ntap = 3; %number of channel coefficients
nBits = 100000; % number of bits
data = randi([0 1],1,nBits); % bit stream to send
tx_bpsk = real(pskmod(data,2)); %binary psk modulated signal
snr = 0:5:20; %signal to noise ratios
h=[1 rand(1,ntap-1)]; %Channel Impulse Response
ch_out = conv(tx_bpsk,h); % channel output


num_states = 4;  % number of states in the trellis
survivor_node = zeros(num_states,nBits); % survivor nodes
survivor_ip = zeros(num_states,nBits); % survivor inputs
path_metric = zeros(num_states,nBits+1); % path metrics
index_temp = [0;1*2;2*2;3*2];

Prev_State = [1,2;3,4;1,2;3,4]; % PREVIOUS STATES
Prev_State_trans =  [1,3,1,3;2,4,2,4]; % TRANSPOSE OF P_State
Prev_Ip = [1,1;1,1;2,2;2,2]; % Previous inputs
Prev_Ip_trans = [1,1,2,2;1,1,2,2]; % tranpose of PREVIOUS INPUTS
Outputs_prev = [1,2;3,4;5,6;7,8];



for j = 1:length(snr)

    Error = 0;

    rec1 = awgn(ch_out,snr(j),'measured'); % awgn is added to signal
    rec1 = rec1(1:end-ntap+1);%received signal
    rec2 = awgn(tx_bpsk,snr(j),'measured'); % awgn is added to signal

    for k= 1:nBits                  
        if ((rec2(k)<0 && data(k)==0)||(rec2(k)>0 && data(k)==1))
                 Error=Error+1;
        end
    end
    BER2(j) = Error/nBits;
    


    metric(1,:) = (rec1-(h(1)+h(2)+h(3))).^2;
    metric(2,:) = (rec1-(h(1)+h(2)-h(3))).^2;
    metric(3,:) = (rec1-(h(1)-h(2)+h(3))).^2;
    metric(4,:) = (rec1-(h(1)-h(2)-h(3))).^2;
    
    metric(5,:) = (rec1-(-h(1)+h(2)+h(3))).^2;
    metric(6,:) = (rec1-(-h(1)+h(2)-h(3))).^2;
    metric(7,:) = (rec1-(-h(1)-h(2)+h(3))).^2;
    metric(8,:) = (rec1-(-h(1)-h(2)-h(3))).^2;
    
    for i=  1:nBits  
       [path_metric(:,i+1),index] = min([path_metric(Prev_State(:,1),i)+ metric(Outputs_prev(:,1),i) ...
           path_metric(Prev_State(:,2),i)+ metric(Outputs_prev(:,2),i)],[],2);
       survivor_node(:,i) = Prev_State(index+index_temp)';
       survivor_ip(:,i) = Prev_Ip(index+index_temp)'; 
    end
    
     [~,backtrace_index] = min(path_metric(:,i+1)); % index stack to backtrace the most likely path 
    
    for i = 1:nBits
       ip = survivor_ip(backtrace_index,nBits+1-i);
       backtrace_index = survivor_node(backtrace_index,nBits+1-i);
       decoded_signal(i) = ip - 1; % demodulated to bit stream
    end
    decoded_signal = fliplr(decoded_signal); % reverse the order of backtraced decoded signal


BER1(j) = sum(abs(data-decoded_signal))/nBits; % bit error rate

end

figure(1);

semilogy(snr,BER1);
title('SNR vs BER for BPSK with ISI and AWGN');
xlabel('SNR in dB');
ylabel('BER');
hold on;
semilogy(snr,BER2);
legend('with ISI','without ISI')










