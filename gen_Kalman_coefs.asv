function [kph,kf]=gen_Kalman_coefs(R,Q,S0,F,G,H,N)
K=zeros(N,2);
S=S0;
for n=0:N-1
    S0=(F*S*transpose(F))+Q; %Covariance Matrix of the prediction error
    P=S0*transpose(H)*(R+(H*S0*transpose(H)))^-1;%Kalman Gains    
    S=(eye(2)-P*H)*S0;
    K(n+1,1)=P(1);
    K(n+1,2)=P(2);
end
display(K)
figure
ax1=subplot(2,1,1);
plot(K(:,1))
title(ax1,'Phase Gain');
ylabel(ax1,'KPH');
xlabel(ax1,'N');
grid on;
ax2=subplot(2,1,2);
plot(K(:,2))
title(ax2,'Frequency Gain')
ylabel(ax2,'KF')
xlabel(ax2,'N');
grid on;
kph=P(1);
kf=P(2);
end