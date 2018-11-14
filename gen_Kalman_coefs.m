function [kph,kf]=gen_Kalman_coefs(R,Q,S0,F,G,H,N)
kph=zeros(1,N);
kf=zeros(1,N);
S=S0;
for n=0:N-1
    S0=(F*S*transpose(F))+Q; %Covariance Matrix of the prediction error
    P=S0*transpose(H)*(R+(H*S0*transpose(H)))^-1;%Kalman Gains    
    S=(eye(2)-P*H)*S0;
    kph(n+1)=P(1);
    kf(n+1)=P(2);
end
figure
ax1=subplot(2,1,1);
plot(kph(1,:))
title(ax1,'Phase Gain');
ylabel(ax1,'KPH');
xlabel(ax1,'N');
grid on;
ax2=subplot(2,1,2);
plot(kf(1,:))
title(ax2,'Frequency Gain')
ylabel(ax2,'KF')
xlabel(ax2,'N');
grid on;
end