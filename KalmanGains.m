function [kph,kf]=KalmanGains(R,Q,S0,F,G,H,N)
for k=1:N-1
    P=S0*transpose(H)*((R+H*S0*transpose(H))^-1);
    S0=F*S0*transpose(F)+Q;
    S1=(eye(2)-P*H)*S0;
end
kph=P(1);
kf=P(2);
end