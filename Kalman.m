function [phEst,wEst]= Kalman(z,Kg,F,H)
theta=atan2(imag(z),real(z));
phEst=zeros(1,128);
wEst=zeros(1,128);
z_encontra=zeros(1,128);
x_est_k=[0;0];
for n=1:128
    x_est_kk=F*x_est_k;%first step
    z_est=H*x_est_kk;%second step
    x_est_k=x_est_kk+(transpose(Kg(n,:))*(theta(n)-z_est));%third step
    phEst(n)=x_est_k(1);
    wEst(n)=x_est_k(2);
    z_encontra(n)=phEst(n)+wEst(n);
end
figure
plot(theta(1,:))
hold on
plot(z_encontra(1,:))
%display(x_est_k)
% figure
% ax1=subplot(2,1,1);
% plot(theta)
% title(ax1,'Zk');
% ylabel(ax1,'Zk');
% xlabel(ax1,'k');
% grid on;
% ax2=subplot(2,1,2);
% plot(kf(1,:))
% title(ax2,'Frequency Gain')
% ylabel(ax2,'KF')
% xlabel(ax2,'N');
% grid on;
% plot(theta)
% grid on
end