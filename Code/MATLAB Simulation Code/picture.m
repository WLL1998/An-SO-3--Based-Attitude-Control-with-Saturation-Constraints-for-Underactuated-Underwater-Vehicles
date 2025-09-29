clc; clear;
load('A1p.mat','pA1');
load('A2p.mat','pA2');
load('A3p.mat','pA3');
load('Xp2p.mat','pXp2');
load('Yp2p.mat','pYp2');
load('Zp2p.mat','pZp2');
load('Time.mat','time');
T = time;
load('Torque1p.mat','pTorque1');
load('Torque2p.mat','pTorque2');
load('Ad.mat','Ad');
load('lambda1_dyn.mat','lambda1_dyn');
load('lambda2_dyn.mat','lambda2_dyn');
load('Thetafq1.mat','Thetafq1');
load('Thetafr1.mat','Thetafr1');

load('A1c2.mat','c2A1');
load('A2c2.mat','c2A2');
load('A3c2.mat','c2A3');
load('Xp2c2.mat','c2Xp2');
load('Yp2c2.mat','c2Yp2');
load('Zp2c2.mat','c2Zp2');
load('Torque1c2.mat','c2Torque1');
load('Torque2c2.mat','c2Torque2');

load('A1c1.mat','c1A1');
load('A2c1.mat','c1A2');
load('A3c1.mat','c1A3');
load('Xp2c1.mat','c1Xp2');
load('Yp2c1.mat','c1Yp2');
load('Zp2c1.mat','c1Zp2');
load('Torque1c1.mat','c1Torque1');
load('Torque2c1.mat','c1Torque2');
load('qd.mat','qd');
load('delta_kine.mat','delta_kine');
load('Ideal_position.mat','Ideal_position');

figure(1);
plot3(Ideal_position(1,:),Ideal_position(2,:),Ideal_position(3,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':'); hold on;
plot3(pXp2,pYp2,pZp2,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');grid on;xlabel('X');ylabel('Y');zlabel('Z');hold on;
plot3(c2Xp2,c2Yp2,c2Zp2,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');grid on;xlabel('X');ylabel('Y');zlabel('Z');hold on;
plot3(c1Xp2,c1Yp2,c1Zp2,'Color','[0.28, 0.47, 0.73]','LineWidth',2);grid on;xlabel('X[m]');ylabel('Y[m]');zlabel('Z[m]');
h2=legend({'Desired path','Proposed','TTSC','TRAC'}, 'Orientation', 'horizontal');
set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');

% figure(1);title('The quaternions result.');hold on;
% subplot(4,1,1); 
% plot(T,c1Q0,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c2Q0,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pQ0,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,qd(1,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% ylabel('\eta');
% subplot(4,1,2);
% plot(T,c1Q1,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c2Q1,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pQ1,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,qd(2,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% ylabel('q_1');
% subplot(4,1,3);
% plot(T,c1Q2,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c2Q2,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pQ2,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,qd(3,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% ylabel('q_2');
% subplot(4,1,4);
% plot(T,c1Q3,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c2Q3,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pQ3,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,qd(4,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% ylim([-.008 .008]);
% xlabel('Time[s]');ylabel('q_3');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% ISE1 = sum((pQ0'-qd(1,:)).^2+(pQ1'-qd(2,:)).^2+(pQ2'-qd(3,:)).^2+(pQ3'-qd(4,:)).^2);
% ISE2 = sum((c1Q0'-qd(1,:)).^2+(c1Q1'-qd(2,:)).^2+(c1Q2'-qd(3,:)).^2+(c1Q3'-qd(4,:)).^2);
% ISE3 = sum((c2Q0'-qd(1,:)).^2+(c2Q1'-qd(2,:)).^2+(c2Q2'-qd(3,:)).^2+(c2Q3'-qd(4,:)).^2);
% 
% ITSE1 = sum(T.*((pQ0'-qd(1,:)).^2+(pQ1'-qd(2,:)).^2+(pQ2'-qd(3,:)).^2+(pQ3'-qd(4,:)).^2));
% ITSE2 = sum(T.*((c1Q0'-qd(1,:)).^2+(c1Q1'-qd(2,:)).^2+(c1Q2'-qd(3,:)).^2+(c1Q3'-qd(4,:)).^2));
% ITSE3 = sum(T.*((c2Q0'-qd(1,:)).^2+(c2Q1'-qd(2,:)).^2+(c2Q2'-qd(3,:)).^2+(c2Q3'-qd(4,:)).^2));


% figure(1);
% plot3(pXp2,pYp2,pZp2,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot3(c1Xp2,c1Yp2,c1Zp2,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot3(c2Xp2,c2Yp2,c2Zp2,'Color','[0.28, 0.47, 0.73]','LineWidth',2,'LineStyle',':'); grid on;
% axis equal;
% h2=legend({'Proposed','TTSC','TRAC'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% figure(1);
% plot(T,c2A1-Ad(1,:),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c1A1-Ad(1,:),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pA1-Ad(1,:),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,zeros(1,8000),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% %yticklabels(strrep(yticklabels,'-','$-$'));
% xlabel('Time[s]');ylabel('Roll angle[deg]');hold on;
% h2=legend('TTSC','TRAC','Proposed','Desired roll angle');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% 
% 
% 
% figure(2);
% plot(T,c2A2,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,(c1A2),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,(pA2),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,Ad(2,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% %yticklabels(strrep(yticklabels,'-','$-$'));
% xlabel('Time[s]');ylabel('Pitch angle[deg]');hold on;
% h2=legend({'TTSC','TRAC','Proposed','Desired pitch angle'});
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% 
% E2p = c2A2-Ad(2,:);
% E1p = c1A2-Ad(2,:);
% E0p = pA2-Ad(2,:);
% % % axes('Position',[0.2,0.17,0.3,0.26]);
% % % plot(T(1,1:432),E2(1,1:432),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% % % plot(T(1,1:432),E1(1,1:432),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% % % plot(T(1,1:432),E0(1,1:432),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% % % plot(T(1,1:432),zeros(1,432),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % % xlim([0,30]);
% % % ylim([-45,5]);
% % % axes('Position',[0.18,0.64,0.25,0.2]);
% % % plot(T(1,2056:2494),E2(1,2056:2494),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% % % plot(T(1,2056:2494),E1(1,2056:2494),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% % % plot(T(1,2056:2494),E0(1,2056:2494),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% % % plot(T(1,2056:2494),zeros(1,439),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % % xlim([205.6,249.4]);
% % % ylim([-5,10]);
% axes('Position',[0.2,0.55,0.65,0.26]);
% plot(T(1,304:8000),c2A2(1,304:8000),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T(1,304:8000),c1A2(1,304:8000),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T(1,304:8000),pA2(1,304:8000),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T(1,304:8000),Ad(2,304:8000),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % 
% figure(3);
% plot(T,c2A3,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c1A3,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% plot(T,pA3,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,Ad(3,:),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% %yticklabels(strrep(yticklabels,'-','$-$'));
% xlabel('Time[s]');ylabel('Yaw angle[deg]');hold on;
% h2=legend({'TTSC','TRAC','Proposed','Desired yaw angle'});
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % %ylim([-0.15,0.1]);
% E2y = c2A3-Ad(3,:);
% E1y = c1A3-Ad(3,:);
% E0y = pA3-Ad(3,:);
% % 
% % axes('Position',[0.18,0.17,0.25,0.2]);
% % plot(T(1,1:432),E2(1,1:432),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% % plot(T(1,1:432),E1(1,1:432),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(1,1:432),E0(1,1:432),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% % plot(T(1,1:432),zeros(1,432),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % xlim([0,30]);
% % ylim([-5,28]);
% % axes('Position',[0.2,0.64,0.3,0.26]);
% % plot(T(1,2056:2494),E2(1,2056:2494),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% % plot(T(1,2056:2494),E1(1,2056:2494),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(1,2056:2494),E0(1,2056:2494),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% % plot(T(1,2056:2494),zeros(1,439),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % xlim([205.6,249.4]);
% % ylim([-5,5]);
% % axes('Position',[0.56,0.64,0.3,0.26]);
% % plot(T(1,3039:3624),E2(1,3039:3624),'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% % plot(T(1,3039:3624),E1(1,3039:3624),'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(1,3039:3624),E0(1,3039:3624),'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% % plot(T(1,3039:3624),zeros(1,586),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% % xlim([303,362]);
% % ylim([-5,5]);
% % 
% % axes('Position',[0.2,0.6,0.3,0.2]);
% % plot(T(1:150,1),r1(1,1:150),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T(1:150,1),r3(1,1:150),'Color','[0.00, 0.45, 0.74]','LineWidth',2);hold on;
% % plot(T(1:150,1),cr1(1,1:150),'Color','[0.85, 0.33, 0.10] ','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(1:150,1),cr3(1,1:150),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;
% % axes('Position',[0.6,0.7,0.3,0.2]);
% % plot(T(5000:6500,1),r1(1,5000:6500),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T(5000:6500,1),r3(1,5000:6500),'Color','[0.00, 0.45, 0.74]','LineWidth',2);hold on;
% % plot(T(5000:6500,1),cr1(1,5000:6500),'Color','[0.85, 0.33, 0.10] ','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(5000:6500,1),cr3(1,5000:6500),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;
% % axes('Position',[0.22,0.18,0.3,0.2]);
% % plot(T(1200:2500,1),r1(1,1200:2500),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T(1200:2500,1),r2(1,1200:2500),'Color','[0.93, 0.69, 0.13]','LineWidth',2);hold on;
% % plot(T(1200:2500,1),cr1(1,1200:2500),'Color','[0.85, 0.33, 0.10] ','LineWidth',2,'LineStyle','-.');hold on;
% % plot(T(1200:2500,1),cr2(1,1200:2500),'Color','[0.93, 0.69, 0.13]','LineWidth',2,'LineStyle','-.');hold on;
% 
% for i=1:1:8000
%     R1(i) = sum(Thetafq1(i,:));
%     R2(i) = sum(Thetafr1(i,:));
% end
% figure(1);
% subplot(3,1,1);
% plot(T,delta_kine(1,:),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;
% plot(T,delta_kine(2,:),'Color','[0.00, 0.45, 0.74]','LineWidth',2);hold on;
% plot(T,delta_kine(3,:),'Color','[0.93, 0.69, 0.13]','LineWidth',2);
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% xlabel('Time[s]');ylabel('Adaptive parameter \delta');hold on;
% h2=legend({'\delta_1','\delta_2','\delta_3'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% subplot(3,1,2);
% plot(T,lambda1_dyn,'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;
% plot(T,lambda2_dyn,'Color','[0.00, 0.45, 0.74]','LineWidth',2);
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% xlabel('Time[s]');ylabel('Adaptive parameter \gamma');hold on;
% h2=legend({'Pitch angle channel','Yaw angle channel'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% subplot(3,1,3);
% plot(T,R1,'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;
% plot(T,R2,'Color','[0.00, 0.45, 0.74]','LineWidth',2);
% xlabel('Time[s]');ylabel({'The weight vector sum', 'of the IT2-FLS |\Theta|'});hold on;
% h2=legend({'Pitch angle channel','Yaw angle channel'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % 
% figure(2);
% subplot(2,1,1);
% plot(T,pTorque1,'Color','[0.28, 0.47, 0.73]','LineWidth',2);hold on;
% plot(T,c2Torque1,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c1Torque1,'Color','[0.02, 0.87, 0.02]','LineWidth',2,'LineStyle','-.');hold on;
% %plot(T,zeros(1,5000),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% %yticklabels(strrep(yticklabels,'-','$-$'));
% xlabel('Time[s]');ylabel('Pitch channel torque[N/m]');hold on;
% h2=legend({'Proposed','TTSC','TRAC'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% subplot(2,1,2);
% plot(T,pTorque2,'Color','[0.02, 0.87, 0.02]','LineWidth',2);hold on;
% plot(T,c2Torque2,'Color','[0.98, 0.75, 0.06]','LineWidth',1.5,'LineStyle','-');hold on;
% plot(T,c1Torque2,'Color','[0.28, 0.47, 0.73]','LineWidth',2,'LineStyle','-.');hold on;
% %plot(T,zeros(1,5000),'Color','[1, 0.1, 0]','LineWidth',2,'LineStyle',':');
% set(gca,'defaultAxesTickLabelInterpreter','latex');
% %yticklabels(strrep(yticklabels,'-','$-$'));
% xlabel('Time[s]');ylabel('Yaw channel torque[N/m]');hold on;
% % h2=legend({'Proposed','TTSC','TRAC'}, 'Orientation', 'horizontal');
% set(h2,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % figure(3);
% % subplot(2, 1, 1);plot(T,lam(:,1),'Color','[0.00, 0.45, 0.74]','LineWidth',2);hold on;grid on;
% % plot(T,cclam(:,1),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % h=legend('Proposed');set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % ylabel('Auxiliary system \lambda_q');
% % subplot(2, 1, 2);plot(T,lam(:,2),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-');hold on; grid on;
% % %plot(T,cclam(:,2),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % %h=legend('Proposed');set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % xlabel('Time[s]');ylabel('Auxiliary system \lambda_r');grid on;hold on;
% 
% % figure(4);
% % axes('Position',[0.1,0.55,0.85,0.35]);
% % plot(T,Torque1(1,:),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T,Torque1c(1,:),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;grid on;
% % plot(T,ccTorque1(1,:),'Color','[0.93, 0.69, 0.13]','LineWidth',2,'LineStyle',':');hold on;grid on;
% % h=legend('\tau_q(proposed)','\tau_q(comparison 1)','\tau_q(comparison 2)');set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % ylabel('Control input \tau_q (N/m)');
% % % 
% % % 
% % axes('Position',[0.1,0.12,0.85,0.35]);
% % plot(T,Torque2(1,:),'Color','[0.85, 0.33, 0.10]','LineWidth',2,'LineStyle','-');hold on; grid on;
% % plot(T,Torque2c(1,:),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;grid on;
% % plot(T,ccTorque2(1,:),'Color','[0.93, 0.69, 0.13]','LineWidth',2,'LineStyle',':');hold on;grid on;
% % h=legend('\tau_r(proposed)','\tau_r(comparison 1)','\tau_q(comparison 2)');set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % xlabel('Time[s]');ylabel('Control input \tau_r (N/m)');grid on;hold on;
% % 
% % axes('Position',[0.15,0.60,0.3,0.15]);
% % plot(T(1:200,1),Torque1c(1,1:200),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;grid on;
% % plot(T(1:200,1),Torque1(1,1:200),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T(1:200,1),ccTorque1(1,1:200),'Color','[0.93, 0.69, 0.13]','LineWidth',2,'LineStyle',':');hold on;grid on;
% % ylim([0,15]);
% % 
% % axes('Position',[0.25,0.3,0.3,0.15]);
% % plot(T(1:10,1),Torque2c(1,1:10),'Color','[0.00, 0.45, 0.74]','LineWidth',2,'LineStyle','-.');hold on;grid on;
% % plot(T(1:10,1),Torque2(1,1:10),'Color','[0.85, 0.33, 0.10]','LineWidth',2);hold on;grid on;
% % plot(T(1:10,1),ccTorque2(1,1:10),'Color','[0.93, 0.69, 0.13]','LineWidth',2,'LineStyle',':');hold on;grid on;
% % ylim([-15,15]);
% 
% % ISE1 = r1*r1'+r2*r2'+r3*r3';
% % ISE2 = cr1*cr1'+cr2*cr2'+cr3*cr3';
% % ISE3 = ccr1*ccr1'+ccr2*ccr2'+ccr3*ccr3';
% % 
% % ITSE1 = T'*((r1.*r1)+(r2.*r2)+(r3.*r3))';
% % ITSE2 = T'*((cr1.*cr1)+(cr2.*cr2)+(cr3.*cr3))';
% % ITSE3 = T'*((ccr1.*ccr1)+(ccr2.*ccr2)+(cr3.*ccr3))';
% % 
% % CE1 = Torque1*Torque1'+Torque2*Torque2';
% % CE2 = Torque1c*Torque1c'+Torque2c*Torque2c';
% % CE3 = ccTorque1*ccTorque1'+ccTorque2*ccTorque2';
% % % 
% % A1 = [ISE1/(ISE1+ISE2+ISE3),ISE2/(ISE1+ISE2+ISE3),ISE3/(ISE1+ISE2+ISE3)];
% % A2 = [ITSE1/(ITSE1+ITSE2+ITSE3),ITSE2/(ITSE1+ITSE2+ITSE3),ITSE3/(ITSE1+ITSE2+ITSE3)];
% % A3 = [CE1/(CE1+CE2+CE3),CE2/(CE1+CE2+CE3),CE3/(CE1+CE2+CE3)];
% %  
% % % 创建一个数据矩阵，每行代表一个分组
% % data = [A1; A2; A3];
% %  
% % % 创建一个标签数组，表示每个分组的标签
% % labels = {'ISE', 'ITSE', 'CE'};
% %  
% % % 画横向柱状图
% % b = barh(data);
% % b(1).FaceColor = [127 255 0]./255;
% % b(2).FaceColor = [32 178 170]./255;
% % b(2).FaceColor = [255 0 0]./255;
% % 
% % 
% % % 添加标签
% % set(gca, 'yticklabel', labels,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% %  
% % % 为了更好的可读性，添加图例
% % legend('Proposed', 'Comparison 1', 'Comparison 2');
% %  
% % % 添加标题和轴标签
% % title('Control performance index','FontName','Times New Roman','FontSize',12,'FontWeight','normal');grid on;
% % xlabel('Index value','FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% %  
% % h=legend(a,'Propsed','Comaprsion');%下方外部居中标注
% % set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% % set(a(1),'facecolor',[0.89, 0.88, 0.57]);
% % 
% % set(a(2),'facecolor',[0.21, 0.33, 0.64]);
% % PARA3(1,1:5000) = 0.608;PARA3(1,5001:10000) = -0.608;
% % figure(5);
% % subplot(4,1,1);
% % plot(T,Q0,'b','LineWidth',2);
% % ylabel('\eta');grid on;hold on;
% % plot(T,QC0,'r','LineWidth',2);
% % plot(T,ccQ0,'g','LineWidth',2);
% % hold on;
% % plot(T,qd(1,:),'k:','LineWidth',2);
% % 
% % subplot(4,1,2);
% % plot(T,Q1,'b','LineWidth',2);
% % ylabel('\epsilon_1');grid on;hold on;
% % plot(T,QC1,'r','LineWidth',2);
% % plot(T,ccQ1,'g','LineWidth',2);
% % hold on; plot(T,qd(2,:),'k:','LineWidth',2);
% % 
% % subplot(4,1,3);
% % plot(T,Q2,'b','LineWidth',2);ylabel('\epsilon_2');grid on;hold on;
% % plot(T,QC2,'r','LineWidth',2);
% % plot(T,ccQ2,'g','LineWidth',2);
% % hold on; 
% % plot(T,qd(3,:),'k:','LineWidth',2);
% % 
% % subplot(4,1,4);
% % plot(T,Q3,'b','LineWidth',2);ylabel('\epsilon_3');grid on;hold on;
% % plot(T,QC3,'r','LineWidth',2);
% % plot(T,ccQ3,'g','LineWidth',2);
% % plot(T,qd(4,:),'k:','LineWidth',2);
% % h=legend(a,'Proposed','Comaprison 1','Comparison 2','Desired value');
% % set(h,'FontName','Times New Roman','FontSize',12,'FontWeight','normal');
% 
% % 假设有两个大组的数据，每个大组有两个小组
% % group1_small1 = [5, 10, 15];
% % group1_small2 = [10, 20, 30];
% % group2_small1 = [15, 25, 35];
% % group2_small2 = [20, 30, 40];
% %  
% % % 创建一个包含所有小组数据的单个数组
% % group1 = [group1_small1; group1_small2];
% % group2 = [group2_small1; group2_small2];
% %  
% % % 创建一个包含小组标签的单个数组
% % groupLabels = {'Group 1 Small 1', 'Group 1 Small 2'; 'Group 2 Small 1', 'Group 2 Small 2'};
% %  
% % % 创建一个包含大组标签的数组
% % groupNames = {'Group 1', 'Group 2'};
% %  
% % % 计算每个小组的宽度以便于在图中对齐
% % width = 0.3;  % 小组之间的宽度可以调整
% %  
% % % 创建柱状图
% % fig = figure;
% % for i = 1:size(group1, 2)
% %     % 绘制第一个大组的柱状图
% %     bar(group1(:, i), (1:length(group1_small1))-width);
% %     % 绘制第二个大组的柱状图
% %     bar(fig, (1:length(group2_small1))+width, group2(:, i));
% % end
% %  
% % % 设置柱状图的标签
% % xlabel('Subgroups');
% % ylabel('Values');
% %  
% % % 设置图例
% % legend('Group 1', 'Group 2');
% %  
% % % 设置每个小组的宽度
% % set(fig, 'DefaultAxesXLim', [0.5, length(group1_small1) + 0.5]);
% % set(fig, 'DefaultAxesXAxisLocation', 'top');
% %  
% % % 设置坐标轴的位置
% % ax = axes('Parent', fig, 'DefaultAxesXAxisLocation', 'top', 'XAxis.TickDir', 'out');
% %  
% % % 设置坐标轴的标签
% % xticks(1:2, groupLabels);
% % xlabel('Groups');
