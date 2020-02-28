% Author: Popov Dmitry, Berezhnoy Vladislav
% Innopolis University
% Industrial Robotics
% Homework 2

clear all;
close all;
clc;

% set initial data
% joints limits
upLimit=[deg2rad(-170) -3 -3];
lowLimit=[deg2rad(170) 3 3];

% ideal parameters / measured parameters
idealParams = [1.2 1 1 1 ...
    1 1 0 1 ...
    2 1 1 1 ...
    1 1 1.5 1 ...
    0 0.5 0.5 0.5 0.8]';

% real parameters1
realParams = [1 1.2 1.1 1.3 ...
    1.3 1.3 0.6 1.5 ...
    2 1 0.8 0.8 ...
    0.8 0.5 1.7 1.4 1.9 ...
    0 0.4 0.5 0.4 0.75]';

% real base and tools transformation matrices
TbaseR = [1.2 0 0 0; 0 1.1 0 0; 0 0 1 0-0.7115; 0 0 0 1];
Ttool1R= Tz(90-0.3056);
Ttool2R= Rx(2*pi/3)*Tz(90-0.5056)*Rx(-2*pi/3);
Ttool3R= Rx(-2*pi/3)*Tz(90+0.8625)*Rx(2*pi/3);
% ideal base and tools transformation matrices
Tbase = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ttool1 =Tz(90);
Ttool2 = Rx(2*pi/3)*Tz(90)*Rx(-2*pi/3);
Ttool3 = Rx(-2*pi/3)*Tz(90)*Rx(2*pi/3);

qNum = 100;
q = [];
B=zeros(15,1);

traj1 = IK(rand(3,6), rand(3,6), rand(3,6), 1);
traj2 = IK(rand(3,6), rand(3,6), rand(3,6), 1);
traj3 = IK(rand(3,6), rand(3,6), rand(3,6), 1);

traj1q= [traj1(:,1) traj2(:,1) traj3(:,1)]';
traj2q= [traj1(:,2) traj2(:,2) traj3(:,2)]';
traj3q= [traj1(:,3) traj2(:,3) traj3(:,3)]';

% noise
sigma = 50*1e-3;
% trajectories
% % 
% traj1q = deg2rad([90,45,-150,45, 60, 110;
%     89.8 26.37 -16.2 93, 95, 145;
%     83.51 3 -121.9 90, 45, 50]);
% 
% % traj2q = [90,45,-170,45, 60, 110;
% %     88.73 37.87 -128.6 72.7, 107, 130 ;
% %     83.51 3 -121.9 90, 45, 50;
% %     90,45,-170,45, 65, 130]*pi/180;
% traj2q= [1.5708    0.7854   -2.9671    0.7854    1.0472    1.9199;
%     1.5486    0.6610   -2.2445    1.2689    1.8675    2.2689;
%     1.4575    0.0524   -2.1276    1.5708    0.7854    0.8727;
%     1.5708    0.7854   -2.9671    0.7854    1.1345    2.2689];
% 
% traj3q = [90,45,-170,45, 57, 110;
%     89.32 50.23 -152.9 39.2, 105, 85;
%     86.58 53.87 -170 28.94, 140, 75;
%     90,45,-170,45, 35, 75]*pi/180;

% use random configurations
q = RandomConfig(qNum, lowLimit, upLimit);

% use optimal configurations
%  q = OptimalConfig(testPose, qNum, idealParams, Tbase,Ttool1,Ttool2,Ttool3, lowLimit, upLimit)
% generate experimental data for 30 experiments (10 configurations * 3 points)

for i=1:size(q,1)
    M1(:,:,i) = RobotModelFK(q(i,:),realParams,sigma,TbaseR,Ttool1R);
    M2(:,:,i) = RobotModelFK(q(i,:),realParams,sigma,TbaseR,Ttool2R);
    M3(:,:,i) = RobotModelFK(q(i,:),realParams,sigma,TbaseR,Ttool3R);
end

% find tools, base and delta pi parameters
[p,tb,tt1,tt2,tt3]=FindAllParams(q,sigma, idealParams, Tbase,Ttool1,Ttool2,Ttool3, M1, M2, M3);

save ('results.mat', 'p', 'tb', 'tt1', 'tt2', 'tt3');

% show trajectoies with and without calibration

tr1=Traj2Points(traj1q,realParams, TbaseR);
tr2=Traj2Points(traj1q,idealParams, Tbase);
tr3=Traj2Points(traj1q,idealParams+p, tb);

figure()
hold on
grid on

plot3(tr1(1,:), tr1(2,:), tr1(3,:))
plot3(tr2(1,:), tr2(2,:), tr2(3,:))
plot3(tr3(1,:), tr3(2,:), tr3(3,:))

legend('target trajectory','without calibration','with calibration')

tr1=Traj2Points(traj2q,realParams, TbaseR);
tr2=Traj2Points(traj2q,idealParams, Tbase);
tr3=Traj2Points(traj2q,idealParams+p, tb);

figure()
hold on
grid on
plot3(tr1(1,:), tr1(2,:), tr1(3,:))
plot3(tr2(1,:), tr2(2,:), tr2(3,:))
plot3(tr3(1,:), tr3(2,:), tr3(3,:))
legend('target trajectory','without calibration','with calibration')
tr1=Traj2Points(traj3q,realParams, TbaseR);
tr2=Traj2Points(traj3q,idealParams, Tbase);
tr3=Traj2Points(traj3q,idealParams+p, tb);
figure()
hold on
% grid on
% idxmin = find(tr1(3,:) == max(tr1(3,:)));
% idxmax = find(tr1(3,:) == min(tr1(3,:)));
% pp=
plot3(tr1(1,:), tr1(2,:), tr1(3,:))
% pp.LineWidth = 1.5;
plot3(tr2(1,:), tr2(2,:), tr2(3,:))
plot3(tr3(1,:), tr3(2,:), tr3(3,:))
legend('target trajectory','without calibration','with calibration')


for i=1:size(q,1)
        Mr(:,:,i) = RobotModelFK(q(i,:),realParams,0,TbaseR,eye(4));
        Mc(:,:,i) = RobotModelFK(q(i,:),idealParams,0,Tbase,eye(4));
        Mdist(:,:,i) = Mr(:,:,i) - Mc(:,:,i);
        n(i)=norm(Mdist(1:3,4,i));       
end

disp('without calibration')
acc = mean(n)

for i=1:size(q,1)
        Mr(:,:,i) = RobotModelFK(q(i,:),realParams,0,TbaseR,eye(4));
        Mc(:,:,i) = RobotModelFK(q(i,:),idealParams + p ,0,tb,eye(4));
        Mdist(:,:,i) = Mr(:,:,i) - Mc(:,:,i);
        n(i)=norm(Mdist(1:3,4,i));       
end

disp('with calibration')
acc = mean(n)