% Author: Popov Dmitry, Berezhnoy Vladislav
% Innopolis University
% Industrial Robotics
% Homework 2
function out = paramPerfomance(q, params, pbase,rbase, ptool1,ptool2,ptool3, m1, m2, m3)
S= 0;
for i=1:size(q,1)
fk = RobotModelFK(q(i,:),params,0,eye(4),eye(4)); 
probot = [eye(3) fk(1:3,4); 0 0 0 1];
rrobot = [fk(1:3,1:3) zeros(3,1); 0 0 0 1];
S=S+norm((m1(:,:,i)-[eye(3) pbase; 0 0 0 1] - [rbase eye(3,1); 0 0 0 1] * probot - [rbase eye(3,1); 0 0 0 1] * rrobot * ptool1),2)+...
    norm((m2(:,:,i)-[eye(3) pbase; 0 0 0 1] - [rbase eye(3,1); 0 0 0 1] * probot - [rbase eye(3,1); 0 0 0 1] * rrobot * ptool2),2)+...
    norm((m3(:,:,i)-[eye(3) pbase; 0 0 0 1] - [rbase eye(3,1); 0 0 0 1] * probot - [rbase eye(3,1); 0 0 0 1] * rrobot * ptool3),2);
end
out = S;
end