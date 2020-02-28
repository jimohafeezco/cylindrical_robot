% Author: Popov Dmitry, Berezhnoy Vladislav
% Innopolis University
% Industrial Robotics
% Homework 2
function out = RobotModelFK(q,params,sigma,Tbase,Ttool)
T = Tbase*Rz(q(1))*...
    Tz(params(1))*Tz(q(2))*Tz(params(3))*Tx(q(3))*Tz(params(4))*Ttool;

T(1:3,4) = T(1:3,4)+sigma*randn(3,1);
out = T; %[T(1:3,4)+sigma*randn(3,1)];
end