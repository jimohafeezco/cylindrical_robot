% Author: Popov Dmitry, Berezhnoy Vladislav
% Innopolis University
% Industrial Robotics
% Homework 2
function out = FindParams(q, params, deltaDist, Tbase,Ttool1,Ttool2,Ttool3)
J1 = zeros(21,21);
J2 = zeros(21,1);
for i=1:size(q,1)
J1=J1 + Jparams(q(i,:),params,Tbase,Ttool1)'*Jparams(q(i,:),params,Tbase,Ttool1)+...
    Jparams(q(i,:),params,Tbase,Ttool2)'*Jparams(q(i,:),params,Tbase,Ttool2)+...
    Jparams(q(i,:),params,Tbase,Ttool3)'*Jparams(q(i,:),params,Tbase,Ttool3);
J2=J2 + Jparams(q(i,:),params,Tbase,Ttool1)'*deltaDist(:,1,i)+...
    Jparams(q(i,:),params,Tbase,Ttool2)'*deltaDist(:,2,i)+...
    Jparams(q(i,:),params,Tbase,Ttool3)'*deltaDist(:,3,i);
end
J1=pinv(J1);
out = J1*J2;
end