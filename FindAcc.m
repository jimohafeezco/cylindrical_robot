% Author: Hafeez Jimoh
% Innopolis University
% Industrial Robotics
% Homework 2

function out = FindAcc(q, params, sigma, Tbase,Ttool1,Ttool2,Ttool3)
J1 = zeros(9,9);
for i=1:size(q,1)
J1=J1 + Jparams(q(i,:),params,Tbase,Ttool1)'*Jparams(q(i,:),params,Tbase,Ttool1)+...
    Jparams(q(i,:),params,Tbase,Ttool2)'*Jparams(q(i,:),params,Tbase,Ttool2)+...
    Jparams(q(i,:),params,Tbase,Ttool3)'*Jparams(q(i,:),params,Tbase,Ttool3);
end

J1=inv(J1)
out = sigma^2*J1;

end