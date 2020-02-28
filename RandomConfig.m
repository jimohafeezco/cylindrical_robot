function [q] = RandomConfig(qNum, lowLimit, upLimit)

for i = 1:qNum
    q(i,:) = [rand(1)*(-lowLimit(1)+upLimit(1)) + lowLimit(1),...
        rand(1)*(-lowLimit(2)+upLimit(2)) + lowLimit(2),...
        rand(1)*(-lowLimit(3)+upLimit(3)) + lowLimit(3)];

end