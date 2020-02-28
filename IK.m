function Q = IK(px,py,pz, l1)

%     l1 = robot.links(1,1);
%     q1 = atan2(py,px);
%     q2 = pz - l1;
%     q3 = sqrt(px^2+py^2);
    q1traj=zeros(length(px),1);
    q2traj=zeros(length(px),1);     
    q3traj=zeros(length(px),1);
   
    for i = 1: length(px)
        q1traj(i) = atan2(py(i), px(i))   ;
        q2traj(i) = pz(i)-l1;
        q3traj(i)= sqrt(px(i)^2+py(i)^2);
      
    end    
            
    Q = [q1traj, q2traj, q3traj];
    
end