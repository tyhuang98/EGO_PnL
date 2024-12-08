function [R_opt] = Rotation_BnB(X,Y,epsilon, R_gt)

min_res = 0.03;

r_angle_best=zeros(2,1);
B=[0;0;pi;2*pi];

best_branch=B;

branch=[];
new_upper=zeros(1,4);
new_lower=zeros(1,4);

best_lower=0;

iter=1;
% figure
% h1=animatedline;
% h2=animatedline;


while true

    new_branch=subBranch(best_branch);

    for i=1:4
          [new_upper(i),new_lower(i)]=get_bounds(X,Y,new_branch(:,i),epsilon);
    end
    
    branch=[branch,[new_branch;new_upper;new_lower]];
    
    [best_upper,ind_upper]=max(branch(5,:));
    [new_best_lower,ind_lower]=max(branch(6,:));
    
    if(best_lower<new_best_lower)
        best_lower=new_best_lower;
        r_branch=branch(1:4,ind_lower);
        r_angle_best=0.5*(r_branch(1:2)+r_branch(3:4));
    end
    
    best_branch=branch(1:4,ind_upper);
    
    branch(:,ind_upper)=[];
    branch(:,branch(5,:)<best_lower)=[];
    
    if(best_upper<=best_lower)
        break;  
    end

    if(new_branch(3,1) - new_branch(1,1) < min_res)
        break;
    end

%     addpoints(h1,iter,best_upper);
%     addpoints(h2,iter,best_lower);

%     fprintf(bound_EGO_file, '%d  %d\n', best_upper, best_lower);

    drawnow
    iter=iter+1;
  
end


alpha = r_angle_best(1);
beta = r_angle_best(2);



% axang_gt = rotm2axang(R_gt);
% beta = acos(axang_gt(3));
% alpha = 0;
% if axang_gt(2) >= 0
%     alpha = acos(axang_gt(1)/sin(beta));
% else
%     alpha = - acos(axang_gt(1)/sin(beta));
% end


r_opt=[sin(beta)*cos(alpha); sin(beta)*sin(alpha);cos(beta)];

[A, phi, c] = get_parameter(X, Y, [alpha;beta]);
intervals = [];
for i=1:size(X,2)
    % interval for lower bound
    [interval_i] = get_interval(A(i), phi(i), c(i), epsilon);
    intervals = [intervals; interval_i];
end
[~, theta_opt] = interval_stabbing(intervals);


R_opt = rotation_from_axis_angle(r_opt, theta_opt);

end

