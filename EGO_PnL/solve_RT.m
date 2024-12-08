function [R_final,t_final,tim]=solve_RT(data_2d_v,data_2d_c,data_3d_v,data_3d_c,X_W,x_c,epsilon, R_gt)
%
data_2d_N_non=cross(data_2d_v,data_2d_c);
data_2d_N=data_2d_N_non./vecnorm(data_2d_N_non);
tic
[R_opt]=Rotation_BnB(data_3d_v,data_2d_N,epsilon, R_gt);

index_inlier=abs(dot(R_opt*data_3d_v,data_2d_N))<=epsilon;

% n = norm(R_gt - R_opt, 'fro');
% theta = 2*asin(n/(2*sqrt(2)));
% R_error = theta* 180 / pi;
% fprintf('\t Rotation error: %13f\n\n\n', R_error);

% tim_R = toc;
% fprintf('RotationTime: %10f\n\n', tim_R);
% 
% tic


% gt_inliers = true(1, size(data_3d_v,2));
% gt_inliers(1, end-N_OUTLIERS+1:end) = false(1, N_OUTLIERS);
% 
% 
% gt_inliers_num = sum(gt_inliers);
% method_inliers_num = sum(index_inlier);
% 
% method_gt_inliers = gt_inliers & index_inlier;
% num_method_gt_inliers = sum(method_gt_inliers);
% 
% if method_inliers_num == 0
%     precision = double(num_method_gt_inliers) / 1e6;   %inlier precision
% else
%     precision = double(num_method_gt_inliers) / double(method_inliers_num);
% end
% recall = double(num_method_gt_inliers) / double(gt_inliers_num);  %inlier recall
% 
% if precision == 0 || recall == 0
%     F1 = 0;                                      %F1 score
% else
%     F1 = 2 / (1/recall + 1/precision);


[t_opt] = get_trans_outlier(data_2d_N(:,index_inlier),data_3d_c(:,index_inlier),R_opt);
t_opt = R_opt'*t_opt;

tim = toc;

t_opt_repeat = repmat(t_opt, 1, size(data_3d_v,2));
data_3d_c_trans = R_opt*(data_3d_c + t_opt_repeat);
m = abs(dot(data_3d_c_trans./vecnorm(data_3d_c_trans), data_2d_N));

index_inlier_final = index_inlier & (m<=epsilon);
is_inlier_final = reshape([index_inlier_final', index_inlier_final']', 2*length(index_inlier_final), 1);

% tim_t = toc;
% fprintf('Translation Time: %10f\n\n', tim_t);


[R_final,t_final] = DLT_Combined_Lines(X_W(:, is_inlier_final), x_c(:, is_inlier_final));

% tim=tim_R + tim_t;
% tim = toc;

end