function [modelRANSAC,inlierIdx] = get_robust_trans(A,y)
%GET_ROBUST_TRANS 此处显示有关此函数的摘要
%   此处显示详细说明


sampleSize = 3; % number of points to sample per trial
maxDistance = 0.05; % max allowable distance for inliers

fitLineFcn = @(data) data(:,1:3)\data(:,4); % fit function using polyfit
evalLineFcn =@(model, data) (data(:,1:3)*model-data(:,4)).^2;

[modelRANSAC, inlierIdx] = ransac([A,y],fitLineFcn,evalLineFcn,sampleSize,maxDistance);


end



