function [t_opt] = get_trans_outlier(data_2d_N,data_3d_c,R_opt)
%GET_TRANS 此处显示有关此函数的摘要
%   此处显示详细说明

data_3d_c_rot=R_opt*data_3d_c;
K=-dot(data_2d_N,data_3d_c_rot);
t_opt = get_robust_trans(data_2d_N',K');


end