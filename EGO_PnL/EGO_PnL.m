function [R_opt,t_opt,tim] = EGO_PnL(X_W,x_c, R_gt)

[data_2d_v,data_2d_c,data_3d_v,data_3d_c] = get_v_c(X_W,x_c);
epsilon=0.0175*1;
[R_opt,t_opt,tim]=solve_RT(data_2d_v,data_2d_c,data_3d_v,data_3d_c,X_W,x_c,epsilon, R_gt);

end
