function [data_2d_v,data_2d_c,data_3d_v,data_3d_c] = get_v_c(point_3d,point_2d)
%
point_2D_a=point_2d(:,1:2:end);
point_2D_b=point_2d(:,2:2:end);

point_3D_a=point_3d(1:3,1:2:end);
point_3D_b=point_3d(1:3,2:2:end);

data_2d_v_non=point_2D_a-point_2D_b;
data_3d_v_non=point_3D_a-point_3D_b;

data_2d_v=data_2d_v_non./vecnorm(data_2d_v_non);
data_3d_v=data_3d_v_non./vecnorm(data_3d_v_non);

data_2d_c=0.5*(point_2D_a+point_2D_b);
data_3d_c=0.5*(point_3D_a+point_3D_b);
end
