%%%
% Test all methods using randomly generated lines with outliers.
%%%

startup

addpath EGO_PnL functions
%% TEST PARAMETERS
N_LINES = 200; % how many lines to generate
N_OUTLIERS = 0.9*N_LINES;
NOISE_SIGMA = 0.5;
OUTL_SIGMA = 100.0;

if(NOISE_SIGMA >= 1.0)
	RANSAC_REPROJ_ERR_TH = 2e-6 * (log(NOISE_SIGMA) / log(1.5) + 0.5);
else
	RANSAC_REPROJ_ERR_TH = 2e-17;
end
AOR_REPROJ_ERR_TH = NOISE_SIGMA;
RANSAC_P = 0.99;
RANSAC_MAX_ITER = 1e4;

CUBE_SIZE = 10;
CAM_DIST = 25.0;
IMG_SIZE = [480 640];
FOCAL = 800;

%% Check parameter values
if (~(N_LINES >= N_OUTLIERS))
	error('The number of outliers N_OUTLIERS cannot exceed the number of all lines N_LINES.')
end

%% GROUND-TRUTH camera pose (randomly generated)
% camera position
cam_X_GT = 2*rand()-1;
cam_Y_GT = 2*rand()-1;
cam_Z_GT = 2*rand()-1;

norm_coef = CAM_DIST / norm([cam_X_GT cam_Y_GT cam_Z_GT]);

cam_X_GT = norm_coef * cam_X_GT;
cam_Y_GT = norm_coef * cam_Y_GT;
cam_Z_GT = norm_coef * cam_Z_GT;

T_GT = getTranslationVector(cam_X_GT, cam_Y_GT, cam_Z_GT);

% camera orientation
if(rand < 0.5)
	cam_Gamma_GT = -atan2(cam_X_GT, cam_Y_GT);
	cam_Beta_GT = 0;
	cam_Alpha_GT = -acos((cam_Z_GT - 0) / CAM_DIST);
else
	cam_Gamma_GT = atan2(cam_Y_GT, cam_X_GT);
	cam_Beta_GT = acos((cam_Z_GT - 0) / CAM_DIST);
	cam_Alpha_GT = 0;
end

R_GT = getRotationMatrix(cam_Alpha_GT, cam_Beta_GT, cam_Gamma_GT);

% camera intrinsics
principal_x = (IMG_SIZE(2)-1) / 2;
principal_y = (IMG_SIZE(1)-1) / 2;
C  = getCameraMatrix(FOCAL, principal_x, principal_y);

TM_GT = R_GT * [eye(3) T_GT];

%% 3D line segment endpoints
% randomly generated
X_W = [CUBE_SIZE * rand(3, 2*N_LINES) - (CUBE_SIZE/2); ones(1, 2*N_LINES)];

%% 2D line segment endpoints
% Project 3D endpoints onto the normalized image plane using generated
% ground-truth camera pose
x_c = TM_GT * X_W;
for i = 1:3
	x_c(i,:)  = x_c(i,:)  ./ x_c(3,:);
end

% Convert 2D endpoint coordinates from normalized coords. to pixel coords.
x_img = C * x_c;
for i = 1:3
	x_img(i,:) = x_img(i,:) ./ x_img(3,:);
end

% Add gaussian noise to the image endpoints
x_img_noisy = x_img;
x_img_noisy(1:2, :) = x_img_noisy(1:2, :) + (NOISE_SIGMA * randn(2, 2*N_LINES));

% Produce outliers by additional strong perturbation of image line endpoints
x_img_noisy(1:2, end-2*N_OUTLIERS+1:end) = x_img_noisy(1:2, end-2*N_OUTLIERS+1:end) + (OUTL_SIGMA * randn(2, 2*N_OUTLIERS));

x_c_noisy = inv(C) * x_img_noisy;

%% Camera pose estimation with outlier rejection
disp('Ground-truth pose [R|T]');
disp([R_GT T_GT]);
fprintf('\n');


if (N_LINES >= 5)
	disp('EGO_PnL (Ours)');
	[R_EGO, T_EGO, time_EGO] = EGO_PnL(X_W, x_c_noisy, R_GT);
	[ER_EGO, ET_EGO, Ep_EGO] = errors(R_EGO, T_EGO, R_GT, T_GT, X_W);
	disp([R_EGO T_EGO]);
	fprintf('\tOrient.err[°]  Pos.err[m]  Reproj.err[] Time\n');
	fprintf('\t%13f %11f %13.4e %11f \n\n\n', 180*ER_EGO/pi,  ET_EGO, Ep_EGO, time_EGO);
end
