function [Q_upper,Q_lower]= get_bounds(Point_M,Point_N,B,epsilon)
% Point_M: v
% Point_N: n

num = size(Point_M,2);

B_l = B(1:2);
B_c = 0.5*(B(1:2) + B(3:4));
B_u = B(3:4);

[A_l, phi_l, c_l] = get_parameter(Point_M, Point_N, B_l);
[A_c, phi_c, c_c] = get_parameter(Point_M, Point_N, B_c);
[A_u, phi_u, c_u] = get_parameter(Point_M, Point_N, B_u);


intervals_lower = [];
intervals_upper = [];

for i=1:num
    % interval for lower bound
    [interval_lower_i] = get_interval(A_c(i), phi_c(i), c_c(i), epsilon);
    intervals_lower = [intervals_lower; interval_lower_i];

    % new threshold for upper bound
    [min_A_i,max_A_i] = bounds([A_u(i), A_c(i), A_l(i)]);
    [min_sinphi_i,max_sinphi_i] = bounds([sin(phi_u(i)), sin(phi_c(i)), sin(phi_l(i))]);
    [min_c_i, max_c_i] = bounds([c_u(i), c_c(i), c_l(i)]);

    epsilon_new_i = epsilon + 0.5*((max_A_i-min_A_i)*(max_sinphi_i-min_sinphi_i) + (max_c_i - min_c_i)) + 0.3 * (B_u(1)-B_l(1));
    [interval_upper_i] = get_interval(A_c(i), phi_c(i), c_c(i), epsilon_new_i);
    intervals_upper = [intervals_upper; interval_upper_i];
end

[Q_lower, ~] = interval_stabbing(intervals_lower);
[Q_upper, ~] = interval_stabbing(intervals_upper);


end
