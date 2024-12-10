function [p, v] = generate_Motion_Profile(final_pos, initial_pos, v0, v1, init_time, final_time, timesteps)
%final_pos = [20, 0, 0];
%initial_pos = [0, 0, 0];
%v0 = [1, 0, 0];
%v1 = [2, 0, 0];
%init_time = 0;
%final_time = 9;
%timesteps = 90;
delta = final_pos - initial_pos;
numE = timesteps;
dt = (final_time - init_time)/numE;
a0_end = floor(numE / 3.0);
a1_start = floor(numE * (2.0 / 3.0));
a0_rat = a0_end/(numE - 1);
a1_rat = a1_start/(numE - 1);

t = final_time - init_time;
v_max = (2 * delta + v1*t*a1_rat - v1 * t - v0 * t * a0_rat)/(t * (a1_rat - a0_rat + 1)); 
t_mat = zeros([3 numE]);
v_max_mat = zeros([3 numE]);
v0_mat = zeros([3 numE]);
v1_mat = zeros([3 numE]);
for i = 1:1:numE
    v_max_mat(:, i) = v_max;
    v0_mat(:, i) = v0;
    v1_mat(:, i) = v1;
end
t_mat(1, :) = linspace(0, t, numE);
t_mat(2, :) = linspace(0, t, numE);
t_mat(3, :) = linspace(0, t, numE);
v = zeros([3 numE]);
p = zeros([3 numE]);
p(:, 1) = initial_pos;
v(:, 1:a0_end + 1) = v0_mat(:, 1:a0_end + 1) + (v_max_mat(:, 1:a0_end + 1) - v0_mat(:, 1:a0_end + 1)) .* t_mat(:, 1:a0_end + 1)/(a0_rat * t);
v(:, a0_end + 1:a1_start) = v_max_mat(:, a0_end + 1:a1_start);
v(:, a1_start + 1:numE) = (v_max_mat(:, a1_start + 1:numE) - ((v_max_mat(:, a1_start + 1:numE) - v1_mat(:, a1_start + 1:numE))/((1 - a1_rat) * t)) .* (t_mat(:, a1_start + 1:numE) - a1_rat * t));


p(:, 1:a0_end + 1) = p(:, 1)+ v0_mat(:, 1:a0_end + 1) .* t_mat(:, 1:a0_end + 1) + (v_max_mat(:, 1:a0_end + 1) - v0_mat(:, 1:a0_end + 1)) .* (t_mat(:, 1:a0_end + 1) .^ 2)/(2 * a0_rat * t);
p(:, a0_end + 1:a1_start + 1) = p(:, a0_end + 1) + v_max_mat(:, a0_end + 1:a1_start + 1) .* (t_mat(:, a0_end + 1:a1_start + 1) - t_mat(1, a0_end + 1));
t0 = t_mat(:, a1_start + 1);
p(:, a1_start + 1:numE) = p(:, a1_start + 1) + (v_max_mat(:, a1_start + 1:numE) .* (t_mat(:, a1_start + 1:numE) - t0)) + (v_max_mat(:, a1_start + 1:numE) - v1_mat(:, a1_start + 1:numE)) .* (a1_rat * t * (t_mat(:, a1_start + 1:numE) - t0)) / (t - a1_rat * t) - ((v_max_mat(:, a1_start + 1:numE) - v1_mat(:, a1_start + 1:numE))/((1 - a1_rat) * t)) .* ((((t_mat(:, a1_start + 1:numE) .^ 2) - (a1_rat * t) ^ 2) / 2.0) );
v = v';
p = p';

% plotting code
% figure(1)
% plot(t_mat, v(:, 1), "r-");
% grid on
% hold on
% plot(t_mat, p(:, 1), "g-");
% 
% xlabel("time (s)");
% ylabel("X Position")
% legend("velocity", "Pos", 'location', "northwest")
