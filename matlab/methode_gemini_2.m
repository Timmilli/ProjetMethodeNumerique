% Stabilized 4-Link Redundant Arm with DLS
clear; clc; close all;

% 1. Parameters
L = [1, 1, 1, 1]; 
q = [0.5; -0.2; 0.1; 0.1]; % Better initial pose
target = [0.2; 2.5];
obs_pos = [1.5; 1.2];
obs_rad = 0.4;
dt = 0.02; 
max_iter = 500;

% --- Stability Gains ---
lambda = 0.1;      % Damping factor (Prevents singularity explosions)
k_task = 2.0;      % Primary task gain
k_null = 1.5;      % Alpha strength
max_dq = 0.5;      % Velocity clamping (Physical speed limit)

figure('Color', 'w'); hold on; grid on; axis equal; xlim([-1 4]); ylim([-1 4]);

% 2. Simulation Loop
for i = 1:max_iter
    % Forward Kinematics
    p0 = [0;0];
    p1 = p0 + L(1)*[cos(q(1)); sin(q(1))];
    p2 = p1 + L(2)*[cos(q(1)+q(2)); sin(q(1)+q(2))];
    p3 = p2 + L(3)*[cos(q(1)+q(2)+q(3)); sin(q(1)+q(2)+q(3))];
    p4 = p3 + L(4)*[cos(sum(q)); sin(sum(q))];
    joint_pos = [p0, p1, p2, p3, p4];

    % --- 1. Primary Task (End-Effector) ---
    error = target - p4;
    x_dot = k_task * error; 

    % Jacobian
    J = [-L(1)*sin(q(1))-L(2)*sin(q(1)+q(2))-L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(2)*sin(q(1)+q(2))-L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(4)*sin(sum(q));
          L(1)*cos(q(1))+L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(4)*cos(sum(q))];

    % --- Damped Least Squares Inverse ---
    % J_dls = J' * inv(J*J' + lambda^2 * I)
    J_dls = J' / (J*J' + lambda^2 * eye(2));

    % --- 2. Secondary Task (Alpha Optimization) ---
    alpha = zeros(4,1);
    eps = 0.01;
    for j = 1:4
        H_curr = compute_cost(q, L, obs_pos, obs_rad);
        q_p = q; q_p(j) = q_p(j) + eps;
        H_pert = compute_cost(q_p, L, obs_pos, obs_rad);
        alpha(j) = -k_null * (H_pert - H_curr) / eps;
    end

    % --- 3. Combined Motion ---
    dq = J_dls * x_dot + (eye(4) - J_dls * J) * alpha;

    % --- Velocity Clamping (Safety) ---
    if norm(dq) > max_dq
        dq = dq * (max_dq / norm(dq));
    end

    q = q + dq * dt;

    % Visualization
    cla;
    rectangle('Position',[obs_pos(1)-obs_rad, obs_pos(2)-obs_rad, 2*obs_rad, 2*obs_rad],...
              'Curvature',[1,1],'FaceColor',[1 0 0 0.2]);
    plot(target(1), target(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    plot(joint_pos(1,:), joint_pos(2,:), '-ob', 'LineWidth', 3);
    drawnow;
    
    if norm(error) < 0.05, break; end
end

function H = compute_cost(q, L, op, or)
    p1 = L(1)*[cos(q(1)); sin(q(1))];
    p2 = p1 + L(2)*[cos(q(1)+q(2)); sin(q(1)+q(2))];
    p3 = p2 + L(3)*[cos(q(1)+q(2)+q(3)); sin(q(1)+q(2)+q(3))];
    p4 = p3 + L(4)*[cos(sum(q)); sin(sum(q))];
    % Check all joints against obstacle
    dists = [norm(p1-op), norm(p2-op), norm(p3-op), norm(p4-op)];
    H = sum(1 ./ (max(0.1, dists - or)).^2);
end