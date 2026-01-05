% 4-Link Redundant Arm with Null-Space Obstacle Avoidance
clear; clc; close all;

% 1. Parameters
L = [1, 1, 1, 1]; 
q = [1.0; -0.0; -0.0; -0.2]; % Initial pose
target = [3.0; 1.0];
obs_pos = [1.5; 1.2];
obs_rad = 0.4;
dt = 0.05; 
max_iter = 300;
k_null = 5; % Gain for obstacle avoidance (Alpha strength)

figure('Color', 'w'); hold on; grid on; axis equal;
xlim([-1 4]); ylim([-1 4]);

% 2. Simulation Loop
for i = 1:max_iter
    % Forward Kinematics
    p0 = [0;0];
    p1 = p0 + L(1)*[cos(q(1)); sin(q(1))];
    p2 = p1 + L(2)*[cos(q(1)+q(2)); sin(q(1)+q(2))];
    p3 = p2 + L(3)*[cos(q(1)+q(2)+q(3)); sin(q(1)+q(2)+q(3))];
    p4 = p3 + L(4)*[cos(sum(q)); sin(sum(q))];
    joint_pos = [p0, p1, p2, p3, p4];

    % --- 1. Primary Task: Move to Target ---
    error = target - p4;
    x_dot = 0.5 * error; % Desired velocity of end-effector

    % Jacobian Calculation
    J = [-L(1)*sin(q(1))-L(2)*sin(q(1)+q(2))-L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(2)*sin(q(1)+q(2))-L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(3)*sin(q(1)+q(2)+q(3))-L(4)*sin(sum(q)), ...
         -L(4)*sin(sum(q));
          L(1)*cos(q(1))+L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(2)*cos(q(1)+q(2))+L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(3)*cos(q(1)+q(2)+q(3))+L(4)*cos(sum(q)), ...
          L(4)*cos(sum(q))];

    J_pinv = pinv(J);

    % --- 2. Secondary Task: Optimize Alpha (Obstacle Avoidance) ---
    % Define Alpha as the gradient of a distance-based cost function
    alpha = zeros(4,1);
    eps = 0.01; % Small perturbation for numerical gradient
    
    for j = 1:4
        % Calculate cost H at current q
        H_current = compute_cost(q, L, obs_pos, obs_rad);
        
        % Perturb joint j
        q_pert = q;
        q_pert(j) = q_pert(j) + eps;
        H_pert = compute_cost(q_pert, L, obs_pos, obs_rad);
        
        % Numerical gradient (Negative because we want to minimize cost)
        alpha(j) = -k_null * (H_pert - H_current) / eps;
    end

    % --- 3. Combine using Redundancy Resolution Equation ---
    I = eye(4);
    q_dot = J_pinv * x_dot + (I - J_pinv * J) * alpha;

    % Update joints
    q = q + q_dot * dt;

    % --- Visualization ---
    cla;
    rectangle('Position',[obs_pos(1)-obs_rad, obs_pos(2)-obs_rad, 2*obs_rad, 2*obs_rad],...
              'Curvature',[1,1],'FaceColor',[1 0.2 0.2 0.3]);
    plot(target(1), target(2), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    plot(joint_pos(1,:), joint_pos(2,:), '-ok', 'LineWidth', 4, 'MarkerFaceColor', 'b');
    pause(0.01);
    
    if norm(error) < 0.02, break; end
end

% Cost Function Helper
function H = compute_cost(q, L, obs_pos, obs_rad)
    % Find joint positions for this specific q
    p1 = L(1)*[cos(q(1)); sin(q(1))];
    p2 = p1 + L(2)*[cos(q(1)+q(2)); sin(q(1)+q(2))];
    p3 = p2 + L(3)*[cos(q(1)+q(2)+q(3)); sin(q(1)+q(2)+q(3))];
    p4 = p3 + L(4)*[cos(sum(q)); sin(sum(q))];
    
    joints = [p1, p2, p3, p4];
    dist_sq = sum((joints - obs_pos).^2, 1);
    dists = sqrt(dist_sq);
    
    % Cost is high when distance is low
    H = sum(1 ./ (dists - obs_rad + 0.1).^2); 
end