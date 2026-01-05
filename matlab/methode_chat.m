clc; clear; close all;

%% --- PARAMETERS ---
L = [1 0.8 0.6 0.4];       % lengths of the 4 links
theta = [0 0 0 0];          % initial joint angles
n_links = length(L);

dt = 0.1;                   % simulation time step
steps = 500;                % number of steps
alpha_goal = 0.1;           % gain toward goal
alpha_obs = 0.05;           % repulsion gain from obstacles
obs_threshold = 0.3;        % distance threshold to start avoiding

% Goal position
goal = [2, 1];

% Obstacles: [x, y, radius]
obstacles = [1.5 0.5 0.3;
             2 0.8 0.2];

%% --- FUNCTION TO DRAW CIRCLES (WITHOUT TOOLBOX) ---
function drawCircle(center, radius, color)
    theta_c = linspace(0,2*pi,50);
    x = center(1) + radius*cos(theta_c);
    y = center(2) + radius*sin(theta_c);
    plot(x, y, 'Color', color, 'LineWidth', 2);
end

%% --- SIMULATION LOOP ---
figure; axis equal; xlim([-3 3]); ylim([-3 3]); hold on;

for t = 1:steps
    % --- 1. Forward kinematics: compute joint positions ---
    joints = zeros(n_links+1,2); % include base at (0,0)
    for i = 1:n_links
        angle_sum = sum(theta(1:i));
        joints(i+1,1) = joints(i,1) + L(i)*cos(angle_sum);
        joints(i+1,2) = joints(i,2) + L(i)*sin(angle_sum);
    end
    
    % --- 2. Plot arm and obstacles ---
    clf; hold on; axis equal; xlim([-3 3]); ylim([-3 3]);
    plot(joints(:,1), joints(:,2), '-o','LineWidth',3,'MarkerSize',6,'MarkerFaceColor','r');
    plot(goal(1), goal(2), 'g*', 'MarkerSize', 12);
    
    for k = 1:size(obstacles,1)
        drawCircle(obstacles(k,1:2), obstacles(k,3), 'b');
    end
    
    % --- 3. Compute forces ---
    
    % 3a. Attractive force toward goal (end-effector)
    ee = joints(end,:);
    F_goal = goal - ee;  % 1x2 vector
    F_goal = F_goal * alpha_goal;
    
    % 3b. Repulsive forces from obstacles
    F_obs = zeros(n_links+1,2); % 1x2 for each joint
    for j = 2:n_links+1 % skip base
        for k = 1:size(obstacles,1)
            vec = joints(j,:) - obstacles(k,1:2);
            dist = norm(vec);
            if dist < obs_threshold
                % Potential field repulsion
                F_obs(j,:) = F_obs(j,:) + alpha_obs*(1/dist - 1/obs_threshold)*(vec/dist)/dist^2;
            end
        end
    end
    
    % --- 4. Compute Jacobian ---
    J = zeros(2, n_links); % 2x4
    for i = 1:n_links
        for j = i:n_links
            r = joints(end,:) - joints(j,:);
            J(:,i) = J(:,i) + [-r(2); r(1)];
        end
    end
    
    % --- 5. Joint angle update ---
    total_force = F_goal + sum(F_obs,1)';  % 2x1
    dtheta = J' * total_force;             % n_links x 1
    theta = theta + (dtheta')*dt;          % row vector 1x4
    
    pause(0.05);
end
