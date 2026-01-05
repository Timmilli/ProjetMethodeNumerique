function drawbot(L, A, name, basePos)
% drawBot Draws a 4-link planar robot in 2D
%
%   drawBot(L, A)
%   drawBot(L, A, basePos)
%
%   L        - 1x4 vector of link lengths
%   A        - 1x4 vector of joint angles (in radians)
%   name     - name to display in legend
%   basePos  - optional 1x2 [x0, y0] position of the base (default = [0 0])
%
% Example:
%   L = [1 0.8 0.6 0.4];
%   A = [pi/6 pi/4 -pi/3 pi/8];
%   drawBot(L, A);

    if nargin < 4
        basePos = [0, 0];
    end

    % Check input
    if numel(L) ~= 4 || numel(A) ~= 4
        error('L and A must be 1x4 vectors.');
    end

    % Initialize joint coordinates
    x = zeros(1, 5);
    y = zeros(1, 5);

    % Base location
    x(1) = basePos(1);
    y(1) = basePos(2);

    % Compute joint positions
    angleSum = 0;
    for i = 1:4
        angleSum = angleSum + A(i);
        x(i+1) = x(i) + L(i)*cos(angleSum);
        y(i+1) = y(i) + L(i)*sin(angleSum);
    end

    % Plot
    plot(x, y, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', name);
    xlabel('X');
    ylabel('Y');

    % Adjust limits
    maxLen = sum(L);
    xlim([basePos(1)-maxLen, basePos(1)+maxLen]);
    ylim([basePos(2)-maxLen, basePos(2)+maxLen]);
end
