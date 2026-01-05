function drawCircle(center, radius, color)
    theta = linspace(0, 2*pi, 50);         % 50 points around circle
    x = center(1) + radius*cos(theta);     % x coordinates
    y = center(2) + radius*sin(theta);     % y coordinates
    plot(x, y, 'Color', color, 'LineWidth', 2);
end