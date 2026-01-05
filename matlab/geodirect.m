function X = geodirect(L, Theta)
% computeX  Returns the 3×1 vector X for a 4-link planar robot
%
%   X = computeX(L, Theta)
%
%   L      - 1×4 vector of link lengths [l1 l2 l3 l4]
%   Theta  - 1×4 vector of joint angles [theta1 theta2 theta3 theta4]
%
%   Returns:
%       X = [
%           l1*cos(theta1) + l2*cos(theta1+theta2) + l3*cos(theta1+theta2+theta3) + l4*cos(theta1+theta2+theta3+theta4);
%           l1*sin(theta1) + l2*sin(theta1+theta2) + l3*sin(theta1+theta2+theta3) + l4*sin(theta1+theta2+theta3+theta4);
%           theta1 + theta2 + theta3 + thetha4
%       ]

% Extract link lengths
l1=L(1); 
l2=L(2); 
l3=L(3);
l4=L(4);

% Extract angles
theta1=Theta(1);
theta2=Theta(2);
theta3=Theta(3);
theta4=Theta(4);


X = [
       l1*cos(theta1)+l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3)+l4*cos(theta1+theta2+theta3+theta4);
       l1*sin(theta1)+l2*sin(theta1+theta2)+l3*sin(theta1+theta2+theta3)+l4*sin(theta1+theta2+theta3+theta4)
   ];
end
