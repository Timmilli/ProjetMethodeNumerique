% efface tous
clear all;
clf;

% gestion des axes des graphiques
hold on;
axis equal;
grid on;

% longueurs & angles
l1=1; l2=0.75; l3=0.5; l4=0.3;

L = [l1 l2 l3 l4];
Theta_true = [pi/8 pi/8 pi/8 pi/8]';

X_true = geodirect(L, Theta_true);

Theta=[0.2+pi/8 0.2+pi/8 0.2-pi/8 0.2+pi/8]';

drawbot(L, Theta, 'Theta0')

for i = 1: 10
    Theta = Theta-pinv(jacobienne(L, Theta))*(geodirect(L, Theta) - X_true);
    %drawbot(L, Theta, int2str(i))
end

drawbot(L, Theta_true, 'ThetaTrue')
drawbot(L, Theta, 'Theta10')

legend