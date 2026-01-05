% efface tous
clear all;
clf;

% gestion des axes des graphiques
hold on;
axis equal;
grid on;

% parametres a determiner
alpha1=0; alpha2=0; alpha3=0; alpha4=0;

% longueurs & angles
l1=1; l2=0.75; l3=0.5; l4=0.3;

L = [l1 l2 l3 l4];
Theta_true = [pi/8 pi/8 pi/8 pi/8]';

X_true = geodirect(L, Theta_true);

Theta=[0.2+pi/8 0.2+pi/8 0.2-pi/8 0.2+pi/8]';

drawbot(L, Theta, 'Theta0')

J = jacobienne(L, Theta);
Jr = pinv(J);
JrJ = Jr * J
I = eye(size(JrJ));
Alpha = [alpha1 alpha2 alpha3 alpha4]';

x0 = Jr*(geodirect(L, Theta) - X_true)

for i = 1: 10
    Theta = Theta - Jr*(geodirect(L, Theta) - X_true) + (I - Jr * J)*Alpha;
end

drawbot(L, Theta_true, 'ThetaTrue')
drawbot(L, Theta, 'Theta10')

legend