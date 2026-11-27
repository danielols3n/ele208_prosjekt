function T = kinematikk9(q)
% Regner ut stillingen til dysen fra gruppens DH-tabell.
% q er en radvektor med liftens tre ledd, armfestet og fem armledd.
if numel(q) ~= 9
    error('Forventet ni leddkoordinater.');
end
q = double(q(:))';
theta = [q(1), pi/2-q(2), 0, q(4)-pi/2, q(5)-pi, q(6:9)];
d = [0, 0, 3.15+6*q(3), 0, .089159, 0, 0, .10915, .09465];
a = [0, 0, 0, 0, 0, -.425, -.39225, 0, 0];
alpha = [pi/2, pi/2, pi/2, -pi/2, pi/2, 0, 0, -pi/2, pi/2];
T = eye(4);
T(1:3,4) = [-3; -0.03; 1.5]; % rotasjonsaksen (= full.base i MATLAB, flyttet 1,97 m i y)
% Jeg fikk kodeforslag/veildening fra KI for DH-beregningen og hjelp til å
% sammenligne fortegn og rekkefølge i matrisene med Xacro-modellen.
for k = 1:9
    c = cos(theta(k)); s = sin(theta(k));
    ca = cos(alpha(k)); sa = sin(alpha(k));
    A = [c -s*ca s*sa a(k)*c; ...
         s  c*ca -c*sa a(k)*s; ...
         0  sa ca d(k); 0 0 0 1];
    T = T*A;
end
% Fast dysefeste. Sjette armledd og L6 er fjernet.
rz = -3*pi/2; ry = -pi/2;
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
R = Rz*Ry;
verktoy = eye(4);
verktoy(1:3,1:3) = R;
verktoy(1:3,4) = R*[.2323;0;0]; % dysespissen ligger 0,0823 + 0,15 m fra siste armledd (som full.tool)
T = T*verktoy;
end
