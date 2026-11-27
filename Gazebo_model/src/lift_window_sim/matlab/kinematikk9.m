function T = kinematikk9(q)
% Regner ut stillingen til dysen fra gruppens DH-tabell (samme som kinematics_model.m).
% q er en radvektor med Gazebo-leddene: boom_yaw, boom_pitch, boom_extension,
% arm_swivel og arm_joint_1..5. Leddvinklene er de samme som i MATLAB-modellen,
% bortsett fra boom_extension, som er uttrekket til hvert av de fire
% teleskoptrinnene: d3 = 3.6 + 4*boom_extension.
% Dysens akse (strålen) er z-aksen til T, dvs. T(1:3,3).
if numel(q) ~= 9
    error('Forventet ni leddkoordinater.');
end
q = double(q(:))';
theta = [q(1), q(2), 0, q(4), q(5:9)];
d     = [0, 0, 3.6 + 4*q(3), 0, 0.089159, 0, 0, 0.10915, 0.09465];
a     = [0, 0, 0, 0, 0, -0.425, -0.39225, 0, 0];
alpha = [pi/2, pi/2, pi/2, -pi/2, pi/2, 0, 0, -pi/2, pi/2];
T = eye(4);
T(1:3,4) = [-3; -2; 1.5];                % liftbasen i verdenskoordinater, som i kinematics_model.m
for k = 1:9
    c = cos(theta(k)); s = sin(theta(k));
    ca = cos(alpha(k)); sa = sin(alpha(k));
    A = [c -s*ca s*sa a(k)*c; ...
         s  c*ca -c*sa a(k)*s; ...
         0  sa ca d(k); 0 0 0 1];
    T = T*A;
end
verktoy = eye(4);
verktoy(3,4) = 0.0823 + 0.15;            % UR5-flens + dyse langs z9
T = T*verktoy;
end
