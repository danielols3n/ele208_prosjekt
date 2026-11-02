%% Clear the workspace 
clear;
clc;
close all force;

%% Konstanter
% Geometri
L_min = 3.6;                    % innskjøvet bomlengde [m]
L_max = 15;                     % utskjøvet bomlengde [m]
L_dyse = 0.15;                  % lengde på dysen [m]
base_pos = [0 -2 1.5];          % liftbasen i verdenskoordinater [m]
h_c = 0.3;                      % kameraets avstand under origo i frame 4 [m]

% Dynamiske parametere, lift (verdier fra utregning)
s = 3.6;                        % seksjonslengde [m]
m1 = 40;                        % dreiekrans (anslag) [kg]
m2 = 66.5;                      % seksjon 1 [kg]
m3 = 201.8;                     % seksjon 2-5 + halve slangen [kg]
y3 = 6.6;                       % tyngdepunkt bak tuppen [m]
m_plate = 21;                   % plate + kamera + halve slangen (anslag) [kg]
r_plate = [0 0 0];              % tyngdepunkt for platen i frame 4 [m]

% Dynamiske parametere, UR5-inspirert arm (fra UR5-spesifikasjonen)
m_ur = [3.7 8.393 2.33 1.219 1.219];
r_ur = [0 -0.02561 0.00193;
        0.2125 0 0.11336;
        0.15 0 0.0265;
        0 -0.0018 0.01634;
        0 0.0018 0.01634];
m6 = 0.1879;                    % UR5 ledd 6 (slås sammen med ledd 5) [kg]
r6 = [0 0 0.0823 - 0.001159];   % tyngdepunkt ledd 6, i frame 5 [m]
md = 3;                         % dysen [kg]
rd = [0 0 0.0823 + L_dyse/2];   % tyngdepunkt dyse (midt på), i frame 5 [m]

g = [0 0 9.81];                 % gravitasjon [m/s^2]

%% Kinematics model of lift using Toolbox

L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(3) = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', pi/2, 'qlim', [L_min L_max]);
L(4) = Link('revolute','d', 0,'a', 0,'alpha', -pi/2);

lift = SerialLink(L, 'name', 'Lift');
liftQ = @(t1, t2, d3) [t1, t2, d3, t2 - pi/2];
q_lift = liftQ(0, 2*pi/3, 6);  
% figure; hold on; 
% lift.plot(q_lift, 'workspace', [-13 13 -13 13 -1 13], 'scale', 0.3);

%% Kinematics model of robot arm using Toolbox

L2(1) = Link('revolute', 'd', 0.089159, 'a', 0, 'alpha', pi/2);
L2(2) = Link('revolute', 'd', 0, 'a', -0.425, 'alpha', 0);
L2(3) = Link('revolute', 'd', 0, 'a', -0.39225, 'alpha', 0);
L2(4) = Link('revolute', 'd', 0.10915, 'a', 0, 'alpha', -pi/2);
L2(5) = Link('revolute', 'd', 0.09465, 'a', 0, 'alpha', pi/2);

arm = SerialLink(L2, 'name', 'arm', 'tool', SE3(0, 0, 0.0823 + L_dyse));       
q_arm = [0, -pi/2, pi/2, 0, pi/2];
% Lager en virtuell arm med 6 ledd for å bruke ikine-funksjonen i Toolbox,
% fjerner 6. leddet etterpå før vi bruker verdiene til å posisjonere
% robotarmen.
arm_ik = SerialLink([L2, Link('revolute', 'd', 0.0823, 'a', 0, 'alpha', 0)], 'name', 'arm_ik', 'tool', SE3(0, 0, L_dyse));
q_arm_ik0 = [q_arm 0];

% figure; hold on;
% arm.plot(q_arm, 'workspace', [-1 1 -1 1 -0.5 1.2],    'scale', 0.3)

%% Kinematics model of the whole robotic system using Toolbox
full = lift * arm;
full.base = SE3(base_pos);         % Where the base of the robot arm is located in world coordinates
full.tool = arm.tool;

%% Dynamics of the whole robot
% Lift
% First revolute joint
full.links(1).m = m1;
full.links(1).r = [0 0 0];      

% Section 1 of prismatic arm, along z2
full.links(2).m = m2; 
full.links(2).r = [0 0 s/2];    

% Section 2-5 of prismatic arm, behind the lift's end
full.links(3).m = m3;
full.links(3).r = [0 -y3 0];    

% Horizontal plate
full.links(4).m = m_plate;
full.links(4).r = r_plate;

% UR5 inspired robot arm
% Joint 6 and the nozzle are added to joint 5
m5 = m_ur(5) + m6 + md;
r_ur(5,:) = (m_ur(5) * r_ur(5,:) + m6*r6 + md*rd) / m5;
m_ur(5) = m5;

for k = 1:5
    full.links(4+k).m = m_ur(k);
    full.links(4+k).r = r_ur(k,:);
end

for k = 1:full.n
    full.links(k).I = zeros(3);         % Assume ideal joints, no inertia
end

full.gravity = g;        % Gravitasjonskrefter på roboten

full.dyn

q = [q_lift q_arm];
figure; hold on;
full.plot(q, 'workspace', [-13 13 -13 13 -1 13], 'scale', 0.1)

%% Demonstration of equivalence of the transformation matrix of the first four joints
syms t1 t2 d3 t4 real

q = [t1 t2 d3 t4 zeros(1,6)];
T04_tb = full.A(1:4, q).T;

c1 = cos(t1); s1 = sin(t1);
c2 = cos(t2); s2 = sin(t2);
c4 = cos(t4); s4 = sin(t4);

trans_0_4 = [c1*c2*c4+c1*s2*s4 s1 c1*s2*c4-c1*c2*s4 d3*c1*s2; s1*c2*c4+s1*s2*s4 -c1 s1*s2*c4-s1*c2*s4 d3*s1*s2; s2*c4-c2*s4 0 -c2*c4-s2*s4 -d3*c2; 0 0 0 1];

result = simplify(T04_tb - trans_0_4);
vpa(result, 2)

%% Develop the inverse kinematics using Toolbox
p = [3, 1.5, 8];        % Vilkårlig punkt på veggen
T_mal = SE3(p) * SE3.Rx(-pi/2);      % End-effectoren peker mot veggen langs positiv y0
q_ik = ik_system(lift, arm_ik, full.base, T_mal, q_arm_ik0, L_min, L_max);           % Bruker egen inverse-kinematics funksjon

%% Sensor to robot command (camera detection → joint angles)

% 0T4 evaluated numerically at the current lift configuration
t1 = q_lift(1); t2 = q_lift(2); d3 = q_lift(3); t4 = q_lift(4);
c1 = cos(t1); s1 = sin(t1);
c2 = cos(t2); s2 = sin(t2);
c4 = cos(t4); s4 = sin(t4);

trans_0_4 = [c1*c2*c4+c1*s2*s4,  s1, c1*s2*c4-c1*c2*s4, d3*c1*s2;
             s1*c2*c4+s1*s2*s4, -c1, s1*s2*c4-s1*c2*s4, d3*s1*s2;
             s2*c4-c2*s4,         0, -c2*c4-s2*s4,      -d3*c2;
             0,                   0, 0,                  1];

dirt_position = [0.2 -0.1 2.0];        % Vilkårlig punkt på veggen der kameraet oppdager skitt
%% Komponenter i dirt_position
% x - hvor langt til høyre i bildet
% y - hvor langt ned i bildet
% z - avstand fra kamera til flekk
trans_4_C = [0 -1 0 0; 0 0 -1 0; 1 0 0 0; 0 0 0 1]
trans0_C = trans_0_4 * trans_4_C;

T_wC = full.base.T * trans0_C;        % transformation matrix camera -> world
p_w = T_wC * [dirt_position'; 1];   % dirt position in world coordinates
T_dirt = SE3(p_w(1), p_w(2) - 0.3, p_w(3)) * SE3.Rx(-pi/2); 

q0 = [q_lift q_arm];
q = full.ikine(T_dirt, 'q0', q0);

%% Oppsett av workspace med vegg
figure; hold on; axis equal; grid on
view(35, 20)

% Vegg: 10 x 10 m i planet y = 0
patch([0 10 10 0], [0 0 0 0], [0 0 10 10], [0.8 0.8 0.8], ...
      'FaceAlpha', 0.5, 'EdgeColor', 'k');

% Bakke
patch([-2 12 12 -2], [-6 -6 1 1], [0 0 0 0], [0.6 0.75 0.6], ...
      'FaceAlpha', 0.3, 'EdgeColor', 'none');

% Noen vinduer (rektangler litt foran veggen så de synes)
win = [2 4 3 5; 6 8 3 5; 2 4 7 9; 6 8 7 9];   % [x1 x2 z1 z2]
for i = 1:size(win,1)
    patch(win(i,[1 2 2 1]), -0.01*ones(1,4), win(i,[3 3 4 4]), ...
          [0.6 0.8 1], 'FaceAlpha', 0.7);
end

disp(['Flekk (world):     ' num2str(p_w(1:3)')])
disp(['Dysemål (world):   ' num2str(T_dirt.t')])
disp(['Armbase (world):   ' num2str(arm.base.t')])
T_check = full.fkine(q);
disp(['Dyse oppnådd:      ' num2str(T_check.t')])

% Skittflekk (målpunkt fra kameraet)
plot3(p_w(1), 0, p_w(3), 'r.', 'MarkerSize', 25);

full.plot(q, 'workspace', [-2 12 -6 2 0 13], 'scale', 0.1);

%% Invers kinematikk-funksjon (denne må stå nederst i scriptet)
% Denne grovposisjonerer liften først, før den regner ut ikine for
% robotarmen. På denne måten bruker vi liften til grovposisjonering, før vi
% gjør selve vaskeoperasjonen med robotarmen.
function q = ik_system(lift, arm_ik, base, T_mal, q0_arm, L_min, L_max)
    % T_mal.t = målposisjonen til dysen
    % T_mal.a = z-aksen til dysen, altså retningen strålen peker
    % T_mal.t - 0.5 * T_mal.a = 0.5 meter bak dysen, der platen med
    % robotarmen festet skal stå. 
    % - base.t gjør punktet relativt til liftbasen i stedet for
    % verdenskoordinater.

    % Steg 1: grovposisjonering av lifen. Regnes ut ved hjelp av geometri
    p = (T_mal.t - 0.5*T_mal.a) - base.t; 
    d3 = min(max(norm(p), L_min), L_max);   % begrenser til bommens lengde
    q_lift = [atan2(p(2), p(1)), atan2(hypot(p(1), p(2)), -p(3)), d3];
    q_lift(4) = q_lift(2) - pi/2;

    % Steg 2: invers kinematikk av robotarmen
    % Bruker ikke verdien for ledd 6 her, siden det er et virtuelt ledd for
    % å kunne bruke ikine-funksjonen
    arm_ik.base = base * lift.fkine(q_lift);    % Plasserer robotarmen på platen (enden av liften) (i verdenskoordinater)
    q6 = arm_ik.ikine(T_mal, 'q0', q0_arm);     % ikine-funksjonen regner ut robotarmens 6 leddvinkler for å sette dysen i T_mal.
    q = [q_lift q6(1:5)];                       % Setter sammen verdiene. Dropper siste verdi, siden robotarmen har 5 ledd.
end
Claude finished the response
