%% Clear the workspace 
clear;
clc;
close all force;

%% Konstanter
% Geometri
L_min = 3.6;                    % innskjøvet bomlengde [m]
L_max = 15;                     % utskjøvet bomlengde [m]
L_dyse = 0.15;                  % lengde på dysen [m]
base_pos = [-3 -2 1.5];         % liftbasen i verdenskoordinater [m]
h_c = 0.3;                      % kameraets avstand under origo i frame 4 [m]
q2_min = deg2rad(75);           % bommen kan gå maks 15 grader under horisontal
z_plate_min = 0.9;              % platen skal aldri lavere enn dette [m] (kamera 0.3 m under)

% Dynamiske parametere, lift (verdier fra utregning)
s = 3.6;                        % seksjonslengde [m]
m1 = 40;                        % dreiekrans (anslag) [kg]
m2 = 66.5;                      % seksjon 1 [kg]
m3 = 201.8;                     % seksjon 2-5 + halve slangen [kg]
y3 = 6.6;                       % tyngdepunkt bak tuppen [m]
m_plate = 10;                   % plate [kg]
m_camera = 1;                   % kamera [kg]
m_plate_hose = 10;              % antar at halve slangen sin vekt hviler på platen [kg]
% Posisjoner i frame 4 (x4 peker opp, z4 horisontalt ut mot armen)
r_plate_only = [0 0 -0.005];    % platen, sentrert om origo, armen på forsiden [m]
r_camera = [-h_c 0 0];          % kameraet, h_c under origo [m]
r_plate_hose = [0 0 -0.05];     % slangen kobles til bak platen [m]

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

% Robotmodell (Puma560)
mdl_puma560;

%% Kinematics model of lift using Toolbox

L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0, 'a', 0, 'alpha', pi/2, 'qlim', [q2_min pi]);   % bom maks 15° under horisontal
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

% Vertical plate: plate, camera and half the hose combined into one point mass
m4 = m_plate + m_camera + m_plate_hose;
r4 = (m_plate*r_plate_only + m_camera*r_camera + m_plate_hose*r_plate_hose) / m4;
full.links(4).m = m4;
full.links(4).r = r4;

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

% For requirement 4.ii, så legger vi til G, Jm og B for joint 2 på
% robotarmen (joint 6 på full)
for k = 5:9                                  % robot arm joints
    pk = k - 4;                              % samme joint nummer på P560
    full.links(k).G  = 101;
    full.links(k).Jm = p560.links(pk).Jm;
    full.links(k).B  = p560.links(pk).B;
end

%% Kontroll av dynamikkmodellen
q_hor = [liftQ(0, pi/2, L_max) q_arm];   % horisontal, helt utskjøvet bom
tau_g = full.gravload(q_hor);
disp(tau_g)

% Håndregning: moment om ledd 2, med armen som punktmasse i tuppen
m_arm = sum(m_ur);                       % inkluderer ledd 6 og dysen
M0 = 9.81*(m2*s/2 + m3*(L_max - y3) + (m4 + m_arm)*L_max);
fprintf('gravload ledd 2: %.0f Nm, håndregning: %.0f Nm\n', abs(tau_g(2)), M0)

q = [q_lift q_arm];
figure; hold on;
full.plot(q, 'workspace', [-13 13 -13 13 -1 13], 'scale', 0.1)

%% Demonstration of equivalence of the transformation matrix of the first four joints
% syms t1 t2 d3 t4 real
% 
% q = [t1 t2 d3 t4 zeros(1,6)];
% T04_tb = full.A(1:4, q).T;
% 
% c1 = cos(t1); s1 = sin(t1);
% c2 = cos(t2); s2 = sin(t2);
% c4 = cos(t4); s4 = sin(t4);
% 
% trans_0_4 = [c1*c2*c4+c1*s2*s4 s1 c1*s2*c4-c1*c2*s4 d3*c1*s2; s1*c2*c4+s1*s2*s4 -c1 s1*s2*c4-s1*c2*s4 d3*s1*s2; s2*c4-c2*s4 0 -c2*c4-s2*s4 -d3*c2; 0 0 0 1];
% 
% result = simplify(T04_tb - trans_0_4);
% vpa(result, 2)

%% Develop the inverse kinematics using Toolbox
p = [3, 1.5, 8];        % Vilkårlig punkt på veggen
T_mal = SE3(p) * SE3.Rx(-pi/2);      % End-effectoren peker mot veggen langs positiv y0
q_ik = ik_system(lift, arm_ik, full.base, T_mal, q_arm_ik0, L_min, L_max, z_plate_min);           % Bruker egen inverse-kinematics funksjon

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

%% Joint dynamic model and demonstration of how to analyze the robot dynamics and transfer function
q_str = [q_lift 0 0 0 0 0];
joint_tf = full.jointdynamics(q_str);

% Fokuserer videre på skulderleddet (ledd 2 på robotarm / ledd 6 på hele
% roboten)
% Henter ut transferfunksjonen til ledd 6. 
s_ = tf('s');
P_v = tf(joint_tf(6));           % moment -> hastighet. jointdynamics gir hastigheten
P = P_v / s_;                    % Deler på s, får da vinkelen. Går fra moment til vinkel.

% Leser ut J' og B'
% Nevneren i P_v er J's + B', så de to koeffisientene som blir hentet ut er
% J' og B'
[~, den] = tfdata(P_v, 'v');
J_str = den(1)                          % J' for strukket arm
B = den(2)                              % B'

% Finner polene
% Polene forteller om systemet (her leddet) er stabilt
pole(P)                 
tau = J_str / B                         % tidskonstant [s]

% Sjekker sprangresponsen, strukket mot bøyd arm
% Lager samme modell for armen strukket ut og bøyd
q_boy = [q_lift 0 0 pi/2 0 0];          % albuen bøyd 90 grader
tf_b = full.jointdynamics(q_boy);
P_b = tf(tf_b(6)) / s_;
[~, den_b] = tfdata(P_b, 'v');
J_boy = den_b(1);

figure;
step(P, P_b, 5);
grid on;
legend('Strukket', 'Bøyd', 'Location', 'northwest');
title('Skulder, åpen sløyfe: konstant moment \rightarrow vinkel')

% Gravitasjonsforstyrrelsen
% Roterer skulderen gjennom alle vinkler med armen strukket, og regner ut
% hvor mye tyngdekraften drar i den for hver vinkel
q2v = linspace(-pi, pi, 73);            % skuldervinkel å teste
tg = zeros(size(q2v));
for k = 1:numel(q2v)
    qk = q_str;
    qk(6) = q2v(k);                     % endrer bare skulderen, ingen andre ledd
    tau_k = full.gravload(qk);
    tg(k) = tau_k(6);                   % moment på skulderen
end

tau_g_max = max(abs(tg))                % leddside [Nm]
d_motor = tau_g_max / full.links(6).G   % gravitasjonsforstyrrelsen sett fra motoren [Nm]

figure;
plot(rad2deg(q2v), tg, 'LineWidth', 1.5)
grid on;
xlabel('Skuldervinkel [grader]')
ylabel('/tau_g [Nm]')
title('Gravitasjonsmoment på skulderen, strukket arm')

%% Velocity controller (P + PI)
% 1. Velg polen og regn ut Kv
tau_i = 0.05;                           % tidskonstant for indre sløyfe [s]
p_i = 1 / tau_i;                        % polen [rad/s]
Kv = J_str * p_i - B;                   % regner ut Kv fra polen og tidligere utregnede verdier
% Velger Ki slik at nullpunktet -Ki/Kv havner rundt -4
Ki = 4 * Kv;

% 2. Regulatoren
C_P = Kv;
C_PI = Kv + Ki/s_;

% 3. Lukket sløyfe
T_ref_P = feedback(C_P * P_v, 1);
T_dist_P = -feedback(P_v, C_P);

T_ref_PI = feedback(C_PI * P_v, 1);
T_dist_PI = -feedback(P_v, C_PI);

% 4. Simulering
t      = 0:0.001:2;                     % tidsvektor [s]
w_ref  = 50 * ones(size(t));            % ønsket motorhastighet [rad/s]
d      = d_motor * ones(size(t));       % tyngdekraften, konstant

w_P = lsim(T_ref_P, w_ref, t) + lsim(T_dist_P, d, t);
w_PI = lsim(T_ref_PI, w_ref, t) + lsim(T_dist_PI, d, t);

% 5. Plot
figure; plot(t, w_ref, 'w--', t, w_P, t, w_PI, 'LineWidth', 1.3); grid on
legend('\omega^*', 'P', 'PI', 'Location', 'southeast')
xlabel('t [s]'); ylabel('\omega [rad/s]')

%% Posisjonsregulator (P)
% Posisjonsregulatoren er en P-regulator
% ts = 4/Kp => Kp = 4/ts, Ts < 2 sek => Kp > 2
% Velger Kp = 3
% Den indre sløyfen bør være 5-10 ganger raskere => p = -Kp = -3
% Indre pol / ytre pol = 20/3 = 6-7 ganger raskere
% Gir ts = 4/3 = 1.33 sek, som er under kravet om 2 sek. 

Kp = 3;
L_ytre = Kp * T_ref_PI / s_;                            % ytre sløyfeforsterkning
T_pos = feedback(L_ytre, 1);                            % theta* -> theta
T_pos_d = minreal((T_dist_PI / s_) / (1 + L_ytre));     % d -> theta

G = full.links(6).G;
t = 0:0.001:4;
theta_ref = deg2rad(10) * G * ones(size(t));            % 10 grader sprang, omregnet til motorside
d = d_motor * ones(size(t));

theta = lsim(T_pos, theta_ref, t) + lsim(T_pos_d, d, t);
theta_ledd = rad2deg(theta / G);                        % tilbake til leddvinkel i grader

figure; plot(t, theta_ledd, 'LineWidth', 1.3); grid on; hold on
yline(10, '--'); yline(11.5, ':', 'OS 15 %'); xline(2, ':', 't_s = 2 s')
xlabel('t [s]'); ylabel('Leddvinkel [°]')
title('Posisjonsregulator, skulder')

info = stepinfo(theta_ledd, t, 10, 'SettlingTimeThreshold', 0.02)
avvik = abs(10 - theta_ledd(end))

% Sjekk av kommandert moment
dt = t(2) - t(1);
omega = gradient(theta, dt);                    % faktisk motorhastighet [rad/s]
omega_ref = Kp * (theta_ref' - theta);          % ønsket hastighet fra ytre sløyfe
e_omega = omega_ref - omega;                    % hastighetsavvik inn til PI
tau_cmd = lsim(C_PI, e_omega, t);               % kommandert motormoment [Nm]

tau_maks = max(abs(tau_cmd));

figure; plot(t, tau_cmd, 'LineWidth', 1.3); grid on; hold on
yline(0.9, '--', 'Momentgrense');
yline(d_motor, ':', 'Holde mot tyngdekraften');
xlabel('t [s]'); ylabel('\tau_{cmd} [Nm]')
title('Kommandert motormoment, 10° sprang')

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

%% Motion planning 
plan = window_washer_planner(full, struct('nSpots', 10, 'showAnimation', false));

% Kontroll: planleggerens dyseposisjon = full.fkine
q_test = plan.traj(end, 4:12);
disp(full.fkine(q_test).t' - plan.traj(end, 13:15));            % Skal være ~0

% Kontroll av hele 10x10-området: én flekk i hver av de 100 rutene
% plan_alle = window_washer_planner(full, struct('nSpots', 100, 'showAnimation', false, 'exportCSV', false));
% lav = plan_alle.patches(:,3) < 0.7;
% fprintf('Nederste rad: %d/%d løst, totalt %d/100\n', sum(plan_alle.complete(lav)), sum(lav), sum(plan_alle.complete));
% plan_alle.failure(~plan_alle.complete)                         % årsak for de som feiler

%% Animasjon av vaskeoppgaven
plan = window_washer_planner(full, struct('showAnimation', false));
Q = plan.traj(1:5:end, 4:12);              % kvar 5. sample, så det går raskare

figure; hold on; axis equal; grid on; view(35, 20)
patch([0 10 10 0], [0 0 0 0], [0 0 10 10], [0.8 0.9 1], ...
      'FaceAlpha', 0.3, 'EdgeColor', 'k');                 % veggen
m = plan.traj(:,2) == 2;                                   % vaskefasen
plot3(plan.traj(m,13), plan.traj(m,14), plan.traj(m,15), ...
      '.', 'Color', [0.2 0.7 0.3], 'MarkerSize', 3);       % vaskespora

full.plot(Q, 'workspace', [-4 11 -6 2 0 13], 'scale', 0.1, ...
          'fps', 30, 'trail', 'r', 'movie', 'vasking.mp4');

%% Recursive Newton-Euler på en representativ vaskebane
% Velg første flekk som ble vasket, og ta med hele sekvensen:
% lift-transport (fase 1), tilnærming (3), vasking (2) og tilbaketrekking (4)
ids = plan.order(plan.complete(plan.order));
id  = ids(1);
seg = plan.traj(plan.traj(:,3) == id, :);
qs  = seg(:, 4:12);                      % leddvinkler, samme konvensjon som full
ph  = seg(:, 2);                         % fase per sample
tip = seg(:, 13:15);                     % dyseposisjon

% 1) Realistisk tidsskala: planleggeren har fast dt per sample, så vi
%    retimer etter dysens buelengde. 10 cm/s under vasking (kravet), raskere ellers.
v = 0.10*ones(size(ph));  v(ph == 1) = 0.30;  v(ph == 3 | ph == 4) = 0.15;   % [m/s]
ds = [0; vecnorm(diff(tip), 2, 2)];
ts = cumsum(ds ./ v);
[ts, iu] = unique(ts);  qs = qs(iu, :);  % fjern samples uten bevegelse

% 2) Jevn trajektorie: interpoler, glatt og deriver numerisk
dtr = 0.02;
tt  = (0:dtr:ts(end))';
Q   = smoothdata(interp1(ts, qs, tt, 'pchip'), 'gaussian', 25);
QD  = gradient(Q', dtr)';
QDD = gradient(QD', dtr)';

% 3) Invers dynamikk med RNE
tau   = full.rne(Q, QD, QDD);            % totalt moment/kraft per ledd
tau_g = full.gravload(Q);                % bare gravitasjon
tau_d = tau - tau_g;                     % treghet, Coriolis og friksjon

% Vannstrålen: 40 N langs dysens akse, bare under vasking
F_jet = 40;
phT = interp1(ts, ph(iu), tt, 'nearest');
T = full.fkine(Q);
tau_jet = zeros(size(Q));
for k = find(phT == 2)'
    J = full.jacob0(Q(k,:));
    tau_jet(k,:) = J(1:3,:)' * (F_jet * T(k).a(:));
end
tau_tot = tau + tau_jet;

% 4) Plot
figure('Name', 'RNE – representativ vaskebane');
subplot(3,1,1); plot(tt, tau_tot(:,[1 2 4]), 'LineWidth', 1.3); grid on
ylabel('\tau [Nm]'); legend('Ledd 1 (sving)', 'Ledd 2 (bom)', 'Ledd 4 (plate)');
title(sprintf('Lift – flekk %d', id))
subplot(3,1,2); plot(tt, tau_tot(:,3), 'LineWidth', 1.3); grid on
ylabel('F_3 [N]'); legend('Ledd 3 (teleskop)');
subplot(3,1,3); plot(tt, tau_tot(:,5:9), 'LineWidth', 1.3); grid on
ylabel('\tau [Nm]'); xlabel('t [s]'); legend('Arm 1', 'Arm 2', 'Arm 3', 'Arm 4', 'Arm 5');
title('Robotarm')

% 5) Nøkkeltall til rapporten
fprintf('\n%-6s %12s %12s %12s\n', 'Ledd', 'max|tau|', 'max|grav|', 'max|dyn|');
for j = 1:9
    fprintf('%-6d %12.1f %12.1f %12.2f\n', j, max(abs(tau_tot(:,j))), ...
            max(abs(tau_g(:,j))), max(abs(tau_d(:,j))));
end

fprintf('\n%-6s %12s %12s %12s %12s\n', 'Ledd', 'max|tau|', 'max|grav|', 'max|dyn|', 'max|jet|');
for j = 1:9
    fprintf('%-6d %12.1f %12.1f %12.2f %12.1f\n', j, max(abs(tau_tot(:,j))), ...
            max(abs(tau_g(:,j))), max(abs(tau_d(:,j))), max(abs(tau_jet(:,j))));
end

G = full.links(6).G;
fprintf('Arm-skulder sett fra motoren: %.3f Nm (G = %d)\n', max(abs(tau_tot(:,6)))/G, G);

%% Dysefart under vasking
% Farten regnes fra den glattede trajektorien som RNE bruker, med
% differensialkinematikk: v = J(q) * qd. (Tidligere ble farten målt som
% korden mellom to interpolerte punkter, som gir 0.1/sqrt(2) = 0.07 m/s
% over 90-graders hjørner i rasteret selv om dysen faktisk går 0.1 m/s.)
vel = zeros(size(tt));
for k = 1:numel(tt)
    J = full.jacob0(Q(k,:));
    vel(k) = norm(J(1:3,:) * QD(k,:)');                % lineær dysefart [m/s]
end
phT = interp1(ts, ph(iu), tt, 'nearest');              % fase per sample

% Skill rette strøk fra hjørner (retningsendring > 10 grader), med margin rundt hjørnene
tipF = full.fkine(Q).transl;                           % dyseposisjon fra glattet trajektorie
retn_t = diff(tipF);
retn_t = retn_t ./ max(vecnorm(retn_t, 2, 2), eps);
vinkel = acosd(min(1, max(-1, sum(retn_t(1:end-1,:) .* retn_t(2:end,:), 2))));
turn = [false; vinkel > 10; false];
hjorne = movmax(double(turn), 31) > 0;                 % ±0.3 s rundt hvert hjørne
rett = phT == 2 & ~hjorne;

fprintf('Dysefart på rette vaskestrøk: min %.3f, snitt %.3f m/s\n', min(vel(rett)), mean(vel(rett)));
if any(phT == 2 & hjorne)
    fprintf('Dysefart i hjørner: min %.3f m/s (bremser bevisst i retningsendring)\n', min(vel(phT == 2 & hjorne)));
end

figure; plot(tt, vel, 'LineWidth', 1.2); grid on; hold on
yline(0.10, 'r--', 'Krav 10 cm/s');
plot(tt(rett), vel(rett), '.', 'MarkerSize', 4);
xlabel('t [s]'); ylabel('Dysefart [m/s]');
legend('Alle faser', 'Krav', 'Rette vaskestrøk', 'Location', 'best')
title('Dysefart, representativ vaskebane')

%% Kapasitet: klarer armen 10 cm/s overalt der den vasker?
% Kravet er at armen skal KUNNE bevege dysen med minst 10 cm/s i hele
% workspacet. For hver vaskekonfigurasjon i planen regnes leddhastighetene
% som trengs for 0.1 m/s i strøkretningene (vertikalt og sideveis) med
% pseudoinversen av Jacobianen, og sammenlignes med UR5-grensen (180 grader/s).
qa = plan.traj(plan.traj(:,2) == 2, 4:12);             % alle vaskekonfigurasjoner
qa = qa(1:5:end, :);                                   % hver 5., for fart
qd_max_UR = pi;                                        % UR5: 180 grader/s per ledd [rad/s]
retn = [0 0 1; 1 0 0];                                 % vertikalt strøk, sidesteg
behov = zeros(size(qa,1), 1);
for k = 1:size(qa,1)
    J = full.jacob0(qa(k,:));
    Jv = J(1:3, 5:9);                                  % bare armleddene, liften står stille
    for r = 1:2
        qd = pinv(Jv) * (0.10 * retn(r,:)');
        behov(k) = max(behov(k), max(abs(qd)));
    end
end
v_maks = 0.10 * qd_max_UR ./ behov;                    % høyeste mulige dysefart per konfigurasjon
fprintf('Laveste oppnåelige dysefart i workspace: %.2f m/s (krav 0.10)\n', min(v_maks));
fprintf('Største leddhastighet ved 10 cm/s: %.1f grader/s (grense 180)\n', rad2deg(max(behov)));

figure; histogram(v_maks, 30); grid on; hold on
xline(0.10, 'r--', 'Krav 10 cm/s', 'LineWidth', 1.5);
xlabel('Maks oppnåelig dysefart [m/s]'); ylabel('Antall konfigurasjoner');
title('Fartskapasitet over alle vaskekonfigurasjoner')

%% Simulering: vannstrålen når hele omkretsen av 10 x 10 m-området
% Lim inn i kinematics_model.m, rett før "%% Invers kinematikk-funksjon".
% Hvert punkt er der strålen treffer veggen. Dysen står 'avstand' fra treffpunktet.
% Står strålen vinkelrett og armen ikke rekker, vinkles dysen nedover (krav: maks 45°).

steg    = 0.25;                          % avstand mellom punktene på omkretsen [m]
avstand = 0.3;                           % dyse til vegg, samme som T_dirt [m]
tilts   = deg2rad([0:10:40 45]);         % dysevinkler som prøves, minste først
tol_p   = 1e-3;                          % godkjent posisjonsfeil [m]
tol_r   = deg2rad(0.5);                  % godkjent feil på dyseaksen [rad]

% Omkretsen: nederst, høyre side, øverst, venstre side
u = (0:steg:10)';
omkrets = [u,                 zeros(size(u));
           10*ones(numel(u)-1,1), u(2:end);
           flipud(u(1:end-1)), 10*ones(numel(u)-1,1);
           zeros(numel(u)-2,1), flipud(u(2:end-1))];
N = size(omkrets, 1);

% Startgjetninger til ikine
starter = [q_arm_ik0;
           0     -1.2  1.5  -0.3  pi/2  0;
           0.2    0.3  0.1   1.6  0.4   0;
          -0.5   -1.7  2.5  -1.0  0.6   1.4];

Q_omkrets = nan(N, 9);
tilt_brukt = nan(N, 1);
feil_p = nan(N, 1);
q_forrige = q_arm_ik0;
varsel = warning('off', 'all');          % ikine varsler når et forsøk ikke konvergerer

for i = 1:N
    treff = [omkrets(i,1) 0 omkrets(i,2)];
    for phi = tilts
        % Rx(-pi/2) peker dysen mot veggen, -phi vinkler den nedover
        T_mal = SE3(treff) * SE3.Rx(-pi/2 - phi) * SE3(0, 0, -avstand);
        kandidater = [q_forrige; starter];
        funnet = false;
        for r = 1:size(kandidater, 1)
            try
                q_i = ik_system(lift, arm_ik, full.base, T_mal, kandidater(r,:), ...
                                L_min, L_max, z_plate_min);
            catch
                continue
            end
            if any(isnan(q_i)), continue; end
            T_i = full.fkine(q_i);
            e_p = norm(T_i.t - T_mal.t);
            % Bare dyseaksen teller: rotasjon om strålen har ingen betydning
            e_r = acos(min(1, max(-1, dot(T_i.a, T_mal.a))));
            if e_p < tol_p && e_r < tol_r
                funnet = true;
                break
            end
        end
        if funnet
            Q_omkrets(i,:) = q_i;
            tilt_brukt(i) = rad2deg(phi);
            feil_p(i) = e_p;
            q_forrige = [q_i(5:9) 0];
            break
        end
    end
end
warning(varsel);

% Resultat
naadd = ~isnan(tilt_brukt);
fprintf('\nOmkrets: %d av %d punkter nådd\n', sum(naadd), N);
fprintf('Største posisjonsfeil: %.2f mm\n', 1000*max(feil_p));
fprintf('Største dysevinkel: %.0f grader (krav: 45)\n', max(tilt_brukt));
fprintf('Vinkelrett på veggen: %d punkter, vinklet: %d punkter\n', ...
        sum(tilt_brukt == 0), sum(tilt_brukt > 0));
fprintf('Bomlengde: %.2f til %.2f m (grenser %.1f og %.1f)\n', ...
        min(Q_omkrets(:,3)), max(Q_omkrets(:,3)), L_min, L_max);

% Figur 1: hvilke punkter som ble nådd, og med hvilken dysevinkel
figure('Name', 'Omkrets av arbeidsområdet'); hold on; axis equal; grid on
patch([0 10 10 0], [0 0 0 0], [0 0 10 10], [0.8 0.9 1], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
scatter3(omkrets(naadd,1), zeros(sum(naadd),1), omkrets(naadd,2), 45, tilt_brukt(naadd), 'filled');
plot3(omkrets(~naadd,1), zeros(sum(~naadd),1), omkrets(~naadd,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
c = colorbar; c.Label.String = 'Dysevinkel nedover [°]';
view(0, 0); xlabel('x [m]'); zlabel('z [m]');
title(sprintf('Treffpunkt på omkretsen: %d av %d nådd', sum(naadd), N))

% Figur 2: hva løftet og dysen må gjøre langs omkretsen
figure('Name', 'Omkrets: bom og dysevinkel');
subplot(2,1,1); plot(1:N, Q_omkrets(:,3), 'LineWidth', 1.3); grid on; hold on
yline(L_max, '--', 'L_{max}'); yline(L_min, '--', 'L_{min}');
ylabel('Bomlengde [m]');
subplot(2,1,2); stairs(1:N, tilt_brukt, 'LineWidth', 1.3); grid on; hold on
yline(45, '--', 'Krav 45°');
xlabel('Punkt langs omkretsen'); ylabel('Dysevinkel [°]');

% Animasjon: roboten følger omkretsen
Q_ok = Q_omkrets(naadd, :);
m = size(Q_ok, 1);
Q_anim = interp1(1:m, Q_ok, linspace(1, m, 3*m));

figure('Name', 'Animasjon av omkretsen'); hold on; axis equal; grid on; view(35, 20)
patch([0 10 10 0], [0 0 0 0], [0 0 10 10], [0.8 0.9 1], 'FaceAlpha', 0.3, 'EdgeColor', 'k');
plot3(omkrets(naadd,1), zeros(sum(naadd),1), omkrets(naadd,2), 'g.', 'MarkerSize', 10);
full.plot(Q_anim, 'workspace', [-4 12 -6 2 0 13], 'scale', 0.1, 'fps', 30, 'trail', 'r');
% Legg til 'movie', 'omkrets.mp4' i full.plot for å lagre video til presentasjonen.

%% Invers kinematikk-funksjon (denne må stå nederst i scriptet)
% Denne grovposisjonerer liften først, før den regner ut ikine for
% robotarmen. På denne måten bruker vi liften til grovposisjonering, før vi
% gjør selve vaskeoperasjonen med robotarmen.
function q = ik_system(lift, arm_ik, base, T_mal, q0_arm, L_min, L_max, z_min)
    % T_mal.t = målposisjonen til dysen
    % T_mal.a = z-aksen til dysen, altså retningen strålen peker
    % T_mal.t - 0.5 * T_mal.a = 0.5 meter bak dysen, der platen med
    % robotarmen festet skal stå. 
    % - base.t gjør punktet relativt til liftbasen i stedet for
    % verdenskoordinater.
    % z_min = laveste høyde platen kan stå på. For lave mål står platen
    % høyere, og robotarmen rekker ned resten.
    if nargin < 8, z_min = -inf; end

    % Steg 1: grovposisjonering av liften. Regnes ut ved hjelp av geometri
    p_plate = T_mal.t - 0.5*T_mal.a;
    p_plate(3) = max(p_plate(3), z_min);    % platen aldri under z_min
    p = p_plate - base.t;
    h = hypot(p(1), p(2));
    t2 = atan2(h, -p(3));
    d3 = norm(p);
    q2_lim = lift.links(2).qlim;            % bomvinkelen begrenses (maks 15° under horisontal)
    if ~isempty(q2_lim) && (t2 < q2_lim(1) || t2 > q2_lim(2))
        t2 = min(max(t2, q2_lim(1)), q2_lim(2));
        d3 = h / sin(t2);                   % behold horisontal rekkevidde
    end
    d3 = min(max(d3, L_min), L_max);        % begrenser til bommens lengde
    q_lift = [atan2(p(2), p(1)), t2, d3];
    q_lift(4) = q_lift(2) - pi/2;

    % Steg 2: invers kinematikk av robotarmen
    % Bruker ikke verdien for ledd 6 her, siden det er et virtuelt ledd for
    % å kunne bruke ikine-funksjonen
    arm_ik.base = base * lift.fkine(q_lift);    % Plasserer robotarmen på platen (enden av liften) (i verdenskoordinater)
    q6 = arm_ik.ikine(T_mal, 'q0', q0_arm);     % ikine-funksjonen regner ut robotarmens 6 leddvinkler for å sette dysen i T_mal.
    q = [q_lift q6(1:5)];                       % Setter sammen verdiene. Dropper siste verdi, siden robotarmen har 5 ledd.
end
