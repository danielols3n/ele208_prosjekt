%% gazebo_vaskebane.m
% Kjorer vaskebanen fra window_washer_planner.m paa roboten i Gazebo over ROS 2.
%
% Krav: ROS Toolbox + Peter Corkes Robotics Toolbox. Gazebo startes med pakken i
% Gazebo_model/ og samme ROS_DOMAIN_ID som MATLAB:
%     bash start_gazebo.sh
% Liftbasen er fast i URDF-en i [-3 -2 1.5], saa full.base maa vaere den samme.
% Vent til Gazebo-terminalen skriver TOP READY eller CHAIN NOT READY.
%
% Rekkefolge i MATLAB:
%     kinematics_model                         % lager `full`
%     res = window_washer_planner(full);       % lager vaskebanen (kan hoppes over)
%     gazebo_vaskebane                         % denne fila
% Finnes ikke `res`, planlegges banen her. Finnes ikke `full`, kjores kinematics_model.m.
%
% res.traj(:,4:12) = q = [t1 t2 d3 t2-pi/2 q_arm(1:5)] (konvensjonen til `full`).
% Omregning til Gazebo-leddene (URDF-en er bygd direkte fra DH-tabellen):
%   boom_yaw, boom_pitch, arm_swivel, arm_joint_1..5 = samme vinkler som i `full`
%   boom_extension = (d3 - 3.6)/4           (fire like teleskoptrinn)
%   stage2_slide..stage4_slide = boom_extension (sendes med til arm_controller)
% Verden: samme koordinater i MATLAB og Gazebo (glasset ligger i y = 0).

%% Deklarasjon om KI
% Til denne koden har vi brukt KI som støtte i å skrive kode. Teksten over
% er også skrevet av KI for å lage en oversikt over hva koden inneholder,
% slik at resten av gruppen lettere kan forstå hvordan systemet fungerer.
% Koden er delvis skrevet for hånd, og delvis klipt og limt direkte fra KI.

%% 1) Robotmodell (for innstillingene, siden kinematics_model.m starter med clear)
if ~exist('full','var') || ~isa(full,'SerialLink')
    fprintf('Fant ikke `full`, kjorer kinematics_model.m ...\n');
    kinematics_model;
end

%% Innstillinger
vfrak    = 0.85;     % bruk maks 85 % av leddenes fartsgrense i URDF
kun_vask = false;    % true: kjor bare forste flekk (rask test)

%% Vaskebane
sjekk_modell(full);            % stopper hvis DH/verktoy ikke er det Gazebo-modellen bygger paa
if ~exist('res','var') || ~isstruct(res) || ~isfield(res,'traj') || isempty(res.traj)
    fprintf('Fant ikke `res`, kjorer window_washer_planner ...\n');
    res = window_washer_planner(full, struct('showAnimation', false));
end
bane = res.traj;
if isempty(bane), error('Vaskebanen er tom.'); end
B = full.base; if isa(B,'SE3'), B = B.T; end
base = B(1:3,4)';

if kun_vask
    id = bane(find(bane(:,2) == 2, 1), 3);
    bane = bane(bane(:,3) == id, :);           % flytting, tilnaerming, vask og retur
    fprintf('Test: bare flekk %d (%d punkter).\n', id, size(bane,1));
end

%% 2) Regn om til Gazebo-ledd
ledd = ["boom_yaw","boom_pitch","boom_extension","arm_swivel", ...
        "arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"];
trinn = ["stage2_slide","stage3_slide","stage4_slide"];   % foelger boom_extension
% Leddgrenser og fartsgrenser som i URDF-en (lift_arm.urdf.xacro) og kjor_test.m
nedre = [-3.14, deg2rad(75), 0,    -0.3, -2*pi*ones(1,5)];
ovre  = [ 3.14, pi,          2.85,  1.6,  2*pi*ones(1,5)];
vmax  = [0.15, 0.12, 0.12, 0.5, pi*ones(1,5)];

q_gz = full_til_gazebo(bane(:,4:12));
for j = 5:9        % +-2*pi endrer ikke roboten; velg forskyvning som holder banen innenfor +-2*pi
    k = round((max(q_gz(:,j)) + min(q_gz(:,j)))/2 / (2*pi));
    q_gz(:,j) = q_gz(:,j) - 2*pi*k;
end
utenfor = q_gz < nedre - 1e-6 | q_gz > ovre + 1e-6;
if any(utenfor, 'all')
    [r, c] = find(utenfor, 1);
    error('Punkt %d: %s = %.4f er utenfor Gazebo-grensen [%.3f, %.3f].', ...
          r, ledd(c), q_gz(r,c), nedre(c), ovre(c));
end
[spiss, ~] = dyse_fra_gazebo(q_gz, base);
fprintf('Kontroll av omregning: maks avvik til planlagt dysespiss %.1e m.\n', ...
        max(vecnorm(spiss - bane(:,13:15), 2, 2)));

%% 3) Tidsplan: behold dt fra planleggeren der leddene rekker det, ellers strekk
dt_plan = max(diff(bane(:,1)), 1e-3);
dt_ny = max(dt_plan, max(abs(diff(q_gz)) ./ (vfrak*vmax), [], 2));
tid = [0; cumsum(dt_ny)];
fprintf('Banen: %d punkter, %.0f s i MATLAB -> %.0f s i Gazebo (fartsgrenser).\n', ...
        size(q_gz,1), bane(end,1) - bane(1,1), tid(end));

%% 4) Koble til Gazebo
setenv("ROS_DOMAIN_ID","24")   % samme som i VM-en og kjor_test.m
node = ros2node('/matlab_vaskebane');
sjekk_base_i_gazebo(node, base);
tilstand = ros2subscriber(node, '/arm_controller/controller_state', ...
                          'control_msgs/JointTrajectoryControllerState');
leddsub = ros2subscriber(node, '/joint_states', 'sensor_msgs/JointState');
[klient, maal] = ros2actionclient(node, ...
    '/arm_controller/follow_joint_trajectory', ...
    'control_msgs/FollowJointTrajectory');
if ~waitForServer(klient, 'Timeout', 30)
    error('Gazebo-kontrolleren svarer ikke. Vent til TOP READY og sjekk ROS_DOMAIN_ID.');
end
js = receive(leddsub, 10);
[finnes, idx] = ismember(ledd, string(js.name));
if ~all(finnes), error('Mangler ledd i /joint_states.'); end
q_naa = double(js.position(idx(:)))';

% Myk overgang fra naavaerende stilling til banens start
t_flytt = max(2, max(abs(q_gz(1,:) - q_naa) ./ (0.5*vmax)));
s_ = linspace(0, 1, 21)'; s_ = s_(2:end); s_ = 3*s_.^2 - 2*s_.^3;
q_send = [q_naa + s_.*(q_gz(1,:) - q_naa); q_gz(2:end,:)];
t_send = 0.5 + [linspace(t_flytt/20, t_flytt, 20)'; t_flytt + tid(2:end)];

%% 5) Send hele banen som FollowJointTrajectory
punkt = ros2message('trajectory_msgs/JointTrajectoryPoint');
punkter = repmat(punkt, numel(t_send), 1);
for k = 1:numel(t_send)
    punkter(k).positions = [q_send(k,:), q_send(k,3), q_send(k,3), q_send(k,3)].';   % + teleskoptrinn 2-4
    punkter(k).velocities = zeros(0,1);      % tomme felt, ellers avviser kontrolleren
    punkter(k).accelerations = zeros(0,1);
    punkter(k).effort = zeros(0,1);
    punkter(k).time_from_start = ros2duration(t_send(k));
end
maal.trajectory.joint_names = cellstr([ledd(:); trinn(:)]);
maal.trajectory.points = punkter;
fprintf('Sender %d punkter (%.0f s simtid) til Gazebo ...\n', numel(t_send), t_send(end));
handtak = sendGoal(klient, maal);

%% 6) Logg referanse og maalt posisjon mens roboten kjorer
logg = zeros(0, 19);
klokke = tic; sist = 0;
while true
    status = getStatus(klient, handtak);
    if any(status == [4 5 6]), break; end
    try
        m = receive(tilstand, 2);
        [ok, j] = ismember(ledd, string(m.joint_names));
        if ~all(ok), continue; end
        if isfield(m,'reference'), ref = m.reference.positions; else, ref = m.desired.positions; end
        if isfield(m,'feedback'),  akt = m.feedback.positions;  else, akt = m.actual.positions;  end
        if numel(ref) < max(j) || numel(akt) < max(j), continue; end
        ts = double(m.header.stamp.sec) + double(m.header.stamp.nanosec)*1e-9;
        logg(end+1,:) = [ts, double(ref(j(:)))', double(akt(j(:)))']; %#ok<SAGROW>
    catch e
        if ~contains(e.message, 'receive') && ~contains(e.message, 'timed out'), rethrow(e); end
    end
    if toc(klokke) - sist > 30 && ~isempty(logg)
        sist = toc(klokke);
        fprintf('  %.0f / %.0f s simtid\n', logg(end,1) - logg(1,1), t_send(end));
    end
    if toc(klokke) > 3*t_send(end) + 120, error('Tidsavbrudd mens MATLAB ventet paa Gazebo.'); end
end
resultat = getResult(klient, handtak);
if status ~= 4 || resultat.error_code ~= 0
    error('Gazebo avbrot banen: status=%d, kode=%d (%s).', status, ...
          resultat.error_code, string(resultat.error_string));
end

%% 7) Resultat: planlagt mot maalt
[spiss_ref, ~]    = dyse_fra_gazebo(logg(:,2:10),  base);
[spiss_akt, akse] = dyse_fra_gazebo(logg(:,11:19), base);
avvik = vecnorm(spiss_akt - spiss_ref, 2, 2);
fprintf('Dysespiss, folgeavvik: maks %.1f mm, RMS %.1f mm (%d maalinger).\n', ...
        1000*max(avvik), 1000*sqrt(mean(avvik.^2)), numel(avvik));

loggfil = sprintf('gazebo_vaskebane_%s.csv', string(datetime('now','Format','yyyyMMdd_HHmmss')));
writetable(array2table(logg, 'VariableNames', cellstr(["sim_time_s","ref_"+ledd,"actual_"+ledd])), loggfil);
fprintf('Logg lagret: %s\n', loggfil);

% Hvor straalen treffer glasset (y = 0), maalt i Gazebo, mot planlagt treffpunkt
L = -spiss_akt(:,2) ./ akse(:,2); L(akse(:,2) <= 1e-6 | L > res.c.jetRange) = NaN;
treff = spiss_akt + L.*akse;
figure('Name','Vaskebane: plan mot Gazebo','Color','w'); hold on; grid on; axis equal
rectangle('Position',[0 0 10 10],'EdgeColor','k','LineWidth',1.5);
for w = 0:9, xline(w,':'); yline(w,':'); end
vask = bane(:,2) == 2;
plot(bane(vask,17), bane(vask,18), '.', 'Color',[.6 .6 .6], 'MarkerSize',5, 'DisplayName','Planlagt treff');
plot(treff(:,1), treff(:,3), '.', 'Color',[.1 .35 .85], 'MarkerSize',3, 'DisplayName','Treff fra maalte ledd i Gazebo');
xlim([0 10]); ylim([0 10]); xlabel('Vegg X [m]'); ylabel('Vegg Z [m]');
legend('Location','eastoutside'); title('Straalens treffpunkt paa glasset');

ts = logg(:,1) - logg(1,1);
figure('Name','Folgeavvik','Color','w');
subplot(2,1,1); plot(ts, 1000*avvik); grid on; ylabel('Spissavvik [mm]');
title('Avvik mellom referanse og maalt dysespiss');
subplot(2,1,2); plot(ts, logg(:,7), '--', ts, logg(:,16), '-'); grid on
xlabel('Simtid [s]'); ylabel('arm\_joint\_2 [rad]'); legend('ref','maalt'); title('Skulderledd');

%% Lokale funksjoner
function q_gz = full_til_gazebo(q)
% `full`-ledd (N x 9) -> Gazebo-ledd (N x 9)
    q_gz = [q(:,1:2), (q(:,3) - 3.6)/4, q(:,4:9)];
end

function q = gazebo_til_full(q_gz)
    q = [q_gz(:,1:2), 3.6 + 4*q_gz(:,3), q_gz(:,4:9)];
end

function T = fk_full(q, base)
% Samme DH og verktoy som `full` i kinematics_model.m (uten Toolbox)
    d  = [0, 0, q(3), 0, 0.089159, 0, 0, 0.10915, 0.09465];
    a  = [0, 0, 0, 0, 0, -0.425, -0.39225, 0, 0];
    al = [pi/2, pi/2, pi/2, -pi/2, pi/2, 0, 0, -pi/2, pi/2];
    th = [q(1), q(2), 0, q(4), q(5:9)];
    T = eye(4); T(1:3,4) = base(:);
    for j = 1:9
        ct = cos(th(j)); st = sin(th(j)); ca = cos(al(j)); sa = sin(al(j));
        T = T * [ct -st*ca st*sa a(j)*ct; st ct*ca -ct*sa a(j)*st; 0 sa ca d(j); 0 0 0 1];
    end
    T = T * [eye(3) [0; 0; 0.0823 + 0.15]; 0 0 0 1];
end

function [P, Z] = dyse_fra_gazebo(q_gz, base)
% Dysespiss og dyseakse (z9) i MATLAB-verden fra Gazebo-ledd
    Q = gazebo_til_full(q_gz); P = zeros(size(Q,1),3); Z = P;
    for n = 1:size(Q,1)
        T = fk_full(Q(n,:), base); P(n,:) = T(1:3,4)'; Z(n,:) = T(1:3,3)';
    end
end

function sjekk_modell(full)
% Gazebo-URDF-en er bygd paa denne DH-tabellen. Endres `full`, maa URDF-en endres ogsaa.
    B = full.base; if isa(B,'SE3'), B = B.T; end
    rng(1); verst = 0;
    for k = 1:20
        t2 = pi/2 + rand*pi/2;
        q = [2*pi*rand-pi, t2, 3.6 + 10*rand, t2 - pi/2, 2*pi*rand(1,5) - pi];
        T = full.fkine(q); if isa(T,'SE3'), T = T.T; end
        verst = max(verst, norm(T - fk_full(q, B(1:3,4)'), 'fro'));
    end
    if verst > 1e-6
        error(['`full` stemmer ikke med DH-tabellen Gazebo-modellen er bygd paa (avvik %.3g). ' ...
               'Er DH-parametere, offset eller dyselengde endret i kinematics_model.m?'], verst);
    end
end

function sjekk_base_i_gazebo(node, base)
% Leser world -> base_footprint fra /tf_static og sammenligner med full.base
    try
        sub = ros2subscriber(node, '/tf_static', 'tf2_msgs/TFMessage', ...
                             'Durability','transientlocal', 'Reliability','reliable');
        m = receive(sub, 10);
        for tr = m.transforms(:)'
            if strcmp(tr.child_frame_id, 'base_footprint')
                v = tr.transform.translation;
                gz = double([v.x + 0.7, v.y, v.z + 1.5]);   % yaw-aksen: base_footprint + [0.7 0 1.5]
                if norm(gz - base) > 1e-3
                    melding = sprintf(['Liftbasen i Gazebo er [%g %g %g], men full.base er ' ...
                        '[%g %g %g]. Basen er fast i URDF-en, saa full.base maa vaere [-3 -2 1.5].'], ...
                        gz, base);
                    error('vask:base', '%s', melding);
                end
                fprintf('Basen i Gazebo stemmer med full.base [%g %g %g].\n', base);
                return
            end
        end
        error('base_footprint ble ikke funnet i /tf_static');
    catch e
        if strcmp(e.identifier, 'vask:base'), rethrow(e); end
        melding = sprintf('Kunne ikke lese basen fra /tf_static (%s). Sjekk selv at Gazebo-basen er [%g %g %g].', ...
                          e.message, base);
        warning('vask:tf', '%s', melding);
    end
end
