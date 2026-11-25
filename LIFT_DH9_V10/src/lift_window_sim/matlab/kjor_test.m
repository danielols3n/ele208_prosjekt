function loggfil = kjor_test(testnavn)
% Sender banepunkter fra MATLAB til Gazebo med ROS 2
% Bruk for testing av systemet: kjor_test("last"), kjor_test("top") eller kjor_test("vinkel").

arguments
    testnavn (1,1) string {mustBeMember(testnavn,["last","top","vinkel"])}
end

mappe = fileparts(fileparts(mfilename('fullpath')));
ledd = ["boom_yaw","boom_pitch","boom_extension","arm_swivel", ...
        "arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"];
% Leddgrensene er ikke symmetriske for boom_pitch og boom_extension.
nedre = [-3, -1.48, 0, -1.57, -6.28, -3.14, -2.95, -6.28, -3.14];
ovre  = [ 3,  1.00, 2.3, 1.57,  6.28,  3.14,  2.95,  6.28,  3.14];
vmax = [0.15, 0.12, 0.12, 0.5, 0.7, 0.7, 0.7, 1, 1];

if testnavn == "vinkel"
    kilde = fullfile(mappe,'trajectories','angles.csv');
else
    kilde = fullfile(mappe,'trajectories','demo.csv');
end
% Jeg brukte KI til å lage
% forslaget til banepunktene som leses fra CSV her. Det var nice og
% effekitvt! Og forstå hvordan det gjorde det
tabell = readtable(kilde);
q0 = table2array(tabell(1,cellstr(ledd)));

if testnavn == "last"
    % Hold startstillingen i 60 sekunder med last i Gazebo-verdenen. For
    % lastetesten
    tid = [25;60];
    q = [q0;q0];
else
    % MATLAB fyller inn punkter mellom de lagrede leddstillingene.
    vei = table2array(tabell(:,cellstr(ledd)));
    tVei = tabell.time_s;
    if any(diff(tVei) <= 0) || abs(tVei(1)) > 1e-9
        error('Ugyldige tidsstempler i bane-CSV.');
    end
    tMidt = (tVei(1:end-1)+tVei(2:end))/2;
    tInterpolert = sort([tVei;tMidt]);
    q = interp1(tVei,vei,tInterpolert,'linear');
    tid = tInterpolert + 0.5;
    % Jeg fikk KI-veiloedning og forklaring til å kontrollere farten mellom banepunktene.
    hastighet = abs(diff(q)./diff(tid));
    faktor = max([1, max(hastighet./vmax,[],'all')/0.85]);
    tid = 0.5 + (tid-0.5)*faktor;
end
if any(~isfinite(q),'all') || any(q < nedre-1e-6 | q > ovre+1e-6,'all')
    error('Baneverdier er ugyldige eller utenfor leddgrensene.');
end

node = ros2node('/matlab_lift_test');
sub = ros2subscriber(node,'/joint_trajectory_controller/controller_state', ...
                     'control_msgs/JointTrajectoryControllerState');
leddsub = ros2subscriber(node,'/joint_states','sensor_msgs/JointState');
[klient,maal] = ros2actionclient(node, ...
    '/joint_trajectory_controller/follow_joint_trajectory', ...
    'control_msgs/FollowJointTrajectory');
if ~waitForServer(klient,'Timeout',30)
    error('Gazebo-kontrolleren svarer ikke. Vent til TOP READY.');
end
startmaal = receive(leddsub,10);
[finnes,indeks] = ismember(ledd, string(startmaal.name));
if ~all(finnes)
    error('Mangler ledd i /joint_states.');
end
startfeil = abs(double(startmaal.position(indeks(:)))' - q0);
toleranse = [0.08 0.08 0.02 0.08 0.08 0.08 0.08 0.08 0.08];
if any(startfeil(:)' > toleranse)
    error('Robot er ikke ved banens startpose. Start Gazebo paa nytt.');
end

punkt = ros2message('trajectory_msgs/JointTrajectoryPoint');
punkter = repmat(punkt,numel(tid),1);
for k = 1:numel(tid)
    punkter(k).positions = double(q(k,:)).';
    % Her tømmes feltene for hastighet, akselerasjon og kraft.
    punkter(k).velocities = zeros(0,1);
    punkter(k).accelerations = zeros(0,1);
    punkter(k).effort = zeros(0,1);
    if numel(punkter(k).positions) ~= numel(ledd) || ...
            ~isempty(punkter(k).velocities) || ...
            ~isempty(punkter(k).accelerations) || ~isempty(punkter(k).effort)
        error('Banepunkt %d har feil antall leddverdier.',k);
    end
    punkter(k).time_from_start = ros2duration(tid(k));
end
% Jeg brukte hjelp/debugging fra KI til å feilsøke når tidsstemplene og sette opp sjekkingen, av at hvert banepunkt har et senere tidspunkt enn det forrige.
meldingTid = arrayfun(@(p) double(p.time_from_start.sec) + ...
    double(p.time_from_start.nanosec)*1e-9,punkter);
if any(~isfinite(meldingTid)) || any(diff(meldingTid) <= 0)
    error('Banepunktene har ikke strengt stigende ROS-tider.');
end
maal.trajectory.joint_names = cellstr(ledd(:));
maal.trajectory.points = punkter;

fprintf('MATLAB sender %d punkter, %.1f s simuleringstid.\n',numel(tid),tid(end));
handtak = sendGoal(klient,maal);
% Jeg brukte veiledning av KI til å sette opp logging av ønskede og målte leddstillinger.
logg = zeros(0,19);
folgere = zeros(0,7);
siste = tic;
while true
    status = getStatus(klient,handtak);
    if any(status == [4 5 6])
        break
    end
    try
        m = receive(sub,2);
        navn = string(m.joint_names);
        [ok,j] = ismember(ledd,navn);
        if ~all(ok), error('Uventet leddliste i controller_state.'); end
        if isfield(m,'reference'), ref = m.reference.positions;
        else, ref = m.desired.positions; end
        if isfield(m,'feedback'), faktisk = m.feedback.positions;
        else, faktisk = m.actual.positions; end
        if numel(ref) < max(j) || numel(faktisk) < max(j), continue; end
        klokke = double(m.header.stamp.sec)+double(m.header.stamp.nanosec)*1e-9;
        logg(end+1,:) = [klokke,double(ref(j(:)))',double(faktisk(j(:)))']; %#ok<AGROW>
    catch e
        if ~contains(e.message,'receive') && ~contains(e.message,'timed out')
            rethrow(e)
        end
    end
    if testnavn == "last"
        try
            jm = receive(leddsub,0.05);
            fn = ["boom_extension","stage2_slide","stage3_slide", ...
                  "stage4_slide","stage5_slide","stage6_slide"];
            [okf,jf] = ismember(fn,string(jm.name));
            if all(okf)
                ts = double(jm.header.stamp.sec)+double(jm.header.stamp.nanosec)*1e-9;
                folgere(end+1,:) = [ts,double(jm.position(jf(:)))']; %#ok<AGROW>
            end
        catch
            % Ingen ny joint_states-melding under denne korte avlesningen.
        end
    end
    if toc(siste) > max(90,tid(end)*3+90)
        error('Tidsavbrudd mens MATLAB ventet paa action-resultat.');
    end
end
resultat = getResult(klient,handtak);
if status ~= 4 || resultat.error_code ~= 0
    error('Gazebo-action feilet: status=%d, kode=%d.',status,resultat.error_code);
end
if isempty(logg)
    error('Action ble fullfort, men ingen controller_state ble logget.');
end

navn = ["sim_time_s","ref_"+ledd,"actual_"+ledd];
loggfil = fullfile(pwd, sprintf('matlab_%s_%s.csv',testnavn,datestr(now,'yyyymmdd_HHMMSS')));
writetable(array2table(logg,'VariableNames',cellstr(navn)),loggfil);
if testnavn == "last" && ~isempty(folgere)
    [dirnavn,basenavn] = fileparts(loggfil);
    folgerfil = fullfile(dirnavn,string(basenavn)+"_folgere.csv");
    fnavn = ["sim_time_s","boom_extension","stage2_slide", ...
             "stage3_slide","stage4_slide","stage5_slide","stage6_slide"];
    writetable(array2table(folgere,'VariableNames',cellstr(fnavn)),folgerfil);
    fprintf('Teleskopledd: %s\n',folgerfil);
end
fprintf('Action fullfort. Maalinger: %s\n',loggfil);
end
