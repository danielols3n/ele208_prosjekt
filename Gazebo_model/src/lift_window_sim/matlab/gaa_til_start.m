%% Declaration of AI use
% This file is partly made with support from Claude Code
% All code is written manually, but with support

function gaa_til_start(testnavn, varighet)
% Kjører roboten i Gazebo til første punkt i banen for valgt test, slik at
% kjor_test finner roboten i startposen. Trengs fordi Gazebo starter
% teleskopet inntrukket (boom_extension = 0) i stedet for i startverdien.
% Bruk: gaa_til_start("last"), gaa_til_start("top") eller gaa_til_start("vinkel")
arguments
    testnavn (1,1) string {mustBeMember(testnavn,["last","top","vinkel"])} = "top"
    varighet (1,1) double {mustBePositive} = 30   % simuleringstid [s] for bevegelsen
end

mappe = fileparts(fileparts(mfilename('fullpath')));
ledd = ["boom_yaw","boom_pitch","boom_extension","arm_swivel", ...
        "arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"];
if testnavn == "vinkel"
    kilde = fullfile(mappe,'trajectories','angles.csv');
else
    kilde = fullfile(mappe,'trajectories','demo.csv');
end
tabell = readtable(kilde);
q0 = table2array(tabell(1,cellstr(ledd)));

% Samme ROS_DOMAIN_ID som i kjor_test.m
setenv("ROS_DOMAIN_ID","24")
node = ros2node('/matlab_gaa_til_start');
leddsub = ros2subscriber(node,'/joint_states','sensor_msgs/JointState');
[klient,maal] = ros2actionclient(node, ...
    '/arm_controller/follow_joint_trajectory', ...
    'control_msgs/FollowJointTrajectory');
if ~waitForServer(klient,'Timeout',30)
    error('Gazebo-kontrolleren svarer ikke. Sjekk at arm_controller er aktiv.');
end

punkt = ros2message('trajectory_msgs/JointTrajectoryPoint');
punkt.positions = double([q0, q0(3), q0(3), q0(3)]).';
punkt.velocities = zeros(0,1);
punkt.accelerations = zeros(0,1);
punkt.effort = zeros(0,1);
punkt.time_from_start = ros2duration(varighet);
maal.trajectory.joint_names = cellstr([ledd(:); "stage2_slide"; "stage3_slide"; "stage4_slide"]);
maal.trajectory.points = punkt;

fprintf('Kjører til startposen for "%s" over %.0f s simuleringstid ...\n',testnavn,varighet);
handtak = sendGoal(klient,maal);
start = tic;
while true
    status = getStatus(klient,handtak);
    if any(status == [4 5 6]), break; end
    if toc(start) > 3*varighet + 120
        error('Tidsavbrudd mens roboten kjørte til startposen.');
    end
    pause(0.5)
end
if status ~= 4
    % 6 = avbrutt av arm_controller, typisk fordi et ledd ikke nådde målet innen goal_time
    warning('Gazebo-action ble ikke fullført (status=%d). Viser hvor leddene står likevel.',status);
end

% Vent litt så leddene får satt seg, og sjekk avviket mot startposen
pause(3)
m = receive(leddsub,10);
[finnes,indeks] = ismember(ledd,string(m.name));
if ~all(finnes), error('Mangler ledd i /joint_states.'); end
maalt = double(m.position(indeks(:)))';
avvik = abs(maalt - q0);
toleranse = [0.08 0.08 0.02 0.08 0.08 0.08 0.08 0.08 0.08];   % samme som i kjor_test.m
disp(table(ledd(:),q0(:),maalt(:),avvik(:),avvik(:) <= toleranse(:), ...
    'VariableNames',{'ledd','start','maalt','avvik','ok'}))
if all(avvik <= toleranse)
    fprintf('Roboten står i startposen. Kjør nå kjor_test("%s").\n',testnavn);
else
    warning('Noen ledd er fortsatt utenfor toleransen. Vent litt og kjør gaa_til_start på nytt.');
end
end
