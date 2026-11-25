function resultat = vurder_logg(loggfil,testnavn)
% Regner ut avvik, veggtreff og dekning fra målt leddlogg.
arguments
    loggfil (1,1) string
    testnavn (1,1) string {mustBeMember(testnavn,["last","top","vinkel"])}
end
d = readtable(loggfil);
ledd = ["boom_yaw","boom_pitch","boom_extension","arm_swivel", ...
        "arm_joint_1","arm_joint_2","arm_joint_3","arm_joint_4","arm_joint_5"];
qr = table2array(d(:,cellstr("ref_"+ledd)));
qa = table2array(d(:,cellstr("actual_"+ledd)));
t = d.sim_time_s;
if numel(t) < 3 || any(diff(t) <= 0)
    error('Mangler maalt tidsserie, eller Gazebo-klokken er nullstilt under testen.');
end
resultat.test = char(testnavn);
resultat.samples = numel(t);
resultat.duration_s = t(end)-t(1);
resultat.joint_max_error = max(abs(qa-qr),[],1);
resultat.boom_extension_error_m = resultat.joint_max_error(3);
resultat.method = 'Gazebo controller_state; TCP beregnet fra maalte ledd';
resultat.nozzle_mass_kg = 3;
resultat.supported_hose_mass_kg = 20;
resultat.configured_force_N = [40 0 0];
resultat.base_anchored = true;

if testnavn == "last"
    slutt = t >= t(end)-10;
    mappePakke = fileparts(fileparts(mfilename('fullpath')));
    startTabell = readtable(fullfile(mappePakke,'trajectories','demo.csv'));
    startq = table2array(startTabell(1,cellstr(ledd)));
    e = max(abs(qa(slutt,:)-startq),[],1);
    resultat.final_10s_max_error = e;
    [mappe,navn] = fileparts(loggfil);
    folgerfil = fullfile(mappe,string(navn)+"_folgere.csv");
    if ~isfile(folgerfil)
        error('Mangler joint_states-logg for teleskopleddene.');
    end
    f = readtable(folgerfil);
    f = f(f.sim_time_s >= t(end)-10,:);
    if isempty(f), error('Mangler teleskopmaalinger i siste 10 s.'); end
    qf = table2array(f(:,2:7));
    resultat.max_follower_error_m = max(abs(qf(:,2:6)-qf(:,1)),[],'all');
    resultat.pose_hold_screen_pass = all(e([1,2,4:9]) < .1) && ...
        e(3) < .025 && resultat.max_follower_error_m < .025;
else
    posfeil = zeros(numel(t),1);
    % Jeg brukte KI til å sette opp sammenligningen av ønsket posisjon og
    % posisjonen som
    % DH-modellen beregner fra målte leddverdier. For å vite HVORDAN mann
    % skulle gjøre det!
    for k = 1:numel(t)
        Tr = kinematikk9(qr(k,:)); Ta = kinematikk9(qa(k,:));
        posfeil(k) = norm(Ta(1:3,4)-Tr(1:3,4));
    end
    resultat.max_tcp_tracking_error_m = max(posfeil);
    resultat.rms_tcp_tracking_error_m = sqrt(mean(posfeil.^2));
    utvalg = unique([1:5:numel(t),numel(t)]);
    referanse = zeros(numel(utvalg),2);
    treff = zeros(numel(utvalg),2);
    retning_ref = zeros(numel(utvalg),2);
    retning = zeros(numel(utvalg),2);
    for k = 1:numel(utvalg)
        Tr = kinematikk9(qr(utvalg(k),:));
        Ta = kinematikk9(qa(utvalg(k),:));
        for typ = 1:2
            if typ == 1, T = Tr; else, T = Ta; end
            v = T(1:3,1);
            if v(2) <= .05, error('Dysen peker ikke mot veggen.'); end
            % Jeg fikk KI til å forklare HVORDAN strålens retning kan
            % brukes til å finne veggtreffet, og fikk hjelp til koden når det ikke funket/error.
            p = T(1:3,4)+v*((1.97-T(2,4))/v(2));
            vinkler = atan2d([v(1),v(3)],v(2));
            if typ == 1
                referanse(k,:) = p([1 3])'; retning_ref(k,:) = vinkler;
            else
                treff(k,:) = p([1 3])'; retning(k,:) = vinkler;
            end
        end
    end
    resultat.max_wall_hit_tracking_error_m = max(vecnorm(treff-referanse,2,2));
    if testnavn == "top"
        % Jeg brukte KI til å forstå utviklingen/sjekking av avstanden fra hvert rutenettpunkt til banen.
        % Et punkt regnes som dekket innenfor halv sprøytebredde.
        [gx,gz] = meshgrid(0:.05:10,9:.05:10);
        g = [gx(:),gz(:)];
        avstand = zeros(size(g,1),1);
        for k = 1:200:size(g,1)
            r = k:min(k+199,size(g,1));
            dx = g(r,1)-treff(:,1)'; dz = g(r,2)-treff(:,2)';
            avstand(r) = sqrt(min(dx.^2+dz.^2,[],2));
        end
        resultat.grid_points = numel(avstand);
        resultat.grid_spacing_m = .05;
        resultat.assumed_effective_spray_swath_m = .13;
        resultat.max_grid_distance_to_path_m = max(avstand);
        resultat.grid_covered_at_assumed_swath = all(avstand <= .13/2);
    else
        % Jeg brukte KI til å sjekke HVORDAN hvert vinkelmål kobles
        % til nærmeste punkt i referansebanen og få hjelp med dt.
        maal = [-45 0;45 0;0 -45;0 45];
        resultat.target_deg = maal;
        resultat.actual_deg = zeros(4,2);
        resultat.target_error_deg = zeros(4,1);
        resultat.wall_hit_error_m = zeros(4,1);
        for n = 1:4
            [~,idx] = min(vecnorm(retning_ref-maal(n,:),2,2));
            resultat.actual_deg(n,:) = retning(idx,:);
            resultat.target_error_deg(n) = norm(retning(idx,:)-maal(n,:));
            resultat.wall_hit_error_m(n) = norm(treff(idx,:)-[1 9.5]);
        end
        resultat.all_four_targets_within_1deg = all(resultat.target_error_deg < 1);
    end
end
resultat.caveat = 'Sproeytebredde antatt; kraft ikke separat maalt; fast base og posisjonsstyring.';
[mappe,navn] = fileparts(loggfil);
utfil = fullfile(mappe,string(navn)+"_analyse.json");
fid = fopen(utfil,'w');
if fid < 0, error('Kunne ikke skrive analysefil.'); end
c = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(resultat,'PrettyPrint',true));
fprintf('Analyse lagret: %s\n',utfil);
disp(resultat);
end
