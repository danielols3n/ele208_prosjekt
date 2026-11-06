function res = window_washer_planner(robot, opt)
%WINDOW_WASHER_PLANNER Collision-checked motion planning for the lift + 5-axis UR5 window cleaner.
%   res = window_washer_planner(full)        plans with the robot model from the main script
%   res = window_washer_planner(full, opt)   opt can set: nSpots, randomSeed, showAnimation, exportCSV
%   Base, DH parameters, joint limits and tool (nozzle) are all read from the SerialLink model,
%   which must use the main script's convention q = [t1 t2 d3 t2-pi/2 q_arm(1:5)].
%   res.traj(:,4:12) can therefore be passed straight to full.fkine / full.rne / ROS.
%
%   Endring: bommen kan gå inntil 15 grader under horisontal (q2 >= 75 deg, leses fra qlim),
%   og platen plasseres over lave flekker (aldri under c.mountMinZ). Da når armen nederste rad.
if nargin<2, opt=struct(); end
assert(isa(robot,'SerialLink') && robot.n==9, 'robot must be the 9-joint lift+arm SerialLink (full).');

%% Robot model, read from the caller's SerialLink (single source of truth)
B=robot.base; if isa(B,'SE3'), B=B.T; end
Tt=robot.tool; if isa(Tt,'SE3'), Tt=Tt.T; end
assert(norm(B(1:3,1:3)-eye(3))<1e-9, 'Base must be a pure translation.');
assert(norm(Tt(1:3,1:3)-eye(3))<1e-9 && norm(Tt(1:2,4))<1e-9, 'Tool must be a pure z-offset (nozzle).');
c.robot=robot; c.basePosition=B(1:3,4)'; c.toolLen=Tt(3,4);
qlim=[-pi pi; 0 pi; 0 inf; -inf inf; repmat([-2*pi 2*pi],5,1)];   % defaults where links have none
for j=1:9
    lk=robot.links(j);
    % [theta d a alpha sigma]; the joint variable (theta or d) can be stored empty -> 0
    c.dh(j,:)=[nz(lk.theta) nz(lk.d) nz(lk.a) nz(lk.alpha) double(lk.isprismatic())];
    c.offset(j)=nz(lk.offset);
    if ~isempty(lk.qlim), qlim(j,:)=lk.qlim; end
end
c.lo=qlim([1:3 5:9],1)'; c.hi=qlim([1:3 5:9],2)';           % u = [q1 q2 d3 q5..q9], q4 follows q2

%% Parameters (everything the helper functions need lives in struct c)
c.motionStyle     = 'cartesian';      % 'cartesian' or 'joint'
c.wallW = 10; c.wallH = 10;
c.workingStandoff = 1.0;              % lift-mount distance from wall while cleaning (0.75 + nozzle)
c.contactY        = 0;                % glass plane is y = 0
armPark = [0 -pi/2 pi/2 0 0];
% Water jet: the planner targets where the jet hits the glass, the nozzle sits jetLength back along it
c.wallNormal = [0;1;0];               % into the wall
c.zDes = c.wallNormal;                % desired nozzle axis, updated per spot (tilted for low spots)
c.jetRange   = 0.7;                   % effective jet length [m]
c.jetOptions = [0.15 0.25 0.35 0.45 0.55 0.65 0.70];      % jet lengths tried (nozzle -> hit point)
c.tiltBelowZ = 0.7;                   % patches reaching below this height are sprayed from above
c.lowTilt    = deg2rad(43);           % downward tilt for those (43 deg + IK tolerance < 45 deg limit)
c.lowStandoff = 0.8;                  % lift-mount distance from wall for those
c.mountMinZ  = 0.9;                   % lift mount (plate) never below this height [m] (camera 0.3 m below)
c.lowMountAbove = 0.3;                % for low patches the mount is placed this far above the spot [m]
c.oriWeight = 0.5;                    % weight of orientation error vs position error in IK [m/rad]
c.angTol    = deg2rad(2);             % IK accepts solutions within 2 deg of the desired direction
c.maxSpray  = deg2rad(45);            % requirement: spray within +-45 deg of the wall normal
c.nCol = 18;                          % trajectory columns: ... spray angle [deg], jet hit x, jet hit z
c.lo(2) = max(c.lo(2), deg2rad(75));  % boom at most 15 deg below horizontal (also set as qlim in main script)

nSpots=getOpt(opt,'nSpots',20); randomSeed=getOpt(opt,'randomSeed',7);
spotPlacement='spread';                                    % 'spread' or 'random'
patchW=0.50; patchH=0.50; lineSpacing=0.12; paneMargin=0.06;
[IZ,IX]=meshgrid(0:9,0:9);
c.windows=[IX(:) IZ(:) ones(100,2)];                       % [x z w h], 10x10 panes of 1x1 m

fastMode=true;
c.maxArmIter=45; if ~fastMode, c.maxArmIter=120; end
c.maxArmCollisionSeeds=6;
c.tol=0.015;
c.liftStep=0.20; c.armStep=0.035; c.maxSamples=2000; c.dt=0.1;
c.retract=0.30; c.transferY=-1.30;
c.frameHalfWidth=0.018; c.frameDepth=0.060;
c.radii=[0.065 0.035 0.035 0.035 0.035 0.035 0.012]+0.010; % boom, 5 UR links, nozzle (+ safety margin)
c.checkGround=true;                   % boom can now go below horizontal, so check the ground
c.angStep=0.10; c.extStep=0.10; c.surfTol=0.015;
c.phaseCol ={[.9 .4 .1],[.02 .54 .26],[.55 .25 .80],[.65 .17 .55]};
c.phaseName={'Lift transfer','Cleaning','Approach','Retract'};
showAnimation=getOpt(opt,'showAnimation',true); playbackPause=0.025; plotEvery=3; followRadius=1.45;
showEntirePath=true; exportCSV=getOpt(opt,'exportCSV',true);

W=c.windows;
assert(all(W(:,3:4)>2*paneMargin,'all') && all(W(:,1:2)>=0,'all') && ...
    all(W(:,1)+W(:,3)<=c.wallW+1e-9) && all(W(:,2)+W(:,4)<=c.wallH+1e-9), ...
    'Every pane must fit inside the wall and be larger than 2*paneMargin.');

%% Dirty spots: one local cleaning patch per spot
rng(randomSeed); nWin=size(W,1);
switch lower(spotPlacement)
    case 'spread', paneID=repmat(randperm(nWin)',ceil(nSpots/nWin),1); paneID=paneID(1:nSpots);
    case 'random', paneID=randi(nWin,nSpots,1);
    otherwise, error('spotPlacement must be spread or random.');
end
half=[patchW patchH]/2;
spots=zeros(nSpots,2); patches=zeros(nSpots,4);            % spots [x z], patches [xmin xmax zmin zmax]
for k=1:nSpots
    pmin=W(paneID(k),1:2)+paneMargin; pmax=W(paneID(k),1:2)+W(paneID(k),3:4)-paneMargin;
    a=pmin+half; b=pmax-half; mid=(pmin+pmax)/2;
    tight=a>b; a(tight)=mid(tight); b(tight)=mid(tight);
    s=a+(b-a).*rand(1,2); spots(k,:)=s;
    bl=max(pmin,s-half); tr=min(pmax,s+half); patches(k,:)=[bl(1) tr(1) bl(2) tr(2)];
end
order=nearestTwoOpt(spots,c.basePosition([1 3]));
routeDist=sum(vecnorm(diff([c.basePosition([1 3]); spots(order,:)]),2,2));
fprintf('Generated %d spots across %d panes; approximate wall route %.2f m.\n',nSpots,nWin,routeDist);

figure('Name','Window spots and visit order','Color','w'); drawWall(c,[.55 .67 .78],[.93 .97 1]);
plot(spots(order,1),spots(order,2),'-o','Color',[.15 .35 .85],'LineWidth',1.5,'MarkerFaceColor',[1 .55 .1]);
for k=1:nSpots
    p=patches(order(k),:);
    rectangle('Position',[p(1) p(3) p(2)-p(1) p(4)-p(3)],'EdgeColor',[.15 .65 .3],'LineStyle','--');
end
text(spots(order,1)+0.04,spots(order,2),string(1:nSpots)','FontSize',8);
title('Numbered visit order and local cleaning patches');

%% Plan: retract -> lift transfer -> approach -> raster at first feasible depth -> retract
tic; traj=zeros(0,c.nCol); t=0; status={'UNSOLVED','COLLISION-CHECKED COMPLETE'};
complete=false(nSpots,1); usedJet=nan(nSpots,1); tiltDeg=zeros(nSpots,1); failure=repmat({''},nSpots,1);
u=min(max([0 pi/2 4 armPark],c.lo),c.hi);
[ok,why]=collisionFree(u,c,0,false);
if ~ok, error('Initial robot configuration is in collision: %s.',why); end
for visit=1:nSpots
    id=order(visit);
    tip=tipPos(u,c);                                        % 1) pull tool back before moving the lift
    if tip(2)>-c.retract+c.surfTol
        [chunk,u2,t2,r]=moveArm(u,[tip(1);-c.retract;tip(3)],4,id,c,t,'cartesian',true);
        if ~isempty(r)
            failure{id}=['Cannot retract before lift: ' r];
            warning('Planning stopped: tool could not be safely retracted.'); break;
        end
        traj=[traj;chunk]; u=u2; t=t2; %#ok<AGROW>
    end
    low=patches(id,3)<c.tiltBelowZ;                         % low patch: spray down from above
    standoff=c.workingStandoff; if low, standoff=c.lowStandoff; end
    mountZ=max(spots(id,2),c.mountMinZ);                    % 2) mount height: at the spot, never below mountMinZ
    if low, mountZ=max(spots(id,2)+c.lowMountAbove,c.mountMinZ); end   % low patch: mount above, arm reaches down
    [chunk,u2,t2,r]=moveLift(u,[spots(id,1);-standoff;mountZ],id,c,t);
    if ~isempty(r)
        failure{id}=['Lift transfer rejected: ' r];
        fprintf('Visit %d spot %d: %s\n',visit,id,failure{id});
        warning('Stopping after unsafe transfer. No further moves were committed.'); break;
    end
    traj=[traj;chunk]; u=u2; t=t2; %#ok<AGROW>
    tilt=0; jets=c.jetOptions;                              % 3) jet direction for this patch
    if low, tilt=c.lowTilt; jets=fliplr(jets); end          % longest jet first for low patches
    c.zDes=[0; cos(tilt); -sin(tilt)]; tiltDeg(id)=rad2deg(tilt);
    raster=verticalRaster(patches(id,:),lineSpacing);       % jet hit points on the glass [x z]
    hit0=[raster(1,1); c.contactY; raster(1,2)];
    [chunk,u2,t2,r]=moveArm(u,hit0-(c.retract/cos(tilt))*c.zDes,3,id,c,t,'cartesian',false);
    if ~isempty(r)
        failure{id}=['Clearance approach rejected: ' r]; fprintf('Spot %d: %s\n',id,failure{id}); continue;
    end
    traj=[traj;chunk]; u=u2; t=t2; %#ok<AGROW>
    reasons={};                                             % 4) first jet length with a full valid sequence
    for jet=jets
        H=[raster(:,1) repmat(c.contactY,size(raster,1),1) raster(:,2)];
        goals=H-jet*c.zDes';                                % nozzle tip = hit point - jet * direction
        [chunk,u2,t2,r]=cleanPatch(u,t,goals,id,paneID(id),c);
        if isempty(r)
            traj=[traj;chunk]; u=u2; t=t2; usedJet(id)=jet; complete(id)=true; %#ok<AGROW>
            fprintf('Spot %d solved: jet %.2f m, tilt %.0f deg.\n',id,jet,rad2deg(tilt)); break;
        end
        reasons{end+1}=sprintf('jet %.2f m %s',jet,r); %#ok<AGROW>
    end
    if ~complete(id), failure{id}=strjoin(reasons,' | '); end
    fprintf('Visit %d/%d, spot %d: %s\n',visit,nSpots,id,status{complete(id)+1});
end
fprintf('Planning %.2f s; collision-checked completed %d/%d spots.\n',toc,sum(complete),nSpots);

mountPath=zeros(size(traj,1),6);
for i=1:size(traj,1)
    P=framePoints(traj(i,4:12),c); mountPath(i,:)=[traj(i,1:3) P(:,5)'];
end

%% Result plots
figure('Name','Full solved Cartesian path on windows','Color','w'); drawWall(c,[.70 .75 .80]);
plot(spots(order,1),spots(order,2),':','Color',[.6 .6 .6],'DisplayName','Visit order (nominal)');
if ~isempty(traj)
    plot(traj(:,13),traj(:,15),'-','Color',[.76 .76 .76],'LineWidth',.6,'DisplayName','Entire sampled tool trace');
    for ph=1:4, plotPhase([],traj,ph,c,1.3+0.3*(ph==2)); end
end
scatter(spots(:,1),spots(:,2),32,[.7 .1 .1],'filled','DisplayName','Target spots');
text(spots(order,1)+.045,spots(order,2),string(1:nSpots)','FontSize',8);
title('Validated sampled route: transfers, approaches, retracts and cleaning'); legend('Location','eastoutside');

figure('Name','Full 3D Cartesian paths','Color','w'); hold on; grid on; axis equal; view(40,20);
if ~isempty(traj)
    plot3(traj(:,13),traj(:,14),traj(:,15),'-','Color',[.76 .76 .76],'LineWidth',.6,'DisplayName','Entire sampled tool trace');
    for ph=1:4, plotPhase(gca,traj,ph,c,1.3+0.3*(ph==2)); end
    plot3(mountPath(:,4),mountPath(:,5),mountPath(:,6),'Color',[.25 .45 .85],'DisplayName','Lift mount path');
end
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); legend('Location','best');
title('Collision-checked FK trace of tool and lift mount');

figure('Name','Spot cleaning result','Color','w'); drawWall(c,[.65 .75 .83]);
for k=1:nSpots
    p=patches(k,:); col=[1 .78 .78]; if complete(k), col=[.80 .94 .80]; end
    rectangle('Position',[p(1) p(3) p(2)-p(1) p(4)-p(3)],'FaceColor',col,'EdgeColor',[.35 .5 .35]);
    if complete(k)
        text(spots(k,1)+0.06,spots(k,2),sprintf('%.0f cm, %.0f°',100*usedJet(k),tiltDeg(k)),'FontSize',7,'Color',[.05 .4 .12]);
    end
end
if ~isempty(traj), m=traj(:,2)==2; plot(traj(m,17),traj(m,18),'.','Color',[.1 .3 .8],'MarkerSize',5); end
plot(spots(:,1),spots(:,2),'k.');
title(sprintf('Planned cleaning: %d/%d patches solved (red=incomplete)',sum(complete),nSpots));

if any(traj(:,2)==2)                                        % orientation requirement check
    m=traj(:,2)==2;
    figure('Name','Spray angle','Color','w'); plot(traj(m,1),traj(m,16),'.','MarkerSize',4); grid on; hold on;
    yline(45,'r--','Krav: 45°'); xlabel('t (s)'); ylabel('Vinkel fra veggnormal (°)');
    title(sprintf('Dysevinkel under vasking: maks %.1f°',max(traj(m,16))));
end

%% Export and animation
if exportCSV
    visitNo=zeros(nSpots,1); visitNo(order)=1:nSpots;
    writetable(table((1:nSpots)',paneID,visitNo,spots(:,1),spots(:,2),patches(:,1),patches(:,2), ...
        patches(:,3),patches(:,4),complete,usedJet,tiltDeg,failure,'VariableNames', ...
        {'spot_id','pane_id','visit_order','x_m','z_m','x_min','x_max','z_min','z_max', ...
        'planned_complete','jet_length_m','tilt_deg','failure_reason'}),'window_spots_route_collision.csv');
    if ~isempty(traj)
        writetable(array2table(mountPath,'VariableNames',{'time_s','phase','spot_id', ...
            'mount_x_m','mount_y_m','mount_z_m'}),'window_lift_mount_collision.csv');
        writetable(array2table(traj,'VariableNames',{'time_s','phase','spot_id','q1_rad','q2_rad', ...
            'd3_m','q4_rad','q5_rad','q6_rad','q7_rad','q8_rad','q9_rad','tool_x_m','tool_y_m','tool_z_m','spray_angle_deg','hit_x_m','hit_z_m'}), ...
            'window_spot_collision_trajectory.csv');
    end
end
if showAnimation && ~isempty(traj)
    playRobotViews(traj,c,plotEvery,playbackPause,followRadius,showEntirePath);
end

%% Results returned to the caller
res=struct('traj',traj,'mountPath',mountPath,'spots',spots,'patches',patches, ...
    'paneID',paneID,'order',order,'complete',complete,'jetLength',usedJet,'tiltDeg',tiltDeg,'c',c);
res.failure=failure;                                       % cell array, set separately (struct() would expand it)
end

function v=getOpt(opt,name,default)                        % optional field with default value
    if isfield(opt,name), v=opt.(name); else, v=default; end
end

function v=nz(x)                                           % empty -> 0 (Link stores the joint variable as [])
    if isempty(x), v=0; else, v=x(1); end
end

%% ===================== Kinematics =====================
function q=fullQ(u)                 % 8 controlled joints -> 9 joints, liftQ: q4 = q2 - pi/2 (plate vertical)
    q=[u(1) u(2) u(3) u(2)-pi/2 u(4:8)];
end

function [P,zt]=framePoints(q,c)    % hand-written DH forward kinematics: frame origins + nozzle tip
    T=eye(4); T(1:3,4)=c.basePosition(:); P=zeros(3,11); P(:,1)=T(1:3,4);
    for j=1:9
        th=c.dh(j,1); d=c.dh(j,2); a=c.dh(j,3); al=c.dh(j,4);
        if c.dh(j,5), d=q(j); else, th=q(j)+c.offset(j); end
        T=T*[cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
             sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
             0        sin(al)          cos(al)         d;
             0        0                0               1];
        P(:,j+1)=T(1:3,4);
    end
    zt=T(1:3,3);                                            % nozzle axis = z9 (tool is a pure z-offset)
    P(:,11)=T(1:3,4)+zt*c.toolLen;                          % nozzle tip
end

function [p,z]=tipPos(u,c), [P,z]=framePoints(fullQ(u),c); p=P(:,11); end
function p=mountPos(u,c),   P=framePoints(fullQ(u),c);     p=P(:,5);  end
function a=angTo(z,v), a=acos(max(-1,min(1,z'*v))); end   % angle between nozzle axis and direction v [rad]

function z=solveLiftIK(c,target)
    % closed-form lift IK (same as ik_system): 0p4 = d3*[c1*s2; s1*s2; -c2]
    % boom length is clamped to its limits - the lift only coarse-positions, the arm reaches the rest
    % boom angle is clamped too (max 15 deg below horizontal); then the horizontal reach is kept
    p=target(:)-c.basePosition(:); h=hypot(p(1),p(2));
    q2=atan2(h,-p(3)); d3=norm(p);
    if q2<c.lo(2) || q2>c.hi(2), q2=min(max(q2,c.lo(2)),c.hi(2)); d3=h/sin(q2); end
    z=[atan2(p(2),p(1)) q2 min(max(d3,c.lo(3)),c.hi(3))];
end

function [z,err,ok]=solveArmIK(c,liftU,seed,target)
    % damped least-squares IK for the 5 arm joints, lift held fixed:
    % 3 position rows + nozzle pointing (z9 -> zDes), 5 joints for 5 constraints
    lo=c.lo(4:8); hi=c.hi(4:8); z=min(max(seed,lo),hi); w=c.oriWeight;
    for i=1:c.maxArmIter
        [p,zt]=tipPos([liftU z],c); ep=target-p;
        if norm(ep)<c.tol && angTo(zt,c.zDes)<c.angTol, break; end
        e=[ep; w*cross(zt,c.zDes)];                         % rotating z toward zDes needs omega ~ z x zDes
        J=c.robot.jacob0(fullQ([liftU z])); J=[J(1:3,5:9); w*J(4:6,5:9)];
        dz=J'*((J*J'+0.06^2*eye(6))\e);
        z=min(max(z+max(min(dz,0.18),-0.18)',lo),hi);
    end
    [p,zt]=tipPos([liftU z],c); err=norm(target-p);
    ok=err<c.tol && angTo(zt,c.zDes)<c.angTol;
end

%% ===================== Motion primitives =====================
function P=waypoints(p0,p1,step,c,style)
    if ~strcmpi(style,'cartesian'), P=p1(:)'; return; end
    n=max(1,ceil(norm(p1-p0)/step));
    assert(n<=c.maxSamples,'Cartesian segment too long. Increase step.');
    P=p0(:)'+((1:n)'/n)*(p1(:)-p0(:))';
end

function n=nSweeps(u0,u1,c)
    d=abs(u1-u0); n=max([1 ceil(max(d([1:2 4:8]))/c.angStep) ceil(d(3)/c.extStep)]);
end

function [chunk,t,reason]=addSweep(chunk,u0,u1,phase,id,c,t,goalY)
    % sample the joint interpolation u0->u1; while cleaning, the nozzle must keep its wall distance,
    % spray within +-45 deg of the wall normal, and the jet must reach the glass (<= jetRange)
    reason=''; n=nSweeps(u0,u1,c);
    for s=1:n
        us=u0+(u1-u0)*(s/n); [p,zt]=tipPos(us,c); ang=angTo(zt,c.wallNormal);
        hit=[NaN;NaN;NaN]; L=Inf;
        if zt(2)>1e-6, L=(c.contactY-p(2))/zt(2); hit=p+L*zt; end   % jet axis meets glass plane
        if phase==2
            if abs(p(2)-goalY)>c.surfTol, reason='joint interpolation departed from intended standoff'; return; end
            if ang>c.maxSpray, reason=sprintf('spray angle %.0f deg exceeds limit',rad2deg(ang)); return; end
            if L>c.jetRange+c.surfTol, reason=sprintf('jet %.2f m longer than its range',L); return; end
        end
        t=t+c.dt; chunk(end+1,:)=[t phase id fullQ(us) p' rad2deg(ang) hit(1) hit(3)]; %#ok<AGROW>
    end
end

function [chunk,u,t,reason]=moveLift(u0,goal,id,c,t0)
    % straight lift move; if blocked, go out to transferY, across, and back in
    [chunk,u,t,reason]=moveLiftDirect(u0,goal,id,c,t0);
    if isempty(reason), return; end
    first=reason; p0=mountPos(u0,c); y=min([c.transferY p0(2) goal(2)]);
    G=[p0(1) y p0(3); goal(1) y goal(3); goal(:)'];
    chunk=zeros(0,c.nCol); u=u0; t=t0;
    for j=1:3
        if norm(G(j,:)'-mountPos(u,c))<c.tol, continue; end
        [part,u,t,r]=moveLiftDirect(u,G(j,:)',id,c,t);
        if ~isempty(r)
            chunk=zeros(0,c.nCol); u=u0; t=t0;
            reason=sprintf('direct: %s | clearance stage %d: %s',first,j,r); return;
        end
        chunk=[chunk;part]; %#ok<AGROW>
    end
    reason='';
end

function [chunk,u,t,reason]=moveLiftDirect(u,goal,id,c,t)
    chunk=zeros(0,c.nCol); reason='';
    targets=waypoints(mountPos(u,c),goal,c.liftStep,c,c.motionStyle);
    for k=1:size(targets,1)
        cand=[solveLiftIK(c,targets(k,:)') u(4:8)];
        [ok,why]=sweptSafe(u,cand,c,0,false);
        if ~ok, reason=sprintf('lift sweep %d: %s',k,why); return; end
        [chunk,t]=addSweep(chunk,u,cand,1,id,c,t,NaN); u=cand;
    end
end

function [chunk,u,t,reason]=moveArm(u,goal,phase,id,c,t,style,contact,pane)
    if nargin<9, pane=0; end
    chunk=zeros(0,c.nCol); reason='';
    seeds=[0 -pi/2 pi/2 0 0; 0 -pi/3 pi/2 -pi/2 0; 0 -2*pi/3 pi/3 0 pi/2;
           pi/2 -pi/2 pi/2 0 0; -pi/2 -pi/2 pi/2 0 0];
    targets=waypoints(tipPos(u,c),goal,c.armStep,c,style);
    for k=1:size(targets,1)
        trials=[u(4:8); seeds]; ok=false; why='no IK solution'; err=Inf;
        for a=1:min(c.maxArmCollisionSeeds,size(trials,1))   % first collision-free IK solution wins
            [armU,e,ikOK]=solveArmIK(c,u(1:3),trials(a,:),targets(k,:)');
            err=min(err,e); if ~ikOK, continue; end
            cand=[u(1:3) armU]; [ok,why]=sweptSafe(u,cand,c,pane,contact);
            if ok, break; end
        end
        if ~ok
            reason=sprintf('arm waypoint %d rejected after %d seeds: %s (IK error %.3f m)',k,a,why,err); return;
        end
        [chunk,t,reason]=addSweep(chunk,u,cand,phase,id,c,t,goal(2));
        if ~isempty(reason), return; end
        u=cand;
    end
end

function [chunk,u,t,r]=cleanPatch(u,t,goals,id,pane,c)
    % approach first raster point, run every stroke, retract - all or nothing
    chunk=zeros(0,c.nCol); n=size(goals,1);
    for s=1:n+1
        if s==1,     g=goals(1,:)'; ph=3; style='cartesian';   what='approach';
        elseif s<=n, g=goals(s,:)'; ph=2; style=c.motionStyle; what=sprintf('stroke %d',s);
        else, tip=tipPos(u,c); g=[tip(1);min(tip(2),-c.retract);tip(3)]; ph=4; style='cartesian'; what='retract';
        end
        [part,u,t,r]=moveArm(u,g,ph,id,c,t,style,true,pane);
        if ~isempty(r), r=[what ': ' r]; return; end
        chunk=[chunk;part]; %#ok<AGROW>
    end
end

function path=verticalRaster(p,pitch)                      % p = [xmin xmax zmin zmax]
    xs=linspace(p(1),p(2),max(2,ceil((p(2)-p(1))/pitch)+1)); path=zeros(2*numel(xs),2);
    for k=1:numel(xs)
        z=p(3:4); if mod(k,2)==0, z=fliplr(z); end
        path(2*k-1:2*k,:)=[xs(k) z(1); xs(k) z(2)];
    end
end

function order=nearestTwoOpt(pts,start)                    % nearest neighbour + 2-opt
    n=size(pts,1); left=true(n,1); order=zeros(n,1); last=start;
    for k=1:n
        d=sum((pts-last).^2,2); d(~left)=Inf; [~,i]=min(d);
        order(k)=i; left(i)=false; last=pts(i,:);
    end
    if n<3, return; end
    for pass=1:8
        improved=false;
        for a=1:n-1
            for b=a+1:n
                R=[start; pts(order,:)];                    % R(a)=point before a, R(a+1)=order(a)
                old=norm(R(a,:)-R(a+1,:)); new=norm(R(a,:)-R(b+1,:));
                if b<n, old=old+norm(R(b+1,:)-R(b+2,:)); new=new+norm(R(a+1,:)-R(b+2,:)); end
                if new<old-1e-9, order(a:b)=flipud(order(a:b)); improved=true; end
            end
        end
        if ~improved, break; end
    end
end

%% ===================== Collision checking =====================
function [ok,why]=sweptSafe(u0,u1,c,pane,contact)
    n=nSweeps(u0,u1,c);
    for k=0:n
        [ok,why]=collisionFree(u0+(u1-u0)*(k/n),c,pane,contact);
        if ~ok, why=sprintf('sample %d/%d: %s',k,n,why); return; end
    end
end

function [ok,why]=collisionFree(u,c,paneID,contact)
    % links as capsules: wall/glass, window frames, ground (optional), self-collision
    ok=false; why='';
    if any(u<c.lo-1e-8 | u>c.hi+1e-8), why='Joint limit'; return; end
    P=framePoints(fullQ(u),c);
    A=[P(:,1) P(:,5:10)]; B=[P(:,5) P(:,6:11)];              % boom + 5 UR segments + nozzle (j=7)
    pane=zeros(1,4); if paneID>=1, pane=c.windows(paneID,:); end
    Wn=c.windows; x0=Wn(:,1); x1=x0+Wn(:,3); z0=Wn(:,2); z1=z0+Wn(:,4);
    for j=1:7
        r=c.radii(j); d=B(:,j)-A(:,j); if norm(d)<1e-6, continue; end
        fw=c.frameHalfWidth+r; ns=max(2,ceil(norm(d)/0.035));
        for k=0:ns
            p=A(:,j)+d*(k/ns);
            if c.checkGround && p(3)<r && p(1)>-0.5 && p(1)<c.wallW+0.5
                why=sprintf('link %d near ground',j); return;
            end
            if p(1)<-r || p(1)>c.wallW+r || p(3)<-r || p(3)>c.wallH+r, continue; end
            onGlass=j==7 && contact && paneID>0 && p(1)>=pane(1)+fw && p(1)<=pane(1)+pane(3)-fw ...
                && p(3)>=pane(2)+fw && p(3)<=pane(2)+pane(4)-fw;
            if onGlass
                if p(2)>c.contactY+c.surfTol, why='Tool tip penetrates glass'; return; end
            elseif p(2)>-r
                why=sprintf('link %d intersects glass',j); return;
            end
            if p(2)>-c.frameDepth-r
                inBox=p(1)>=x0-fw & p(1)<=x1+fw & p(3)>=z0-fw & p(3)<=z1+fw;
                onRail=abs(p(1)-x0)<=fw | abs(p(1)-x1)<=fw | abs(p(3)-z0)<=fw | abs(p(3)-z1)<=fw;
                w=find(inBox & onRail,1);
                if ~isempty(w), why=sprintf('link %d hits window frame %d',j,w); return; end
            end
        end
    end
    pairs=nchoosek(2:7,2); pairs=[pairs(pairs(:,2)-pairs(:,1)>=2,:); [1 4;1 5;1 6;1 7]];
    for pr=pairs'                                           % non-adjacent arm/nozzle pairs + boom vs arm
        a=pr(1); b=pr(2);
        if norm(B(:,a)-A(:,a))<1e-6 || norm(B(:,b)-A(:,b))<1e-6, continue; end
        if segmentGap(A(:,a),B(:,a),A(:,b),B(:,b))<c.radii(a)+c.radii(b)
            why=sprintf('collision between link %d and link %d',a,b); return;
        end
    end
    ok=true;
end

function d=segmentGap(p1,q1,p2,q2)                         % closest distance between two segments
    d1=q1-p1; d2=q2-p2; r=p1-p2;
    a=d1'*d1; e=d2'*d2; f=d2'*r; cc=d1'*r; b=d1'*d2; den=a*e-b^2;
    s=0; if den>1e-12, s=min(max((b*f-cc*e)/den,0),1); end
    t=(b*s+f)/e;
    if t<0,     t=0; s=min(max(-cc/a,0),1);
    elseif t>1, t=1; s=min(max((b-cc)/a,0),1); end
    d=norm(p1+d1*s-p2-d2*t);
end

%% ===================== Plotting =====================
function drawWall(c,edge,face)
    if nargin<3, face='none'; end
    hold on; grid on; axis equal;
    rectangle('Position',[0 0 c.wallW c.wallH],'EdgeColor','k','LineWidth',2);
    for j=1:size(c.windows,1), rectangle('Position',c.windows(j,:),'EdgeColor',edge,'FaceColor',face); end
    xlim([0 c.wallW]); ylim([0 c.wallH]); xlabel('Wall X (m)'); ylabel('Wall Z (m)');
end

function plotPhase(ax,traj,phase,c,width)                  % ax = [] -> 2D wall view, else 3D
    i=find(traj(:,2)==phase); if isempty(i), return; end
    P=traj(i,13:15); if isempty(ax) && phase==2, P=[traj(i,17) zeros(numel(i),1) traj(i,18)]; end
    P([false; diff(i)>1 | diff(traj(i,3))~=0],:)=NaN;
    opts={'Color',c.phaseCol{phase},'LineWidth',width,'DisplayName',c.phaseName{phase}};
    if isempty(ax), plot(P(:,1),P(:,3),opts{:}); else, plot3(ax,P(:,1),P(:,2),P(:,3),opts{:}); end
end

function playRobotViews(traj,c,every,pauseT,R,showAll)
    fig=figure('Color','w','Name','Window cleaner | lift vs arm','Position',[100 100 1330 620]);
    ax=[subplot(1,2,1) subplot(1,2,2)]; titles={'Full wall and lift','Cleaning arm (tracking close-up)'};
    [hl,ha,hm,ht,hk]=deal(gobjects(1,2));
    for v=1:2
        a=ax(v); hold(a,'on'); grid(a,'on'); axis(a,'equal'); view(a,37,18); title(a,titles{v});
        xlabel(a,'Wall X (m)'); ylabel(a,'Depth Y (m)'); zlabel(a,'Height Z (m)');
        surf(a,[0 c.wallW;0 c.wallW],zeros(2),[0 0;c.wallH c.wallH],'FaceColor',[.72 .85 .96], ...
            'FaceAlpha',.13,'EdgeColor','none');
        plot3(a,[0 c.wallW c.wallW 0 0],zeros(1,5),[0 0 c.wallH c.wallH 0],'k-','LineWidth',1.2);
        plot3(a,c.basePosition(1),c.basePosition(2),c.basePosition(3),'ks','MarkerFaceColor',[.3 .3 .3],'MarkerSize',7);
        hl(v)=plot3(a,nan,nan,nan,'-','Color',[.63 .67 .72],'LineWidth',1.6);
        ha(v)=plot3(a,nan,nan,nan,'-o','Color',[.03 .43 .8],'MarkerFaceColor',[.12 .65 .94],'LineWidth',3,'MarkerSize',5);
        hm(v)=plot3(a,nan,nan,nan,'s','Color',[.8 .35 .08],'MarkerFaceColor',[1 .65 .2],'MarkerSize',8);
        ht(v)=plot3(a,nan,nan,nan,'p','Color',[0 .45 .19],'MarkerFaceColor',[.19 .82 .35],'MarkerSize',12);
        hk(v)=plot3(a,nan,nan,nan,'-','Color',[.02 .7 .36],'LineWidth',1.1);
    end
    xlim(ax(1),[-5 c.wallW+1]); ylim(ax(1),[-5 2]); zlim(ax(1),[-1 c.wallH+2]);
    m=traj(:,2)==2; plot3(ax(1),traj(m,13),traj(m,14),traj(m,15),'.','Color',[.62 .82 .68],'MarkerSize',2);
    if showAll, for ph=1:4, plotPhase(ax(1),traj,ph,c,1.0); end, end
    trace=zeros(0,3); prev=-1;
    for i=unique([1:every:size(traj,1) size(traj,1)])
        if ~ishandle(fig), return; end
        P=framePoints(traj(i,4:12),c); mnt=P(:,5); tool=P(:,11);
        if traj(i,3)~=prev, trace=zeros(0,3); prev=traj(i,3); end
        if traj(i,2)==2, trace(end+1,:)=[traj(i,17) 0 traj(i,18)]; end %#ok<AGROW>   % where the jet hits
        set(hl,'XData',[P(1,1) mnt(1)],'YData',[P(2,1) mnt(2)],'ZData',[P(3,1) mnt(3)]);
        set(ha,'XData',P(1,5:11),'YData',P(2,5:11),'ZData',P(3,5:11));
        set(hm,'XData',mnt(1),'YData',mnt(2),'ZData',mnt(3));
        set(ht,'XData',tool(1),'YData',tool(2),'ZData',tool(3));
        set(hk,'XData',trace(:,1),'YData',trace(:,2),'ZData',trace(:,3));
        xlim(ax(2),mnt(1)+[-R R]); ylim(ax(2),mnt(2)+[-R R]); zlim(ax(2),mnt(3)+[-R R]);
        title(ax(2),sprintf('Spot %d | %s | %.1f s',traj(i,3),c.phaseName{traj(i,2)},traj(i,1)));
        drawnow limitrate; if pauseT>0, pause(pauseT); end
    end
end
