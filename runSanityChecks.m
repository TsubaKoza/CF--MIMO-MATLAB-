function report = runSanityChecks()
%RUNSANITYCHECKS Execute required Phase 1-10 deterministic checks.
fprintf('Running Cell-Free 3-D channel-estimation sanity checks...\n');
base=configChannelSim3D(struct('showFigures',false,'saveFigures',false, ...
    'saveResults',false,'runMonteCarlo',false,'runPaperValidation',false, ...
    'N_AP',1,'numObstacles',0,'enableObstacles',false,'tauP',1, ...
    'pilotAssignmentMode',"orthogonal"));
passed=false(1,10); details=strings(1,10);

%% Phase 1 - Test 1: zero-noise 1 AP / 1 UE LS recovery.
cfg=base; cfg.M=1; cfg.K=1;
scenario=manualScenario([0 0 10],[30 20 1.5],[],cfg);
[h,pathInfo]=generateRayTracingChannels3D(scenario,cfg); %#ok<ASGLU>
[p,~]=generatePilots(cfg,1,scenario.UEpos);
Y=simulatePilotReception(h,p,1,0,true);
hhat=estimateChannelLS(Y,p,1,true);
relativeError=norm(hhat-h)/max(norm(h),eps);
assert(relativeError<1e-11,'Test 1 failed: zero-noise LS mismatch.');
passed(1)=true; details(1)=sprintf('relative error %.3g',relativeError);
fprintf('Phase 1 / Test 1 passed: %s\n',details(1));

%% Phase 2 - physical 4-element ULA phase check (additional phase gate).
cfg2=cfg; cfg2.N_AP=4; cfg2.arraySpacing=cfg2.lambda/2;
scenario2=manualScenario([0 0 10],[30 20 1.5],[],cfg2);
[h2,~]=generateRayTracingChannels3D(scenario2,cfg2);
u=computeAoAAoD3D(scenario2.APpos,scenario2.UEpos);
expectedRatio=exp(-1j*2*pi/cfg2.lambda*cfg2.arraySpacing* ...
    dot(cfg2.ulaDirection,u.direction));
assert(max(abs(h2(2:end)./h2(1:end-1)-expectedRatio))<1e-10, ...
    'Phase 2 ULA phase check failed.');
fprintf('Phase 2 passed: physical ULA phase progression verified.\n');

%% Phase 3 - Test 3: orthogonal pilots remove other-UE interference.
cfg3=cfg2; cfg3.K=2; cfg3.tauP=2; cfg3.pilotAssignmentMode="orthogonal";
scenario3=manualScenario([0 0 10],[30 0 1.5;0 30 1.5],[],cfg3);
[h3,~]=generateRayTracingChannels3D(scenario3,cfg3);
[p3,~]=generatePilots(cfg3,2,scenario3.UEpos);
Y3=simulatePilotReception(h3,p3,1,0,[true true]);
hhat3=estimateChannelLS(Y3,p3,1,[true true]);
orthError=norm(hhat3(:)-h3(:))/norm(h3(:));
assert(orthError<1e-11,'Test 3 failed: orthogonal interference remains.');
passed(3)=true; details(3)=sprintf('relative error %.3g',orthError);
fprintf('Phase 3 / Test 3 passed: %s\n',details(3));

%% Phase 4 - Test 4: pilot reuse creates contamination.
cfg4=cfg3; cfg4.tauP=1; cfg4.pilotAssignmentMode="randomReuse";
[p4,~]=generatePilots(cfg4,2,scenario3.UEpos);
Y4=simulatePilotReception(h3,p4,1,0,[true true]);
hhat4=estimateChannelLS(Y4,p4,1,[true true]);
contamination=norm(hhat4(:,1,1)-h3(:,1,1));
assert(contamination>1e-12,'Test 4 failed: pilot contamination not detected.');
passed(4)=true; details(4)=sprintf('contamination norm %.3g',contamination);
fprintf('Phase 4 / Test 4 passed: %s\n',details(4));

%% Phase 5 - Tests 5 and 6: true 3-D cuboid blocking.
block=makeObstacle([5 0 2.5],2,2,5,0,0.6);
[availableLow,~]=checkLoS3D([0 0 1],[10 0 1],block,cfg);
assert(~availableLow,'Test 5 failed: building-crossing line was not blocked.');
passed(5)=true; details(5)="low path blocked";
[availableHigh,~]=checkLoS3D([0 0 10],[10 0 10],block,cfg);
assert(availableHigh,'Test 6 failed: over-roof path was incorrectly blocked.');
passed(6)=true; details(6)="over-roof path available";
fprintf('Phase 5 / Tests 5-6 passed: height-aware 3-D blocking verified.\n');

%% Phase 6 - Tests 7 and 8: reflection coefficient and blocked-LoS reflection.
reflector0=makeObstacle([5 5 5],4,2,10,0,0);
cfgR=cfg2; cfgR.enableObstacles=true; cfgR.enableRoofReflection=true;
scenarioR=manualScenario([0 0 5],[10 0 5],reflector0,cfgR);
[hR,infoR]=generateRayTracingChannels3D(scenarioR,cfgR);
reflectionPaths=infoR(1,1).paths([infoR(1,1).paths.type]=="reflection");
assert(~isempty(reflectionPaths),'Test 7 setup failed: no reflection found.');
assert(all(arrayfun(@(x) norm(x.contribution),reflectionPaths)<1e-15), ...
    'Test 7 failed: Gamma=0 reflection is nonzero.');
passed(7)=true; details(7)="Gamma=0 gives zero reflected contribution";
blocker=makeObstacle([5 0 5],2,2,10,0,0.6);
reflector=makeObstacle([5 5 5],4,2,10,0,0.6);
scenarioBR=manualScenario([0 0 5],[10 0 5],[blocker reflector],cfgR);
[hBR,infoBR]=generateRayTracingChannels3D(scenarioBR,cfgR);
assert(~infoBR(1,1).hasLoS && infoBR(1,1).numReflections>0 && norm(hBR)>0, ...
    'Test 8 failed: blocked LoS with available reflection was not represented.');
passed(8)=true; details(8)="LoS=0 and reflected channel nonzero";
fprintf('Phase 6 / Tests 7-8 passed: reflection physics gates verified.\n');

%% Phase 6 extension - knife-edge diffraction gates.
cfgD=cfg2; cfgD.enableObstacles=true;
cfgD.enableSideReflection=false; cfgD.enableRoofReflection=false;
cfgD.enableDiffraction=true;
scenarioD=manualScenario([0 0 5],[10 0 5],blocker,cfgD);
[hD,infoD]=generateRayTracingChannels3D(scenarioD,cfgD);
typesD=[infoD(1,1).paths.type];
assert(~infoD(1,1).hasLoS && any(typesD=="diffraction") && norm(hD)>0, ...
    'Diffraction gate failed: blocked LoS did not produce a diffracted path.');
cfgDoff=cfgD; cfgDoff.enableDiffraction=false;
[hDoff,~]=generateRayTracingChannels3D(scenarioD,cfgDoff);
assert(norm(hDoff)<1e-15, ...
    'Diffraction gate failed: disabled diffraction produced a channel.');
path3=infoD(1,1).paths(find(typesD=="diffraction",1));
cfgD100=cfgD; cfgD100.fc=100e9; cfgD100.lambda=cfgD100.c/cfgD100.fc;
cfgD100.arraySpacing=cfgD100.lambda/2;
scenarioD100=manualScenario([0 0 5],[10 0 5],blocker,cfgD100);
[~,infoD100]=generateRayTracingChannels3D(scenarioD100,cfgD100);
typesD100=[infoD100(1,1).paths.type];
path100=infoD100(1,1).paths(find(typesD100=="diffraction",1));
assert(path100.diffractionLossdB>path3.diffractionLossdB, ...
    'Diffraction gate failed: 100 GHz loss should exceed 3 GHz loss.');
fprintf(['Phase 6 diffraction extension passed: blocked-link recovery and ' ...
    'frequency-loss trend verified.\n']);

%% Phase 7 - Tests 9 and 10: obstacle-off direct path and reproducibility.
scenarioEmpty=manualScenario([0 0 10],[30 20 1.5],[],cfg2);
[~,infoEmpty]=generateRayTracingChannels3D(scenarioEmpty,cfg2);
assert(infoEmpty(1,1).hasLoS,'Test 9 failed: obstacle-free LoS missing.');
passed(9)=true; details(9)="obstacle-off LoS available";
cfg10=configChannelSim3D(struct('M',3,'K',4,'N_AP',2,'numObstacles',2, ...
    'tauP',2,'pilotAssignmentMode',"randomReuse",'showFigures',false));
rng(cfg10.seed,'twister'); a=runSingleSimulation3D(cfg10);
rng(cfg10.seed,'twister'); b=runSingleSimulation3D(cfg10);
assert(isequaln(a.APpos,b.APpos) && isequaln(a.UEpos,b.UEpos) && ...
    isequaln(a.h_true,b.h_true) && isequaln(a.h_hat_LS,b.h_hat_LS), ...
    'Test 10 failed: identical seed did not reproduce the result.');
passed(10)=true; details(10)="identical seed reproduces geometry and result";
fprintf('Phase 7 / Tests 9-10 passed: obstacle-off and reproducibility verified.\n');

%% Phase 8 - Test 2: Monte-Carlo noise average decreases with SNR.
snrValues=[-10 0 10]; R=300; curve=zeros(size(snrValues));
for s=1:numel(snrValues)
    local=zeros(R,1);
    nv=computeNoiseVariance(h2,1,snrValues(s),cfg2);
    for r=1:R
        rng(cfg2.seed+1000*s+r,'twister');
        Y=simulatePilotReception(h2,ones(1,1),1,nv,true);
        hh=estimateChannelLS(Y,ones(1,1),1,true);
        [~,local(r)]=computeNMSE(hh,h2,cfg2.nmseEpsilon);
    end
    curve(s)=mean(local);
end
assert(all(diff(curve)<0),'Test 2 failed: MC NMSE is not decreasing with SNR.');
passed(2)=true; details(2)=sprintf('NMSE [%s]',num2str(curve,'%.3g '));
fprintf('Phase 8 / Test 2 passed: %s\n',details(2));

%% Phase 9 - clustered reuse extension validity.
cfg9=cfg; cfg9.tauP=2; cfg9.clusterRadius=60;
u9=[0 0 1.5;1 0 1.5;100 0 1.5;101 0 1.5];
a9=assignPilots("clusteredReuse",2,4,u9,cfg9);
assert(a9(1)~=a9(2) && a9(3)~=a9(4),'Phase 9 clustered-reuse gate failed.');
fprintf('Phase 9 passed: nearby UE pairs receive different greedy colors.\n');

%% Phase 10 - paper stochastic Eq. (5)-(7) validation.
cfg10p=cfg; cfg10p.paperValidationTrials=5000;
v=validatePilotPaperModel(cfg10p);
assert(v.relativeError<0.08,'Phase 10 paper MMSE validation mismatch.');
fprintf('Phase 10 passed: Eq. (7) predicted/empirical MSE relative error %.3g.\n',v.relativeError);

assert(all(passed),'One or more required sanity checks did not run.');
report=struct('passed',passed,'details',details,'phase10',v);
fprintf('All 10 required sanity checks and all Phase 1-10 gates passed.\n');
end

function scenario=manualScenario(APpos,UEpos,obstacles,cfg)
scenario=struct('APpos',APpos,'UEpos',UEpos,'obstacles',obstacles, ...
    'antennaPos',generateAPArrayPositions(APpos,cfg),'mode',"manual", ...
    'region',cfg.region);
end

function o=makeObstacle(center,w,d,h,yaw,gamma)
o=struct('center',center,'width',w,'depth',d,'height',h,'yaw',yaw, ...
    'reflectionCoefficient',gamma,'active',true);
end
