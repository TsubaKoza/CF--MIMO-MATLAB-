function stats = runMonteCarlo3D(cfg,pilotMode,obstacleEnabled)
%RUNMONTECARLO3D Average link NMSE over random geometry/noise realizations.
if nargin<2, pilotMode=cfg.pilotAssignmentMode; end
if nargin<3, obstacleEnabled=cfg.enableObstacles; end
snrValues = cfg.pilotSNRdB(:).'; R = cfg.numRealizations;
meanPerRealization = NaN(R,numel(snrValues));
for r=1:R
    rng(cfg.seed+r-1,'twister');
    localCfg = cfg;
    localCfg.enableObstacles = obstacleEnabled;
    localCfg.pilotAssignmentMode = pilotMode;
    scenario = generateScenario3D(localCfg);
    [h_true,pathInfo] = generateRayTracingChannels3D(scenario,localCfg);
    K = size(h_true,3);
    activeUE = selectPilotUsers(localCfg.pilotUserMode,pathInfo,K);
    pilotEnergy = localCfg.pilotEnergy*ones(1,K);
    for s=1:numel(snrValues)
        localCfg.currentPilotSNRdB = snrValues(s);
        % Separate deterministic substream for pilots/noise at each point.
        rng(localCfg.seed+100000*r+1000*s,'twister');
        [p,~] = generatePilots(localCfg,K,scenario.UEpos);
        noiseVariance = computeNoiseVariance(h_true,pilotEnergy,snrValues(s),localCfg);
        Y = simulatePilotReception(h_true,p,pilotEnergy,noiseVariance,activeUE);
        hhat = estimateChannelLS(Y,p,pilotEnergy,activeUE);
        [~,meanPerRealization(r,s)] = computeNMSE(hhat,h_true,localCfg.nmseEpsilon);
    end
end
stats = struct('pilotMode',string(pilotMode),'obstacleEnabled',obstacleEnabled, ...
    'snrdB',snrValues,'meanNMSE',mean(meanPerRealization,1,'omitnan'), ...
    'meanNMSEdB',10*log10(max(mean(meanPerRealization,1,'omitnan'),eps)), ...
    'perRealization',meanPerRealization,'numRealizations',R);
end
