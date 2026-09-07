function cfg = configChannelSim3D(varargin)
%CONFIGCHANNELSIM3D Configuration for the 3-D Cell-Free channel simulator.
% All entries marked Assumption/Extension are not claimed to be from either
% reference paper.

cfg.seed = 20260901;
cfg.c = 3e8;
cfg.fc = 3e9;
cfg.lambda = cfg.c/cfg.fc;
cfg.region = [0 200; 0 200; 0 30];
cfg.scenarioMode = "random3D"; % or "rayTracingPaperReference"
cfg.channelModel = "RayTracingHybrid";

cfg.M = 5;
cfg.K = 10;
cfg.N_AP = 4;
cfg.zAP = 10;
cfg.zUE = 1.5;
cfg.minAPUEDistance = 2;
cfg.arraySpacing = cfg.lambda/2;
cfg.ulaDirection = [1 0 0];

cfg.numObstacles = 5;
cfg.obstacleWidthRange = [10 40];
cfg.obstacleDepthRange = [10 40];
cfg.obstacleHeightRange = [5 25];
cfg.obstacleYawRangeDeg = [0 360];
cfg.obstacleReflectionMagnitudeRange = [0.4 0.8];
cfg.obstacleClearance = 1;
cfg.maxPlacementAttempts = 5000;
cfg.enableObstacles = true;
cfg.enableSideReflection = true;
cfg.enableRoofReflection = true;
cfg.enableGroundReflection = false;
cfg.enableDiffraction = true;
cfg.diffractionModel = "singleKnifeEdgeITU";
cfg.diffractionEdges = "topAndVertical";
cfg.maxDiffractionsPerObstacle = 1;
cfg.diffractionOnlyWhenLoSBlocked = true;
cfg.diffractionLossCapdB = 200;

cfg.pathGainModel = "friisField";
cfg.minPathLength = 1e-3;
cfg.epsGeom = 1e-9;
cfg.segmentEndpointMargin = 1e-7;
cfg.nmseEpsilon = 1e-18;

cfg.pilotAssignmentMode = "randomReuse";
cfg.pilotUserMode = "allUE";
cfg.tauP = 5;
cfg.pilotEnergy = 1;
cfg.currentPilotSNRdB = 10;
cfg.pilotSNRdB = [-10 -5 0 5 10 20];
cfg.noiseMode = "networkAverageReceivedSNR";
cfg.noiseVarianceFixed = 1e-10;
cfg.clusterRadius = 60;

cfg.numRealizations = 100;
cfg.paperValidationTrials = 5000;
cfg.selectedAP = 1;
cfg.selectedUE = 1;
cfg.showFigures = true;
cfg.saveFigures = true;
cfg.saveResults = true;
cfg.runMonteCarlo = true;
cfg.runPaperValidation = true;
cfg.resultsDir = fullfile(fileparts(mfilename('fullpath')), 'results');

arraySpacingWasOverridden = false;
if nargin == 1 && isstruct(varargin{1})
    updates = varargin{1};
    arraySpacingWasOverridden = isfield(updates,'arraySpacing');
    names = fieldnames(updates);
    for i = 1:numel(names)
        cfg.(names{i}) = updates.(names{i});
    end
elseif mod(nargin,2) == 0
    for i = 1:2:nargin
        if strcmpi(char(varargin{i}),'arraySpacing')
            arraySpacingWasOverridden = true;
        end
        cfg.(char(varargin{i})) = varargin{i+1};
    end
elseif nargin ~= 0
    error('Overrides must be a struct or name/value pairs.');
end

cfg.lambda = cfg.c/cfg.fc;
if ~arraySpacingWasOverridden || isempty(cfg.arraySpacing)
    cfg.arraySpacing = cfg.lambda/2;
end
cfg.ulaDirection = cfg.ulaDirection/norm(cfg.ulaDirection);
end
