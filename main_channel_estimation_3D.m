function result = main_channel_estimation_3D(cfg)
%MAIN_CHANNEL_ESTIMATION_3D Run the complete requested simulation pipeline.
if nargin<1, cfg=configChannelSim3D(); end
if strcmpi(cfg.channelModel,"pilotPaperStochastic")
    result=validatePilotPaperModel(cfg);
    disp(result); return;
end
rng(cfg.seed,'twister');
if ~cfg.showFigures
    previousVisibility=get(groot,'DefaultFigureVisible');
    set(groot,'DefaultFigureVisible','off');
    cleanup=onCleanup(@() set(groot,'DefaultFigureVisible',previousVisibility)); %#ok<NASGU>
end
if ~exist(cfg.resultsDir,'dir'), mkdir(cfg.resultsDir); end
reportToolboxes();

scenario=generateScenario3D(cfg);
scenarioOff=scenario; scenarioOff.obstacles=scenario.obstacles([]);
cfgOff=cfg; cfgOff.enableObstacles=false;
rng(cfg.seed+500,'twister'); resultOff=runSingleSimulation3D(cfgOff,scenarioOff);
rng(cfg.seed+500,'twister'); resultOn=runSingleSimulation3D(cfg,scenario,resultOff.noiseVariance);
result=resultOn;

m=min(cfg.selectedAP,size(result.APpos,1));
k=min(cfg.selectedUE,size(result.UEpos,1));
figs=gobjects(0);
figs(end+1)=plotScenario3D(result.scenario,result.pathInfo,m,k,cfg);
figs(end+1)=plotChannelComparison(result.h_true,result.h_hat_LS,m,k);
figs(end+1)=plotObstacleComparison(resultOff,resultOn);
figs(end+1)=plotChannelGainHeatmap(result.h_true);
figs(end+1)=plotNMSEHeatmap(result.NMSE);

mcStats=struct([]);
if cfg.runMonteCarlo
    modes=["orthogonal","randomReuse","clusteredReuse"];
    for i=1:numel(modes)
        oneStat=runMonteCarlo3D(cfg,modes(i),cfg.enableObstacles);
        if i==1
            % A fieldless struct([]) cannot receive a struct with fields by
            % subscripted assignment in MATLAB. Seed the array directly.
            mcStats=oneStat;
        else
            mcStats(i)=oneStat; %#ok<AGROW>
        end
    end
    figs(end+1)=plotNMSEvsSNR(mcStats);
end
paperValidation=[];
if cfg.runPaperValidation
    paperValidation=validatePilotPaperModel(cfg);
end
result.obstacleOff=resultOff;
result.monteCarlo=mcStats;
result.paperValidation=paperValidation;

if cfg.saveFigures
    names={'figure1_scenario3D','figure2_channel_comparison', ...
        'figure4_obstacle_comparison','figure5_channel_gain_heatmap', ...
        'figure6_nmse_heatmap'};
    if cfg.runMonteCarlo, names{end+1}='figure3_snr_nmse'; end
    for i=1:numel(figs)
        exportgraphics(figs(i),fullfile(cfg.resultsDir,[names{i} '.png']),'Resolution',180);
        savefig(figs(i),fullfile(cfg.resultsDir,[names{i} '.fig']));
    end
end
if cfg.saveResults
    outputFile=fullfile(cfg.resultsDir,'channel_estimation_3D_results.mat');
    save(outputFile,'result','cfg','-v7.3');
    fprintf('結果を保存しました: %s\n',outputFile);
end
fprintf('有効リンクの平均LS NMSE: %.6g (%.3f dB)\n',result.meanNMSE, ...
    10*log10(max(result.meanNMSE,eps)));
fprintf('伝搬outageリンク: %d / %d (%.2f%%)\n', ...
    nnz(result.outageMask),numel(result.outageMask),100*result.outageFraction);
inspectLink(result,m,k,false);
end
