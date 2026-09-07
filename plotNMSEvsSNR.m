function fig = plotNMSEvsSNR(statsArray)
%PLOTNMSEVSSNR Figure 3: pilot SNR versus Monte-Carlo mean NMSE.
fig=figure('Name','図3: Pilot SNRと平均NMSE'); hold on;
for i=1:numel(statsArray)
    label=modeLabel(statsArray(i).pilotMode);
    plot(statsArray(i).snrdB,statsArray(i).meanNMSEdB,'o-', ...
        'LineWidth',1.5,'DisplayName',label);
end
grid on; xlabel('Pilot SNR [dB]'); ylabel('有効リンクの平均NMSE [dB]');
legend('Location','best');
title('図3: Pilot SNRに対するチャネル推定精度（Monte-Carlo平均）'); hold off;
end

function label=modeLabel(mode)
switch lower(char(mode))
    case 'orthogonal', label='直交Pilot';
    case 'randomreuse', label='ランダムPilot再利用';
    case 'clusteredreuse', label='クラスタ型Pilot再利用';
    otherwise, label=char(mode);
end
end
