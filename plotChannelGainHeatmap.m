function fig = plotChannelGainHeatmap(h_true)
%PLOTCHANNELGAINHEATMAP Figure 5: M x K true channel gain.
gain=squeeze(sum(abs(h_true).^2,1));
gainDB=10*log10(max(gain,eps)); outage=(gain<=eps); gainDB(outage)=NaN;
fig=figure('Name','図5: 真のチャネル利得ヒートマップ');
imagesc(gainDB,'AlphaData',~isnan(gainDB)); set(gca,'Color',[0.75 0.75 0.75]);
axis xy; cb=colorbar; cb.Label.String='真のチャネル利得 [dB]';
xlabel('UE番号'); ylabel('AP番号');
title('図5: AP-UEリンク別の真のチャネル利得（灰色 = 伝搬outage）');
end
