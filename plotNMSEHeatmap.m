function fig = plotNMSEHeatmap(NMSE)
%PLOTNMSEHEATMAP Figure 6: M x K link NMSE.
nmseDB=10*log10(max(NMSE,eps));
fig=figure('Name','図6: リンク別NMSEヒートマップ');
imagesc(nmseDB,'AlphaData',~isnan(nmseDB)); set(gca,'Color',[0.75 0.75 0.75]);
axis xy; cb=colorbar; cb.Label.String='NMSE [dB]';
xlabel('UE番号'); ylabel('AP番号');
title('図6: AP-UEリンク別のLS推定NMSE（灰色 = 伝搬outage）');
end
