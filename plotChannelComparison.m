function fig = plotChannelComparison(h_true,h_hat,m,k)
%PLOTCHANNELCOMPARISON Figure 2: magnitude and phase over AP antennas.
fig=figure('Name','図2: 真のチャネルとLS推定チャネル');
n=(1:size(h_true,1)).';
subplot(2,1,1); plot(n,abs(h_true(:,m,k)),'o-','LineWidth',1.5); hold on;
plot(n,abs(h_hat(:,m,k)),'s--','LineWidth',1.5); grid on;
xlabel('APアンテナ番号 n'); ylabel('チャネル振幅 |h(n,m,k)|');
legend('真のチャネル h_true','LS推定チャネル h_hat_LS', ...
    'Interpreter','none','Location','best');
title(sprintf('図2: チャネル振幅の比較（AP %d - UE %d）',m,k));
subplot(2,1,2); plot(n,angle(h_true(:,m,k)),'o-','LineWidth',1.5); hold on;
plot(n,angle(h_hat(:,m,k)),'s--','LineWidth',1.5); grid on;
xlabel('APアンテナ番号 n'); ylabel('チャネル位相 [rad]');
legend('真のチャネル h_true','LS推定チャネル h_hat_LS', ...
    'Interpreter','none','Location','best');
title('チャネル位相の比較');
end
