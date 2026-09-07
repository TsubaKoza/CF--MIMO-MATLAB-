function fig = plotObstacleComparison(resultOff,resultOn)
%PLOTOBSTACLECOMPARISON Figure 4 using a common AP/UE layout and noise power.
fig=figure('Name','図4: 遮蔽物あり・なし比較');
values=10*log10(max([resultOff.meanNMSE resultOn.meanNMSE],eps));
subplot(1,2,1); bar(values);
set(gca,'XTickLabel',{'遮蔽物なし','遮蔽物あり'});
ylabel('有効リンクの平均NMSE [dB]'); grid on;
title('チャネル推定誤差');
subplot(1,2,2); bar(100*[resultOff.outageFraction resultOn.outageFraction]);
set(gca,'XTickLabel',{'遮蔽物なし','遮蔽物あり'});
ylabel('伝搬outage率 [%]'); ylim([0 100]); grid on;
title('有効なLoS・反射pathがないリンクの割合');
sgtitle('図4: 同一AP/UE配置・同一雑音での遮蔽物あり／なし比較');
end
