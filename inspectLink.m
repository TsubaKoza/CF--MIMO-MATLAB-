function inspectLink(result,m,k,makePlots)
%INSPECTLINK Explain the geometry, paths, pilot observation, and estimate.
if nargin<4, makePlots=true; end
fprintf('\n=== リンク詳細: AP %d - UE %d ===\n',m,k);
fprintf('AP位置: [%g %g %g] m\n',result.APpos(m,:));
fprintf('UE位置: [%g %g %g] m\n',result.UEpos(k,:));
info=result.pathInfo(m,k);
fprintf(['LoS（直接波）: %s; 有効な1回反射path数: %d; ' ...
    '有効な1回回折path数: %d\n'], ...
    string(info.hasLoS),info.numReflections,info.numDiffractions);
if ~info.hasLoS
    fprintf('遮断した遮蔽物番号: %s\n',mat2str(info.blockingObstacleIndices));
end
for l=1:numel(info.paths)
    q=info.paths(l);
    fprintf('Path %d: %s, 経路長=%.6g m, 複素利得=%+.4e%+.4ej\n', ...
        l,q.type,q.length,real(q.complexGain),imag(q.complexGain));
    if q.type=="reflection"
        fprintf('  反射点=[%.4g %.4g %.4g], 遮蔽物=%d, 反射面=%s\n', ...
            q.reflectionPoint,q.obstacleIndex,q.faceName);
    elseif q.type=="diffraction"
        fprintf(['  回折点=[%.4g %.4g %.4g], 遮蔽物=%d, edge=%s, ' ...
            'v=%.4g, 回折損失=%.3f dB\n'],q.diffractionPoint, ...
            q.obstacleIndex,q.edgeName,q.diffractionParameter,q.diffractionLossdB);
    end
    fprintf('  方位角=%.4g rad, 仰角=%.4g rad\n', ...
        q.AoD.azimuth,q.AoD.elevation);
end
disp('真のチャネル h_true ='); disp(result.h_true(:,m,k));
disp('Pilot相関後のAP観測 y_mk ='); disp(result.y_mk(:,m,k));
disp('LS推定チャネル h_hat_LS ='); disp(result.h_hat_LS(:,m,k));
if result.outageMask(m,k)
    fprintf('NMSE = 未定義（有効な伝搬pathがないoutageリンク）\n');
else
    fprintf('NMSE = %.6g (%.3f dB)\n',result.NMSE(m,k), ...
        10*log10(max(result.NMSE(m,k),eps)));
end
if makePlots
    plotScenario3D(result.scenario,result.pathInfo,m,k,result.cfg);
    plotChannelComparison(result.h_true,result.h_hat_LS,m,k);
end
end
