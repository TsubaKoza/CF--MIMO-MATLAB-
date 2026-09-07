function [NMSE,meanNMSE,outageMask,outageFraction] = computeNMSE(h_hat,h_true,epsilon)
%COMPUTENMSE Link-wise M x K NMSE and propagation-outage separation.
% A link with essentially zero ground-truth power has no meaningful NMSE:
% dividing receiver noise by epsilon would make the network average explode.
% Such links are retained as NaN in NMSE and reported through outageMask.
[~,M,K] = size(h_true); NMSE = NaN(M,K); outageMask = false(M,K);
for m=1:M
    for k=1:K
        if any(isnan(h_hat(:,m,k))), continue; end
        truePower = norm(h_true(:,m,k))^2;
        if truePower <= epsilon
            outageMask(m,k) = true;
            continue;
        end
        NMSE(m,k) = norm(h_hat(:,m,k)-h_true(:,m,k))^2/truePower;
    end
end
valid = NMSE(~isnan(NMSE));
if isempty(valid), meanNMSE=NaN; else, meanNMSE=mean(valid); end
outageFraction = nnz(outageMask)/numel(outageMask);
end
