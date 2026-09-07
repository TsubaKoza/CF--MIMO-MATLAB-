function h_hat_LS = estimateChannelLS(Y,p,pilotEnergy,activeUE)
%ESTIMATECHANNELLS AP-local LS from received pilots only (never h_true).
% For unit-norm pilots, hhat_mk = (Y_m*p_k)/sqrt(E_p(k)).
y_mk = despreadPilot(Y,p);
[N,M,K] = size(y_mk);
if isscalar(pilotEnergy), pilotEnergy = repmat(pilotEnergy,1,K); end
if nargin < 4 || isempty(activeUE), activeUE=true(1,K); end
h_hat_LS = complex(NaN(N,M,K),NaN(N,M,K));
for k=1:K
    if activeUE(k)
        h_hat_LS(:,:,k) = y_mk(:,:,k)/sqrt(pilotEnergy(k));
    end
end
assert(size(h_hat_LS,1)==N && size(h_hat_LS,2)==M && size(h_hat_LS,3)==K);
end
