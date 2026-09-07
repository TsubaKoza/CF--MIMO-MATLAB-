function h_hat = estimateChannelLMMSE(Y,p,pilotEnergy,noiseVariance,prior,activeUE)
%ESTIMATECHANNELLMMSE General LMMSE with explicit mean/covariance priors.
% prior.mean is N_AP x M x K; prior.covariance is N_AP x N_AP x M x K.
% This function is optional and is NOT enabled for RayTracingHybrid unless
% such priors are supplied from an independently justified ensemble.
y = despreadPilot(Y,p);
[N,M,K] = size(y);
if isscalar(pilotEnergy), pilotEnergy=repmat(pilotEnergy,1,K); end
if nargin<6 || isempty(activeUE), activeUE=true(1,K); end
assert(size(prior.mean,1)==N && size(prior.mean,2)==M && size(prior.mean,3)==K);
assert(size(prior.covariance,1)==N && size(prior.covariance,2)==N && ...
    size(prior.covariance,3)==M && size(prior.covariance,4)==K);
h_hat = complex(NaN(N,M,K),NaN(N,M,K));
for m=1:M
    for k=1:K
        if ~activeUE(k), continue; end
        muY = zeros(N,1); Cy = noiseVariance*norm(p(:,k))^2*eye(N);
        for j=1:K
            if ~activeUE(j), continue; end
            rho = p(:,j)'*p(:,k);
            muY = muY + sqrt(pilotEnergy(j))*rho*prior.mean(:,m,j);
            Cy = Cy + pilotEnergy(j)*abs(rho)^2*prior.covariance(:,:,m,j);
        end
        rhoKK = p(:,k)'*p(:,k);
        Rk = prior.covariance(:,:,m,k);
        crossCov = sqrt(pilotEnergy(k))*conj(rhoKK)*Rk;
        h_hat(:,m,k) = prior.mean(:,m,k) + crossCov/Cy*(y(:,m,k)-muY);
    end
end
end
