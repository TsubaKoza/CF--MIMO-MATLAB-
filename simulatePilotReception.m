function [Y,W] = simulatePilotReception(h_true,p,pilotEnergy,noiseVariance,activeUE)
%SIMULATEPILOTRECEPTION Generate AP observations Y (N_AP x tau_p x M).
% Y_m = sum_k sqrt(E_p(k))*h_true(:,m,k)*p(:,k)^H + W_m.
[N,M,K] = size(h_true); tauP = size(p,1);
assert(size(p,2)==K);
if isscalar(pilotEnergy), pilotEnergy = repmat(pilotEnergy,1,K); end
if nargin < 5 || isempty(activeUE), activeUE = true(1,K); end
Y = complex(zeros(N,tauP,M));
W = sqrt(noiseVariance/2)*(randn(N,tauP,M)+1j*randn(N,tauP,M));
for m=1:M
    for k=1:K
        if activeUE(k)
            Y(:,:,m) = Y(:,:,m) + sqrt(pilotEnergy(k))* ...
                h_true(:,m,k)*p(:,k)';
        end
    end
    Y(:,:,m) = Y(:,:,m)+W(:,:,m);
end
assert(size(Y,1)==N && size(Y,2)==tauP && size(Y,3)==M);
end
