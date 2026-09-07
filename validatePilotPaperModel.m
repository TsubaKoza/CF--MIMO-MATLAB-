function validation = validatePilotPaperModel(cfg)
%VALIDATEPILOTPAPERMODEL Independent reproduction of paper Eq. (5)-(7).
% This stochastic NLoS model is intentionally separate from RayTracingHybrid.
rng(cfg.seed+77,'twister');
N = cfg.N_AP; K = max(4,min(cfg.K,8)); tauP = min(3,K);
M = 1; %#ok<NASGU>
assignment = mod(0:K-1,tauP)+1;
n=(0:tauP-1).'; q=0:tauP-1;
basis=exp(-1j*2*pi/tauP*(n*q))/sqrt(tauP);
p=basis(:,assignment);
beta = 0.2+0.8*rand(1,K);
Ep = ones(1,K); N0 = 0.1;
target = 1; sharing = find(assignment==assignment(target));
cmk = sqrt(tauP*beta(target)*Ep(target))/ ...
    (tauP*sum(beta(sharing).*Ep(sharing))+N0);
predictedMSE = (tauP*sum(beta(setdiff(sharing,target)).* ...
    Ep(setdiff(sharing,target)))+N0)/ ...
    (tauP*sum(beta(sharing).*Ep(sharing))+N0);
sumError = 0;
for trial=1:cfg.paperValidationTrials
    hdot=(randn(N,K)+1j*randn(N,K))/sqrt(2);
    W=sqrt(N0/2)*(randn(N,tauP)+1j*randn(N,tauP));
    Y=W;
    for k=1:K
        % Paper Eq. (5): sqrt(tau_p*beta_mk*E_p,k) p_i(k)[n] hdot_mk.
        Y=Y+sqrt(tauP*beta(k)*Ep(k))*hdot(:,k)*p(:,k).';
    end
    ymk=Y*conj(p(:,target)); % Eq. (6), p_i(k)^*[n]
    hhat=cmk*ymk;
    sumError=sumError+norm(hhat-hdot(:,target))^2/N;
end
empiricalMSE=sumError/cfg.paperValidationTrials;
validation=struct('equation5ReceivedPilotImplemented',true, ...
    'equation6DespreadImplemented',true,'equation7Coefficient',cmk, ...
    'equation7PredictedMSE',predictedMSE,'empiricalMSE',empiricalMSE, ...
    'relativeError',abs(empiricalMSE-predictedMSE)/predictedMSE, ...
    'tauP',tauP,'assignment',assignment,'beta',beta,'N0',N0);
end
