function [p,pilotAssignment] = generatePilots(cfg,K,UEpos)
%GENERATEPILOTS Unit-norm DFT pilots, dimension tau_p x K.
tauP = cfg.tauP;
if strcmpi(cfg.pilotAssignmentMode,"orthogonal")
    tauP = max(tauP,K);
end
n = (0:tauP-1).'; q = 0:tauP-1;
basis = exp(-1j*2*pi/tauP*(n*q))/sqrt(tauP);
pilotAssignment = assignPilots(cfg.pilotAssignmentMode,tauP,K,UEpos,cfg);
p = basis(:,pilotAssignment);
assert(isequal(size(p),[tauP K]));
assert(max(abs(sum(abs(p).^2,1)-1)) < 1e-12);
end
