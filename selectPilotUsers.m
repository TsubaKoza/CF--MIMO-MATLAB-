function activeUE = selectPilotUsers(mode,pathInfo,K)
%SELECTPILOTUSERS Choose all UEs or paper-style NLoS-only UEs.
switch lower(char(mode))
    case 'allue'
        activeUE = true(1,K);
    case 'papernlosonly'
        activeUE = false(1,K);
        for k=1:K
            activeUE(k) = ~any([pathInfo(:,k).hasLoS]);
        end
    otherwise
        error('Unknown pilotUserMode: %s',mode);
end
end
