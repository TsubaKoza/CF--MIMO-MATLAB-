function assignment = assignPilots(mode,tauP,K,UEpos,cfg)
%ASSIGNPILOTS Assign one of tauP orthogonal basis pilots to each UE.
switch lower(char(mode))
    case 'orthogonal'
        if tauP < K
            error('orthogonal mode requires tauP >= K.');
        end
        assignment = 1:K;
    case 'randomreuse'
        if tauP >= K
            warning('randomReuse with tauP >= K may not force contamination.');
        end
        assignment = randi(tauP,1,K);
        if K > 1 && tauP < K && numel(unique(assignment)) == K
            assignment(end) = assignment(1);
        end
    case 'clusteredreuse'
        % Extension: distance-weighted greedy coloring inspired by the
        % paper's goal. This is not its exact stochastic-geometry algorithm.
        assignment = zeros(1,K);
        [~,order] = sort(sum(squareformLocal(UEpos),2),'descend');
        for ii = 1:K
            k = order(ii);
            costs = zeros(1,tauP);
            for color = 1:tauP
                used = find(assignment==color);
                if isempty(used), costs(color)=0; else
                    d = vecnorm(UEpos(used,:)-UEpos(k,:),2,2);
                    costs(color) = sum(exp(-d/max(cfg.clusterRadius,eps)));
                end
            end
            minCost = min(costs);
            assignment(k) = find(costs==minCost,1,'first');
        end
    otherwise
        error('Unknown pilotAssignmentMode: %s',mode);
end
end

function D = squareformLocal(points)
K = size(points,1); D = zeros(K,K);
for i=1:K
    D(i,:) = vecnorm(points-points(i,:),2,2).';
end
end
