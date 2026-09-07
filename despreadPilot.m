function y_mk = despreadPilot(Y,p)
%DESPREADPILOT Correlate each AP observation with each UE pilot.
[N,~,M] = size(Y); K = size(p,2);
y_mk = complex(zeros(N,M,K));
for m=1:M
    for k=1:K
        y_mk(:,m,k) = Y(:,:,m)*p(:,k);
    end
end
assert(size(y_mk,1)==N && size(y_mk,2)==M && size(y_mk,3)==K);
end
