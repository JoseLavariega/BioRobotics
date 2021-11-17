function u = BezierCurve(ctrl_pt, t)

ctrl_pt;

n = length(ctrl_pt)-1;
u = 0;
for k = 0:n
    u = u + ctrl_pt(:,k+1) * nchoosek(n,k) * ( t.^k .* (1-t).^(n-k));
end

u = [u; u]; %just trying to get the sim to run, this is WRONG
end