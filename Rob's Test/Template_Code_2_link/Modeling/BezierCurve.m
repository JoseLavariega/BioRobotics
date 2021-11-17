function u = BezierCurve(ctrl_pt, t)

ctrl_pt;

n = length(ctrl_pt)-1;
u = 0;
for k = 0:n
    u = u + ctrl_pt(:,k+1) * nchoosek(n,k) * ( t.^k .* (1-t).^(n-k));
end

% u = 0;
%     for i = 1:n
%         u = (u + ctrl_pt(:, i+1) * nchoosek(n,i) .* (t.^i.*(1-t).^(n-i))); % compute return value. Write your code instead of 1.
% %     u = u + nchoosek(n,i-1).*(t.^(i-1)).*((1-t).^(n-i+1));
%     end
end


% 
% function vals = polyval_bz(pts, ts, deriv_order)
%     % vals = polyval_bz(pts, ts, deriv_order)
%     %        Evaluates bezier spline with points pts at times ts.
%     %        Optionally evalulates the derivative of deriv_order if passed
%     %
%     % pts => Columns are waypoints [p1 p2 p3 p4 ... pM]
%     % ts  => Time values as a row vector
%     % derive_order => Optional derivative order
%     N = size(pts,2)-1;
%     M = size(pts,1);
%     if nargin < 3
%         deriv_order = 0;
%     end
%     for i = 1:deriv_order
%         pts = N * diff(pts')';
%         N   = N-1;
%     end
%     vals = zeros(M,length(ts));
%     for k = 0:N
%         vals = vals + pts(:,k+1) * nchoosek(N,k) * ( ts.^k .* (1-ts).^(N-k));
%     end
% end


% Np = size(P, 1); 
% u = linspace(0, 1, N);
% B = zeros(N, Np);
% for i = 1:Np
%    B(:,i) = nchoosek(Np,i-1).*(u.^(i-1)).*((1-u).^(Np-i+1)); %B is the Bernstein polynomial value
% end
% B1 = (nchoosek(Np,Np).*(u.^Np))';
% S = B*P + B1*P(Np,:);
% y = S(:, 2);