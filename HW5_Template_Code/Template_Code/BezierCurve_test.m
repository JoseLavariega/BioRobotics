close all
clc
%% Bezier Curve Test

t = 0;
i = 1;

% Control Points (Please try many different ones)
ctrl_pt = [0, 10, 0, 3, 10];

% The time locations of each control points
ctrl_pt_x = linspace(0,1, length(ctrl_pt));

for d = 0:0.01:1 
    u(i) = BezierCurve(ctrl_pt, d);
    %disp(u);
    %disp('u(i)');
    %disp(u(i));
    %disp(i);
    %print(u(i));
    %disp('t(i)');
    t(i) = d;
    %disp(t(i));
    i = i+1;
    %disp(i);
    
    %disp(length(t));
    %disp(length(u));
end

%disp(length(t));
%disp(length(u));
%disp(u);
figure
plot(t, u(1:101))
hold on
plot(ctrl_pt_x, ctrl_pt, 'o');
hold off