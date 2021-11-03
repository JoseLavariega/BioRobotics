function u = BezierCurve(ctrl_pt, t)

n = length(ctrl_pt);
u = 0;
    for i = 1:n
        %Bezier Curve definition from Wikipedia
        % change to reflect indexing starts at 1
        factorial_term = factorial(n-1)/(factorial(i-1)*factorial(n-i));
        t_term         = (1-t)^(n-i)*t^(i-1)*ctrl_pt(i);
        u = u+factorial_term*t_term; % compute return value. Write your code instead of 1.
    end
    
    %disp(u);
end