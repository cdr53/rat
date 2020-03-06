function F = stPropSolver_func(x,r,Fo)

% x(1) = steepness
% x(2) = y_offset
%     % At -40mV, Am should equal STmaxfactor*Fmax
%     F(1) = (r*Fo)/(1+exp(-.01*x(1)))+x(2)-r*Fo;
%     % At -60mV, Am should equal 0
%     F(2) = (r*Fo)/(1+exp(.01*x(1)))+x(2);
    % At -40mV, Am should equal STmaxfactor*Fmax
    F(1) = x(2)+(r*Fo)*(1/(1+exp(-.01*x(1)))-1);
    % At -60mV, Am should equal 0
    F(2) = (r*Fo)/(1+exp(.01*x(1)))+x(2);
end