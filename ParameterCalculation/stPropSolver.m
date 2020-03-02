function [steep,yoff] = stPropSolver(stmax,Fo)
% For a given resting length and optimal muscle force, calculate Kse, Kpe, and Am based on a pre-defined LT relationship (stored in equilsolver_eqn)  
% Input: Lr: optimal resting length in meters
% Input: Fo: optimal force in Newtons
% Output: params: an 1x4 array containing [muscle names, Kse, Kpe, Am]
% Output: soln: (~)x3 solution array containing other possible solutions to the optimization equation

    % Test data:
        % BFP data in m and N
        %     Lr = .0376;
        %     Fo = 12.49;
    
    % Optimization cost function. Defines three equations for an LT curve relationship
    stsolver = @(x) stPropSolver_func(x,stmax/Fo,Fo);
    rng default % for reproducibility
    N = 100; % try 100 random start points
    pts = 1000*rand(N,2);
    soln = zeros(N,2); % allocate solution
    fval = zeros(N,2); % allocate solution
    opts = optimoptions('fsolve','Algorithm','levenberg-marquardt','Display','none');
    parfor k = 1:N
        [soln(k,:),fval(k,:)] = fsolve(stsolver,pts(k,:),opts); % find solutions
    end
       
    [~,minfvalind] = min(sum(abs(fval),2));
     steep = soln(minfvalind,1);
     yoff = soln(minfvalind,2);
end