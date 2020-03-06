function F = passivePropSolver_func(x,Lr,Fo)
    % Solving for ks, kp, and STmax such that when the muscle is maximally stimulated at steady state, the force produced is less than or equal to Fmax
    % Refers to work in notes from 3/2/2020
        %F(1) = Fo - (x(1)/(x(1)+x(2)))*(x(3)+(Lr^2)/(16*x(3))*(x(2)^2-x(3)^2));
        F(1) = Fo - (x(1)/(x(1)+x(2)))*(x(3)+(x(2)^2*Lr^2)/(16*x(3)));
end