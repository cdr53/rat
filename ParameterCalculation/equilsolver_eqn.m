function F = equilsolver_eqn(x,Lr,Fo)
% This solver is used with equilsolver_func.m to calculate Ks, Kp, and Am values that satisfy a pre-defined FL curve
% Refers to notes taken on 5-13-2019

%     x(1) = ks;
%     x(2) = kp;
%     x(3) = Am;
    
% @ L=Lr, F=Fo
    F(1) = (x(3)./(1+x(2)./x(1)))-Fo;
% @ L=1.4Lr, F=.5Fo
    F(2) = ((.36.*x(3)+.4.*x(2).*Lr)./(1+x(2)./x(1)))-.5*Fo;
% @ L=1.5Lr, F=0
    F(3) = ((.5.*x(2).*Lr)./(1+x(2)./x(1)))-.01*Fo;
end