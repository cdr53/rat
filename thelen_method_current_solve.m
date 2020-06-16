function outI = thelen_method_current_solve(A,muscMat)
    solverOptions = optimoptions('fsolve','Display','none');
    outI = zeros(size(muscMat,1),1);
    for ii = 1:size(muscMat,1)
        STmax = muscMat(ii,5); S = muscMat(ii,6); xoff = muscMat(ii,7);
        STcurve = @(I) STmax/(1+exp(S*(xoff-(I-60)/1000)))-A(ii);
        outI(ii) = fsolve(STcurve,10,solverOptions);
    end
end