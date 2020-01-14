function coeffMat = pearsonTest(forces,recompiled,to_plot)
% Compares the centered Pearson coefficient (r, but most commonly represnted as r^2) to the VAF method from Torres-Olviedo 2006
% Meant for use after sorting force profiles into synergies
% Run NMFdecomposition first
% Input: forces: nx38 array of optimized forces
% Input: recompiled: nx38 array of forces recompiled from synergies(W*H')
% Output: coeffMat: array with 100*r^2 in column 1 and 100*VAF in column 2

close all

% Settings for testing the algorithms with 2 sine waves
off = 0;
mag = -.5;
omega = 1;
t = 0:.001:2*pi;

holder = 1;
for musc = 1:38
    x = forces(:,musc);
    y = recompiled(:,musc);
    
    %x = sin(t)';
    %y = mag*sin(omega*t+off)';

    n = length(x);
    thetaX = sqrt((1/n)*sum(x.^2));
    thetaY = sqrt((1/n)*sum(y.^2));

    % pearsonCorr is the centered Pearson coefficient (r)
    % This is equal to the off-diagonal value you'd get with corr(x,y)
    pearsonCorr = corrcoef(x,y);
    % VAF value follows the guidance of Torres-Olviedo 2006 and Zhang 2006 ("Uncentered (Absolute)...")
    VAFval = 100*(1/n)*sum((x./thetaX).*(y./thetaY));
    coeffMat(holder,1) = 100*pearsonCorr(1,2)^2;
    coeffMat(holder,2) = VAFval;
    holder = holder + 1;
end
    VAF = @(x,y) 100*(1/length(x))*sum((x./sqrt((1/length(x))*sum(x.^2))).*(y./sqrt((1/length(y))*sum(y.^2))));

    if to_plot
        figure('Position',[-1900 130 700 500])
        plot(x)
        xlabel('Optimized Forces')
        hold on
        plot(y)
        xlabel('Forces Recombined form Synergies')
    end
end