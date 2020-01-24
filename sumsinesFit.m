function [fitresult, gof,sinnum] = sumsinesFit(time, inWave)
% Use the curve fitting tool to generate a sum-of-sines representation of the input waveform (r^2>=.97) 

[xData, yData] = prepareCurveData( time, inWave );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
temp = [0 0 -Inf 0 0 -Inf 0 0 -Inf 0 0 -Inf 0 0 -Inf 0 0 -Inf 0 0 -Inf 0 0 -Inf];

r2 = 0;
sinnum = 4;


    while r2 < .97 && sinnum < 9
        sinnum = sinnum+1;
        lower = temp(1:sinnum*3);
        typeStr = ['sin',num2str(sinnum)];
        ft = fittype(typeStr);
        opts.Lower = lower;
        % Fit model to data.
        [fitresult, gof] = fit( xData, yData, ft, opts );
        r2 = gof.rsquare;
    end
end


