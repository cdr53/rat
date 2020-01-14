function [outk,r2scores,recompiled,W,H] = NMFsyncounter(forces)
    %results_cell = results_cell{2,5};
    
    if size(forces,1)>size(forces,2)
        forces = forces';
    end
    r2scores = zeros(38,1);
    k = 0;
    vafrec = [];
    
    while any(r2scores < .5) && k < 25
        k = k+1;
        [r2scores,recompiled,W,H] = NMFdecomposition(k,forces,0);
        vafrec = [vafrec,r2scores];
    end
    
    if k <= 10
        plotWH(forces,W,H,0);
        outk = k;
    else
        fprintf('Over 10 synergies\n')
        outk = k;
    end
end