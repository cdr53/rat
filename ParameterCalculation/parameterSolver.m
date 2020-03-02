function [ks,kp,stmax,steep,yoff] = parameterSolver(lr,fo)
%     newvals = cell(38,6);
%     muscles = obj.musc_obj;
%     parfor mnum = 1:length(muscles)
%         fo = muscles{mnum}.max_force;
%         lr = muscles{mnum}.RestingLength;
%         [ks,kp,stmax] = passivePropSolver(lr,fo);
%         [steep,yoff] = stPropSolver(stmax,fo);
%         newvals(mnum,:) = [{muscles{mnum}.muscle_name},{ks},{kp},{stmax},{steep},{yoff}];
%     end
%     
%     save([pwd,'\Data\passiveParams.mat'],newvals);

    [ks,kp,stmax] = passivePropSolver(lr,fo);
    [steep,yoff] = stPropSolver(stmax,fo);
    %newvals(mnum,:) = [{muscles{mnum}.muscle_name},{ks},{kp},{stmax},{steep},{yoff}];
end