clear meandiffs
count = 1;
wthreshs = linspace(0,.8,10);
meandiffs = zeros(length(wthreshs),1);
stddiffs = zeros(length(wthreshs),1);
gif_plot = 1;

if gif_plot
    close all
    w_filename = [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','wthresh.gif'];
    h_filename = [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','hthresh.gif'];
    re_filename= [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','recthresh.gif'];
    count2 = 0;
end

for tt = 1:10
    meandiffs = zeros(length(wthreshs),1);
    for ii = wthreshs
        [~,recompiled,W,H] = NMFdecomposition(5,forces,0,ii);
        differ = abs((forces-recompiled));
        meandiffs(count,1) = mean(mean(differ));
        stddiffs(count,1) = std(mean(differ));
        count = count + 1;
        if tt == 1 && gif_plot
            plotWH(forces,{W,ii},H,0)
            count2 = count2 +1;
            figHandles = get(groot, 'Children');
            if ~isempty(figHandles)
                w_fig = contains({figHandles(:).Name},{'SynFig'});
                h_fig = contains({figHandles(:).Name},{'CFig'});
                re_fig = contains({figHandles(:).Name},{'RecombFig'});
            end
            % Capture the plot as an image
            figs = {w_fig;h_fig;re_fig};
            figfiles = {w_filename;h_filename;re_filename};
            for jj = 1:length(figs)
                fighand = figs{jj};
                frame = getframe(figHandles(fighand)); 
                im = frame2im(frame); 
                [imind,cm] = rgb2ind(im,256); 
                % Write to the GIF File 
                if count2 == 1 
                  imwrite(imind,cm,figfiles{jj},'gif', 'Loopcount',inf); 
                else 
                  imwrite(imind,cm,figfiles{jj},'gif','WriteMode','append'); 
                end
            end
        end
    end
    count = 1;
    meandiffs = [wthreshs',meandiffs,stddiffs];
    
    if tt ==1
        figure
    end
    plot(meandiffs(:,1),meandiffs(:,2))
    hold on
end
    title('Effect of Synergy Thresholding on Signal Error')
    xlabel('Synergy Threshold Value')
    ylabel('Average Difference Difference from Original Signal')
