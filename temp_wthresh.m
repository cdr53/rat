clear meandiffs
count = 1;
wthreshs = linspace(0,.8,10);
stddiffs = zeros(length(wthreshs),1);
gif_plot = 0;

if gif_plot
    close all
    w_filename = [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','wthresh.gif'];
    h_filename = [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','hthresh.gif'];
    re_filename= [pwd,'\OutputFigures\Gifs\Wthresh\',datestr(datetime('now'),'yyyymmdd'),'_','recthresh.gif'];
    count2 = 0;
end

figHandles = get(groot, 'Children');
if ~isempty(figHandles)
    priorFigures = contains({figHandles(:).Name},{'surffig'});
    close(figHandles(priorFigures))
end
scrSz = get(groot, 'ScreenSize');
scW = scrSz(3);

meandiffs = zeros(length(wthreshs),10);
for tt = 1:10
    count = 1;
    for ii = wthreshs
        %inArray = forces;
        inArray = current2inject';
%         [~,recompiled,W,H] = NMFdecomposition(5,inArray,0,ii);
        [~,recompiled,W,H] = NMFdecomposition(tt,inArray,0,ii);
        differ = abs((inArray-recompiled));
        meandiffs(count,tt) = mean(mean(differ));
        stddiffs(count,tt) = std(mean(differ));
        count = count + 1;
        if tt == 5 && gif_plot
            plotWH(inArray,{W,ii},H,0)
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
end
    figure('name','surffig','Position',[-(scW-10)/2 130 scW/2 985])
    surf(1:10,wthreshs,meandiffs)
    pbaspect([1 1 1])
    view([-61 24])
    title('Differences in NNMF signals vs. Input Signals')
    xlabel('Number of Synergies')
    ylabel('Synergy Threshold Value')
    zlabel('Average Difference Between Signals')
    [minthresh,minsyn] = minmat(meandiffs);
    disp(['Minimum NNMF Error with ',num2str(minsyn),' synergies and a threshold value of ',num2str(wthreshs(minthresh)),'.'])
    
    function [ a,b ] = minmat(c)
        as=size(c);
        [~,I]=min(c(:));
        r=rem(I,as(1));
        a=r;
        b=((I-a)/as(1))+1;
        if a==0
            a=as(1);
            b=b-1;
        else
            a=r;
            b=b;
        end
    end
