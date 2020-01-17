function plotWH(forces,W,H,saver)
    scrSz = get(groot, 'ScreenSize');
    scW = scrSz(3);
    
    [r,c] = size(forces);
    
    if r>c
        forces = forces';
    end
    
    relW = W./max(W);
    bigH = (H'.*max(W))';
    recompiled = (W*H)';
    
    figHandles = get(groot, 'Children');
    if ~isempty(figHandles)
        priorFigures = contains({figHandles(:).Name},{'SynFig','CFig','RecombFig'});
        close(figHandles(priorFigures))
    end
    axisticker = 1:38;
    synfig = figure('color','white','Position',[-(scW-10) 130 scW/2 985],'name','SynFig');
    cm = colormap(jet(3*size(W,2)));
    for i = 1:size(W,2)
        holder = relW(:,i);
        subplot(size(relW,2),1,i)
        bar(holder,'FaceColor',cm(i*3,:))
        set(gca,'XTick',axisticker,'TickLength',[.003 .003])
        if i == 1
            title('Relative Activation of Individual Muscles','FontSize',16)
        elseif i == 5
            xlabel('Muscle #','FontSize',14)
        end
        xlim([0 39])
        ylabel(num2str(i))
        ylim([0 1.1])
    end
    
    cfig = figure('color','white','Position',[-(scW-10)/2 130 scW/2 985],'name','CFig');
    yLims = 1.1.*[min(bigH,[],'all') max(bigH,[],'all')];
    for j = 1:size(bigH,1)
        holder = bigH(j,:);
        subplot(size(bigH,1),1,j)
        plot(linspace(0,100,size(bigH,2)),holder,'LineWidth',3,'Color',cm(j*3,:))
        ylim(yLims)
        ylabel(num2str(j))
        if j == 1
            title('Synergy Activation During Stride','FontSize',16)
        elseif j == 5
            xlabel('% Stride','FontSize',14)
        end
    end


    recombfig = figure('color','white','Position',[50 50 800 700],'name','RecombFig');
    subplot(3,1,1)
        plot(linspace(0,100,size(forces,2)),forces')
        title('Original Signal')
    subplot(3,1,2)
        plot(linspace(0,100,size(forces,2)),recompiled)
        title('Recompiled Signal from NNMF Components')
    subplot(3,1,3)
        differ = abs((forces'-recompiled));
        plot(linspace(0,100,size(forces,2)),differ)
        title('Difference Between Signals')
    
    if saver
        saveas(cfig,['G:\My Drive\Rat\SynergyControl\OutputFigures\Images\',datestr(datetime('now'),'yyyymmdd'),'_','actfig.png']);
        saveas(synfig,['G:\My Drive\Rat\SynergyControl\OutputFigures\Images\',datestr(datetime('now'),'yyyymmdd'),'_','wgtfig.png']);
        saveas(recombfig,['G:\My Drive\Rat\SynergyControl\OutputFigures\Images\',datestr(datetime('now'),'yyyymmdd'),'_','recfig.png']);
    end
end