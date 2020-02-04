function [corrFig,cdata1] = createCorrFig(cdata1,sorted)
    % Creates a correlation figure for visualizing an input array.
    % Input: cdata1: a square matrix of r^2 values (output of corr(x,y))
    % Input: sorted: determines whether the resulting figure is sorted by row (1), sorted by row and then column (2), or unsorted (0)
    % Output: corrFig: figure handle of resulting figure
    
    [a,b] = size(cdata1);
    if a ~= b
        if b ~= 1
            error('Input matrix must be square. Run B = corr(A,A) then use B as input.')
        else
            disp('Plotting diagonal of corr matrix.')
        end
    end
    
    if nargin == 1
        sorted = 0;
    end
    
    if sorted ~= 0 && sorted ~= 1 && sorted ~= 2
        sorted = 0;
        disp('Sorted value must be 0, 1, or 2')
    end
    
    %% Check all open figures to see if planned figures are already open. Close all currently existing planned figures.
    figHandles = get(groot, 'Children');
    if ~isempty(figHandles)
        priorFigures = contains({figHandles(:).Name},{'CorrFig'});
        close(figHandles(priorFigures))
    end
    
    % Create figure
    corrFig = figure('name','CorrFig','Position',[-951,129,944,988]);

    % Create axes
    axes1 = axes('Parent',corrFig);
    hold(axes1,'on');
    clims = [min(cdata1,[],'all') max(cdata1,[],'all')];
    axisticker = 1:length(cdata1);
    ylabels = num2cell(axisticker);
    xlabels = ylabels;
    
    if sorted == 1
        [cdata1,newRowInds] = sortrows(cdata1,1,'descend');
        ylabels = num2cell(newRowInds);
    elseif sorted == 2
        [cdata1,newRowInds] = sortrows(cdata1,1,'descend');
        [cdata1,newColInds] = sortrows(cdata1',1,'descend');
        cdata1 = cdata1';
        ylabels = num2cell(newRowInds);
        xlabels = num2cell(newColInds);
    end
    
    % Create image
    image(cdata1,'Parent',axes1,'CDataMapping','scaled');
    title('Data Correlation','FontSize',15)

    % Uncomment the following line to preserve the X-limits of the axes
    % xlim(axes1,[0.5 10.5]);
    % Uncomment the following line to preserve the Y-limits of the axes
    % ylim(axes1,[0.5 10.5]);
    box(axes1,'on');
    axis(axes1,'ij');
    % Set the remaining axes properties
    xlim([.5 size(cdata1,2)+.5])
    ylim([.5 size(cdata1,1)+.5])
    set(gca,'xaxisLocation','top')
    pbaspect([b a 1])
    set(axes1,'CLim',clims,'Colormap',...
    [0 0 0.00520833333333333;0.0138888888888889 0.0138888888888889 0.0243055555555556;0.0277777777777778 0.0277777777777778 0.0434027777777778;0.0416666666666667 0.0416666666666667 0.0625;0.0555555555555556 0.0555555555555556 0.0815972222222222;0.0694444444444444 0.0694444444444444 0.100694444444444;0.0833333333333333 0.0833333333333333 0.119791666666667;0.0972222222222222 0.0972222222222222 0.138888888888889;0.111111111111111 0.111111111111111 0.157986111111111;0.125 0.125 0.177083333333333;0.138888888888889 0.138888888888889 0.196180555555556;0.152777777777778 0.152777777777778 0.215277777777778;0.166666666666667 0.166666666666667 0.234375;0.180555555555556 0.180555555555556 0.253472222222222;0.194444444444444 0.194444444444444 0.272569444444444;0.208333333333333 0.208333333333333 0.291666666666667;0.222222222222222 0.222222222222222 0.310763888888889;0.236111111111111 0.236111111111111 0.329861111111111;0.25 0.25 0.348958333333333;0.263888888888889 0.263888888888889 0.368055555555556;0.277777777777778 0.277777777777778 0.387152777777778;0.291666666666667 0.291666666666667 0.40625;0.305555555555556 0.305555555555556 0.425347222222222;0.319444444444444 0.319444444444444 0.444444444444444;0.333333333333333 0.338541666666667 0.458333333333333;0.347222222222222 0.357638888888889 0.472222222222222;0.361111111111111 0.376736111111111 0.486111111111111;0.375 0.395833333333333 0.5;0.388888888888889 0.414930555555556 0.513888888888889;0.402777777777778 0.434027777777778 0.527777777777778;0.416666666666667 0.453125 0.541666666666667;0.430555555555556 0.472222222222222 0.555555555555556;0.444444444444444 0.491319444444444 0.569444444444444;0.458333333333333 0.510416666666667 0.583333333333333;0.472222222222222 0.529513888888889 0.597222222222222;0.486111111111111 0.548611111111111 0.611111111111111;0.5 0.567708333333333 0.625;0.513888888888889 0.586805555555556 0.638888888888889;0.527777777777778 0.605902777777778 0.652777777777778;0.541666666666667 0.625 0.666666666666667;0.555555555555556 0.644097222222222 0.680555555555556;0.569444444444444 0.663194444444444 0.694444444444444;0.583333333333333 0.682291666666667 0.708333333333333;0.597222222222222 0.701388888888889 0.722222222222222;0.611111111111111 0.720486111111111 0.736111111111111;0.625 0.739583333333333 0.75;0.638888888888889 0.758680555555555 0.763888888888889;0.652777777777778 0.777777777777778 0.777777777777778;0.674479166666667 0.791666666666667 0.791666666666667;0.696180555555556 0.805555555555556 0.805555555555556;0.717881944444444 0.819444444444444 0.819444444444444;0.739583333333333 0.833333333333333 0.833333333333333;0.761284722222222 0.847222222222222 0.847222222222222;0.782986111111111 0.861111111111111 0.861111111111111;0.8046875 0.875 0.875;0.826388888888889 0.888888888888889 0.888888888888889;0.848090277777778 0.902777777777778 0.902777777777778;0.869791666666667 0.916666666666667 0.916666666666667;0.891493055555555 0.930555555555555 0.930555555555555;0.913194444444444 0.944444444444444 0.944444444444444;0.934895833333333 0.958333333333333 0.958333333333333;0.956597222222222 0.972222222222222 0.972222222222222;0.978298611111111 0.986111111111111 0.986111111111111;1 1 1],...
    'Layer','top','XTick',axisticker,'YTick',axisticker,'XTickLabels',xlabels,'YTickLabels',ylabels);
    colorbar
end