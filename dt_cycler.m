fclose('all');
clear original_text simData sim_file dtbarrier

%dtvals = (10:.3:30).*1e-3;
dtbarrier = (2*10)/(74.81+964.89);
dtvals = round([.00054,(dtbarrier+.00054)/2,dtbarrier],5);
sim_file = "G:\My Drive\Rat\SynergyControl\Animatlab\SynergyWalking\SynergyWalking_reduced_Standalone.asim";
% original_text = importdata(sim_file);
% 
% clear tdata tddata
% for ii = 1:length(dtvals)
%     new_line = ['<PhysicsTimeStep>',num2str(dtvals(ii)),'</PhysicsTimeStep>'];
%     original_text{find(contains(original_text,'<PhysicsTimeStep'),1,'first')} = new_line;
%     fileID = fopen(sim_file,'w');
%     fprintf(fileID,'%s\n',original_text{:});
%     fclose(fileID);
%     simData = processSimData(sim_file); 
%     tensiondata = simData(7).data(:,3);
%     tdotdata = simData(7).data(:,1);
%     tdata(1,ii) = dtvals(ii);
%     tdata(2:length(tensiondata)+1,ii) = tensiondata;
%     tddata(1,ii) = dtvals(ii);
%     tddata(2:length(tdotdata)+1,ii) = tdotdata;
%     maxval(ii) = max(tddata(2:end,ii));
% end

% bb = trapz(abs(tddata(2:end,:)));
% semilogy(dtvals*1000,bb)
% hold on
% xline(dtbarrier*1000);
timevec = linspace(0,3.335,length(tddata)-1);
subcounter = 1;
h = gobjects(length(dtvals),1);
figure('Position',[100,100,1200,628])
for ii = 1:length(dtvals)
    h(ii) = subplot(length(dtvals),1,subcounter);
    plot(timevec,tddata(2:end,ii),'Color','k','LineWidth',2)
    set(h(ii),'FontSize',12)
    set(h(ii),'FontWeight','Bold')
    if ii==1
        title('Rate Change of Tension','FontSize',16)
        sizer = h(ii).Position;
    end
    xlabel("Time (s)"+newline+"",'FontSize',12)
    ylabel('N/s')
    ylim([-2 2].*10e3)
    xlim([0 max(timevec)])
    legend([num2str(dtvals(ii)*1000),' ms'],'Location','eastoutside','FontSize',12)        
    pos = get(h(ii),'Position');
    set(h(ii),'Position',[pos(1)-.07,pos(2),sizer(3:4)]);
    drawnow
    subcounter = subcounter + 1;
end

% semilogy(dtvals*1000,maxval,'LineWidth',2)
% xlabel('dt values in ms')
% ylabel('Maximum value of the rate change of tension')

% subplot(3,1,1)
% plot(tddate(2:end,1))
% ylabel(num2str(tddate(1,1)))
% subplot(3,1,2)
% plot(tddate(2:end,31))
% ylabel(num2str(tddate(1,31)))
% subplot(3,1,3)
% plot(tddate(2:end,32))
% ylabel(num2str(tddate(1,32)))